/**
 ***************************************************************************
 *   @file   bmsd.c
 *   @brief  Implementation of communication between Goodwe GW6000-EH inverter
 * 	     and BICM modules of Chevy Volt battery
 *   @author Yasko (yasko.solar@gmail.com)
 ***************************************************************************
 * 			Copyright (c) 2021 yasko.solar
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1.Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * 2.Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdint.h>
#include <sys/select.h>
#include <fcntl.h>
#include <sys/param.h>

#define TOTAL_CELL	96	/* total cells */
#define NSKIP		2	/*number of not used cells*/
#define SKIP_CELL1	23
#define SKIP_CELL2	28
#define NTEMP		16
#define MAX_CHG_V	4.06F
#define MIN_CELL_V	3.30F
#define MAX_SOC_V	4.15F
#define ICHG            10.0F
#define IDIS            25.0F
#define SOH		85.0F

#define VMAX_CHG	(TOTAL_CELL-NSKIP)*MAX_CHG_V
#define VMAX_SOC	(TOTAL_CELL-NSKIP)*MAX_SOC_V
#define VMIN		(TOTAL_CELL-NSKIP)*MIN_CELL_V
#define K_SOC		(1.0/(VMAX_SOC-VMIN))*100.0

/* safety levels */
#define OVER_V_L1	4.20F
#define OVER_V_L2	4.10F
#define UNDER_V_L1	3.00F
#define UNDER_V_L2	3.20F
#define OVER_T_L1	45.0F
#define OVER_T_L2	40.0F

/*BMS ALARM BITS*/
#define CHG_OV2		0
#define DIS_UV2		1
#define T_HIGH2		2

#define N_FRAMES	7
#define MAX_EVENTS	16


enum frames {
F_STR,
F_ALARM,
F_CHARGE,
F_SOC,
F_VBAT,
F_45A,
F_FLAGS
};

uint16_t frame_id[N_FRAMES] = {
	0x453,
	0x455,
	0x456,
	0x457,
	0x458,
	0x45A,
	0x460 };

int s0, s1;
struct can_frame frame[N_FRAMES];

struct can_frame rx_frame;
struct can_frame tx_frame;

struct ev_batt {
	int total_cell;
	int cell_count;
	int n_size[32];
	int skip_cell[8];
	float v1[TOTAL_CELL];
	float t1[NTEMP];
	float v_total;
	float v_avg;
	float v_delta;
	float v_min;
	float v_max;
	float t_max;
	float t_min;
	uint16_t n_vmin, n_vmax;
	uint16_t bms_alarm, bms_warn;
};

struct bms_ctrl{
	int f_vready;
	int firstd;
	int frun;
};

struct ev_batt bat1;
struct bms_ctrl b1_ctrl;


void print_stat();

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

/**
 * @brief Performs safety checks.
 * @param pbev - pointer to battery struct. 
 * @retval ret=0-no faults, ret=1 faults.
 */
int safety_check(struct ev_batt *pbev)
{
	int ret = 0;

	if (pbev->v_max >= OVER_V_L1){
		pbev->bms_alarm |= (1 << CHG_OV2);
		ret = 1;
	}
	else if (pbev->v_max < OVER_V_L2){
		pbev->bms_alarm &= ~(1 << CHG_OV2);
	}

	if (pbev->v_min <  UNDER_V_L1){
		pbev->bms_alarm |= (1 << DIS_UV2);
		ret = 1;
	}
	else if (pbev->v_min >=  UNDER_V_L2){
		pbev->bms_alarm &= ~(1 << DIS_UV2);
	}

	if (pbev->t_max >= OVER_T_L1){
		pbev->bms_alarm |= (1 << T_HIGH2);
		ret = 1;
	}
	else if (pbev->t_max < OVER_T_L2){
		pbev->bms_alarm &= ~(1 << T_HIGH2);
	}
	return ret;
}

/**
 * @brief Calculate cells statistics.
 * @param pbev - pointer to battery struct, 
 * 	  which contains battery data
 * @retval none.
 */
void cell_stat(struct ev_batt *pbev)
{
        int i, j=0;

	pbev->v_total = 0.0;
	pbev->v_min = 5.0;
	pbev->v_max = 0.0;
	pbev->t_max = -50.0;
	pbev->t_min = 100.0;

        for (i = 0;i < pbev->total_cell;i++){
                if ((j < NSKIP) && (i == (pbev->skip_cell[j]-1))){
                        j++;
                        continue;
                }
                pbev->v_total += pbev->v1[i];

                if (pbev->v1[i] < pbev->v_min){
                        pbev->v_min = pbev->v1[i];
                        pbev->n_vmin = i;
                }
                if (pbev->v1[i] > pbev->v_max){
                        pbev->v_max = pbev->v1[i];
                        pbev->n_vmax = i;
                }
        }
        pbev->v_avg = pbev->v_total/(float)(pbev->total_cell-j);
	pbev->v_delta = pbev->v_max - pbev->v_min;

	for (i=0;i<NTEMP;i++){
		if (pbev->t1[i] > pbev->t_max){
                	pbev->t_max =  pbev->t1[i];
        	}
		if (pbev->t1[i] < pbev->t_min){
                        pbev->t_min =  pbev->t1[i];
                }
	}

        if (b1_ctrl.firstd == 0){
		b1_ctrl.firstd = 1;
		print_stat();
	}
}


/**
 * @brief Prints cells statistics.
 * @retval none.
 */
void print_stat()
{
	printf("\r\n");
	printf("voltage: %.1fV,\r\n",bat1.v_total);
	printf("v_avg: %.3fV,\r\n",bat1.v_avg);
	printf("v_min: %.3fV,\r\n",bat1.v_min);
	printf("v_max: %.3fV,\r\n",bat1.v_max);
	printf("v_delta: %.3fV,\r\n",bat1.v_delta);
	printf("max_temp: %.1fdeg,\r\n",bat1.t_max);
}


/**
 * @brief Updates a parameter into a CAN frame.
 * @param id - CAN frame index 
 * @param pos - postion of parameter inside frame (0-3)
 * @param val - parameter value
 * @retval none.
 */
int update_uint(int id, int pos, int16_t val)
{
	uint8_t byte_l, byte_h;

	byte_l = val;
	byte_h = val >> 8;
	if  ((id >= N_FRAMES)||(pos>=4))
		return -1;

	 frame[id].data[pos*2] = byte_l;
	 frame[id].data[pos*2+1] = byte_h;
	return 0;
}

/**
 * @brief Sets constants parameters for battery
 * @retval none.
 */
void set_const()
{
	uint16_t tmp;

	tmp = (uint16_t)(VMAX_CHG * 10.0);	/*max charge voltage*/
	//printf("CHGVx10: %dV,\r\n",tmp);
	update_uint(F_CHARGE, 0, tmp);
	tmp = (uint16_t)(VMIN * 10.0);		/*min discahrge voltag*/
	//printf("DCHGVx10: %dV,\r\n",tmp);
	update_uint(F_CHARGE, 3, tmp);
	tmp = (uint16_t)(ICHG * 10.0);		/*max charge current*/
	//printf("CHGVx10: %dV,\r\n",tmp);
	update_uint(F_CHARGE, 1, tmp);
	tmp = (uint16_t)(IDIS * 10.0);		/*min discahrge current*/
	//printf("DCHGVx10: %dV,\r\n",tmp);
	update_uint(F_CHARGE, 2, tmp);
	tmp = (uint16_t)(SOH * 100.0);		//state  of health
	//printf("DCHGVx10: %dV,\r\n",tmp);
	update_uint(F_SOC, 1, tmp);
}


/**
 * @brief Sets real-time parameters for battery
 * @retval none.
 */
void update_par()
{
	uint16_t v, soc_i, tmp;
	float soc;

	v = (uint16_t)(bat1.v_total * 10.0);
	//printf("vx10: %dV,\r\n",v);
	update_uint(F_VBAT, 0, v);
	soc = (bat1.v_total-VMIN)*K_SOC;	/*batt volatge */
	//printf("soc: %.2fV,\r\n",soc);
	soc_i = (uint16_t)(soc * 100.0);	/*batt SOC*/
	update_uint(F_SOC, 0 ,soc_i);
	tmp = (uint16_t)(bat1.t_max * 10.0);	/*batt temperature */
	update_uint(F_VBAT, 2, tmp);
	update_uint(F_ALARM, 0, bat1.bms_alarm); /*battery alarms*/
	update_uint(F_ALARM, 2, bat1.bms_warn);  /*battery warnings*/
}

/**
 * @brief A helper function
 * @param n - ID
 * @retval  current index
 */
int get_start_id(int n)
{
        int i, ret = 0;
        for (i = 0; i<n; i++){
                ret+=bat1.n_size[i];
        }
        return ret;
}

/**
 * @brief Extracts and calculates cell voltages
 *	  from  BICM CAN frames
 * @param  pf pointer to received CAN frame
 * @retval none.
 */
void calc_voltage(struct can_frame *pf)
{
        int i, id, index, n;
	uint16_t tmp;

	id = pf->can_id - 0x460;
        n = pf->can_dlc/2;
        if (id >= 16){
                id = (id -16)*2 + 1;
        }
        else{
                id *=2;
        }
        bat1.n_size[id] = n;
        index = get_start_id(id);

        for (i = 0; i < n; i++){
                tmp =  ((pf->data[i*2]&0x0F)*256 + pf->data[i*2+1]);
                id = (TOTAL_CELL-1) - (index+i); /*chage order of the cells */
		bat1.v1[id] = (float)tmp*0.00125;
                //fprintf(stdout,"%d,%.3f\r\n",id+1, v1[id]);
                bat1.cell_count++;
         }
        //printf("\r\n");
}

/**
 * @brief Extracts and calculates cell temperatures
 *        from  BICM CAN frames
 * @param  pf pointer to received CAN frame
 * @retval none.
 */
void calc_temp(struct can_frame *pf)
{

        uint16_t tmp, ind;
        int i=0;

        ind = pf->can_id - 0x7E0;
        if (ind >= 16)
                return;

	switch(ind){
        case 0:
        case 1:
        case 8:
        case 12:
        case 13:
                i=0;
                break;
        case 2:
        case 5:
        case 9:
                i=1;
                break;
        default:
                return;
                break;
        }


        tmp =  ((pf->data[i*2]&0x0F)*256 + pf->data[i*2+1]);
        bat1.t1[ind] = 110.7 - (float)tmp*0.0294;
        //printf("%.1f\r\n",t1[ind]);
}


/**
 * @brief Init frames sent to the inverter
 * @retval none.
 */
void init_frames()
{
	frame[F_STR].can_id  = frame_id[F_STR];
        frame[F_STR].can_dlc = 8;
        frame[F_STR].data[0] = 'D';
        frame[F_STR].data[1] = 'F';

	frame[F_ALARM].can_id  = frame_id[F_ALARM];
        frame[F_ALARM].can_dlc = 8;

 	frame[F_CHARGE].can_id  = frame_id[F_CHARGE];
        frame[F_CHARGE].can_dlc = 8;

	frame[F_SOC].can_id  = frame_id[F_SOC];
        frame[F_SOC].can_dlc = 8;

	frame[F_VBAT].can_id  = frame_id[F_VBAT];
        frame[F_VBAT].can_dlc = 8;

	frame[F_45A].can_id  = frame_id[F_45A];
        frame[F_45A].can_dlc = 8;

	frame[F_FLAGS].can_id  = frame_id[F_FLAGS];
        frame[F_FLAGS].can_dlc = 2;
        frame[F_FLAGS].data[0] = 0x00;
        frame[F_FLAGS].data[1] = 0x00;
}


/**
 * @brief  Some setup before starting communication
 * @retval none.
 */
void setup()
{
	bat1.total_cell = TOTAL_CELL;
	bat1.skip_cell[0] = SKIP_CELL1;
	bat1.skip_cell[1] = SKIP_CELL2;
	bat1.bms_alarm = 0x00;
	bat1.bms_warn = 0x00;
	init_frames();
	set_const();
}

/**
 * @brief  Sends the battery data to the inverter
 * @retval none.
 */
void send_inverter_data()
{
	int nbytes;

	nbytes = write(s1, &frame[F_STR], sizeof(struct can_frame));
	nbytes = write(s1, &frame[F_ALARM], sizeof(struct can_frame));
	nbytes = write(s1, &frame[F_CHARGE], sizeof(struct can_frame));
	nbytes = write(s1, &frame[F_SOC], sizeof(struct can_frame));
	nbytes = write(s1, &frame[F_VBAT], sizeof(struct can_frame));
	nbytes = write(s1, &frame[F_45A], sizeof(struct can_frame));
	nbytes = write(s1, &frame[F_FLAGS], sizeof(struct can_frame));
}

/**
 * @brief  Process the frames coming from the inverter
 * @retval none.
 */
void proc_frame1(struct can_frame *pf)
{

	switch(pf->can_id){
	case 0x425:
		//send_resp();
		break;
	default:
		printf("0x%03X [%d]\r\n",pf->can_id, pf->can_dlc);
		break;
        }
}

/**
 * @brief  Process the frames coming from the battery
 * @retval none.
 */
void proc_frame0(struct can_frame *pf)
{
  	if ((pf->can_id & 0xF00) == 0x400){
                 calc_voltage(pf);
        }

	else if ((pf->can_id & 0xFF0) == 0x7E0){
                 calc_temp(pf);
        }
}

/**
 * @brief  Sends a query to the battery
 * @retval  nbytes humber of bytes sent
 */
int send_qr()
{
	int nbytes;

	tx_frame.can_id  = 0x200;
        tx_frame.can_dlc = 3;
        tx_frame.data[0] = 0x02;
        tx_frame.data[1] = 0x00;
        tx_frame.data[2] = 0x00;
        nbytes = write(s0, &tx_frame, sizeof(struct can_frame));
	return nbytes;
}

/**
 * @brief  This function control teh comunication
 *  It' called every 1 sec.
 * @retval  ret = 0
 */
int pr_1sec()
{
        //printf("cells: %d",cell_count);

	if  (bat1.cell_count >= bat1.total_cell){
		cell_stat(&bat1);
		safety_check(&bat1);
		update_par();
		b1_ctrl.f_vready = 1;
        }
	else{
		b1_ctrl.f_vready = 0;
	}

	if (b1_ctrl.f_vready == 1){
		send_inverter_data();
	}

	send_qr();

        bat1.cell_count = 0;
        return 0;
}


/**
 * @brief  The primary function that listens on CAN sockets.
 * @retval  ret 
 */
int listen_rx()
{
	int running = 1;
	int event_count;
	int  fd_epoll;
	int i,nbytes;
	struct epoll_event event_setup[2], events[MAX_EVENTS];

        fd_epoll = epoll_create(1);
        if (fd_epoll < 0) {
                perror("epoll_create");
                return 1;
        }

	event_setup[0].events = EPOLLIN;
        event_setup[0].data.fd = s0;

	event_setup[1].events = EPOLLIN;
        event_setup[1].data.fd = s1;

        if (epoll_ctl(fd_epoll, EPOLL_CTL_ADD, s0, &event_setup[0])) {
                perror("failed to add socket s0 to epoll");
                return 1;
        }

	if (epoll_ctl(fd_epoll, EPOLL_CTL_ADD, s1, &event_setup[1])) {
                perror("failed to add socket s0 to epoll");
                return 1;
        }


	while(running){
                event_count = epoll_wait(fd_epoll, events, MAX_EVENTS, 1000);
                //printf("%d ready events\n", event_count);
                if (event_count < 0){
                        running = 0;
                        continue;
                }
                if (event_count == 0){
			pr_1sec();
                        continue;
                }
                for(i = 0; i < event_count; i++){
                        //printf("Reading file descriptor '%d' -- ", events[i].data.fd);
                        nbytes = read(events[i].data.fd, &rx_frame, sizeof(struct can_frame));
			if (nbytes <= 0)
				continue;

			if (events[i].data.fd == s0){
                                 proc_frame0(&rx_frame);
			}
			else if (events[i].data.fd == s1){
                                 proc_frame1(&rx_frame);
                        	//printf("%zd bytes read.\n", nbytes);
			}
                 }
        }

	return 0;
}


/**
 * @brief  The main function.
 * @retval
 */
int main(void)
{
	/*int i,j;
	int nbytes; */
	struct sockaddr_can addr0, addr1;
	
	struct ifreq ifr0, ifr1;

	const char *ifname0 = "can0";
	const char *ifname1 = "can1";

	if((s0 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket 0");
		return -1;
	}

	if((s1 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket 1");
		return -1;
	}

	strcpy(ifr0.ifr_name, ifname0);
	ioctl(s0, SIOCGIFINDEX, &ifr0);

	strcpy(ifr1.ifr_name, ifname1);
	ioctl(s1, SIOCGIFINDEX, &ifr1);
	
	addr0.can_family  = AF_CAN;
	addr0.can_ifindex = ifr0.ifr_ifindex;
	
	addr1.can_family  = AF_CAN;
	addr1.can_ifindex = ifr1.ifr_ifindex;

//	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if(bind(s0, (struct sockaddr *)&addr0, sizeof(addr0)) < 0) {
		perror("Error in socket bind 0");
		return -2;
	}

	if(bind(s1, (struct sockaddr *)&addr1, sizeof(addr1)) < 0) {
		perror("Error in socket bind 1");
		return -2;
	}

	setup();
	send_qr();
	listen_rx();

	return 0;
}

