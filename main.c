#include <stdio.h>
#include "main.h"

#define SERIALPORT	"/dev/ttyUSB0"
#define BAUD		9600
#define ETH_NAME	"ens5"

#define NMEA_TMO	2000 /*milliseconds*/

typedef int16_t u16;
typedef int32_t u32;
typedef int64_t u64;

#define BIT(n)		(1 << n)

#define DEBUG		1

#define JL3XXX_DEVAD0	0
#define JL3XXX_READ_PULS	0x308
#define JL3XXX_READ_PULS_DATA	0x309
#define JL3XXX_READ_PULS_CMD	0x8e0e
#define JL3XXX_TOD_NS_WORD0	0x313
#define JL3XXX_TOD_NS_WORD1	0x314
#define JL3XXX_TOD_SEC_WORD0	0x315
#define JL3XXX_TOD_SEC_WORD1	0x316
#define JL3XXX_TOD_SEC_WORD2	0x317

#define PTP_TOD_STORE_COMP	0x2
#define PTP_TOD_STORE_ALL	0x3
#define PTP_TOD_CAPTURE_TIMER	0x4

#define JL3XXX_TOD_COMP_WORD0	0x31c
#define JL3XXX_TOD_COMP_WORD1	0x31d
#define JL3XXX_PTP_TOD_LOD_PTR_WORD0	0x310
#define JL3XXX_PTP_TOD_LOD_PTR_WORD1	0x311
#define JL3XXX_PTP_TOD_CTL0		0x312

#define JL3XXX_PPS_SYNC_MODE		BIT(11)
#define JL3XXX_MULTI_SYNC_MODE		BIT(2)
#define JL3XXX_TRIG_GEN_AMT_WORD0	0x202
#define JL3XXX_TRIG_GEN_AMT_WORD1	0x203
#define JL3XXX_CAP_VALID		BIT(8)

struct jl3xxx_tod_op {
	u16 tod_busy;
	u16 store_op;
	u16 time_array;
	u16 clk_valid;
	u16 domain_num;
};

static int tod_op_data(struct jl3xxx_tod_op *op)
{
	return ((op->tod_busy << 15) |
		(op->store_op << 12) |
		(op->time_array << 9) |
		(op->clk_valid << 8) |
		(op->domain_num));
}

static int wait_tod_done(void)
{
	int i;
	int busyStatus = 0;

	for (i = 0; i < 20; i++) {
		busyStatus = phy_c45_read(ETH_NAME, 0, 0x312);
		busyStatus &= BIT(15);
		if (!busyStatus)
			break;
		usleep(1000);
	}

	if (busyStatus)
		printf("tod opration failed!\n");

	return 0;
}

#define TOD_LOAD_DELAY 3000000
#define TAI_CLOCK_PERIOD 8
static int jl3xxx_ptp_set_tod_time(u64 seconds, u32 nano_seconds)
{
	struct jl3xxx_tod_op ops = {1, PTP_TOD_STORE_ALL, 0, 1, 0};
	u32 ns_lo, ns_hi;
	u32 tai_ptp_gt;
	u32 tod_load_ptr;

	u32 tod_ns = (nano_seconds + TOD_LOAD_DELAY) % 1000000000;
	u64 tod_s = seconds + (nano_seconds + TOD_LOAD_DELAY) / 1000000000;

	phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
		      JL3XXX_READ_PULS, 0x8e0e);
	ns_hi = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
	ns_lo = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
        tai_ptp_gt = (ns_hi << 16) | ns_lo;
	tod_load_ptr = tai_ptp_gt + (TOD_LOAD_DELAY / TAI_CLOCK_PERIOD);

	//TOD load poiner high word
	phy_c45_write(ETH_NAME, 0, 0x311, ((tod_load_ptr >> 16) & 0xffff));
	//TOD load pointer low word
	phy_c45_write(ETH_NAME, 0, 0x310, (tod_load_ptr & 0xffff));

	if (seconds) {
		phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
			      JL3XXX_TOD_SEC_WORD0, tod_s & 0xffff);
		phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
			      JL3XXX_TOD_SEC_WORD1, (tod_s >> 16) & 0xffff);
		phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
			      JL3XXX_TOD_SEC_WORD2, (tod_s >> 32) & 0xffff);
	}
	if (nano_seconds) {
		phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
			      JL3XXX_TOD_NS_WORD0, tod_ns & 0xffff);
		phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
			      JL3XXX_TOD_NS_WORD1, (tod_ns >> 16) & 0xffff);
	}

	/* tod control config 0*/
	phy_c45_write(ETH_NAME, 0, 0x312, tod_op_data(&ops));

	wait_tod_done();

	return 0;
}

static int jl3xxx_ptp_set_pps_seconds(u64 tod_s)
{
	phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
		      JL3XXX_TOD_SEC_WORD0, tod_s & 0xffff);
	phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
		      JL3XXX_TOD_SEC_WORD1, (tod_s >> 16) & 0xffff);
	phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
		      JL3XXX_TOD_SEC_WORD2, (tod_s >> 32) & 0xffff);

	return 0;
}

static int jl3xxx_ptp_get_tod_time(u64 *seconds, u32 *nano_seconds)
{
	struct jl3xxx_tod_op ops = {1, PTP_TOD_CAPTURE_TIMER, 0, 1, 0};
	u32 ns_lo, ns_hi;
	u64 s_lo, s_me, s_hi;

	/* config tod */
	phy_c45_write(ETH_NAME, 0, 0x312, tod_op_data(&ops));

	if (nano_seconds) {
		/* Read global time need command */
		/* need fix mdio indirect */
		phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
			      JL3XXX_READ_PULS, 0x8f13);
		ns_hi = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
		ns_lo = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
		(*nano_seconds) = (ns_hi << 16) | ns_lo;
	}

	if (seconds) {
		phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
			      JL3XXX_READ_PULS, 0x8f15);
		s_hi = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
		s_me = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
		s_lo = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
		(*seconds) = (s_hi << 32) | (s_me << 16) | s_lo;
	}

	return 0;
}

#define NS_PER_SEC 1000000000LL
static inline tmv_t timespec_to_tmv(struct timespec ts)
{
	tmv_t t;
	t.ns = ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
	return t;
}

static void *monitor_recv_status(void *arg)
{
	struct nmea_parser *np = nmea_parser_create();
	struct pollfd pfd = { -1, POLLIN | POLLPRI };
	struct timespec rxtime, tmo = { 2, 0 };
	char input[256], *ptr;
	int num, cnt, parsed;
	struct tod *t = arg;
	struct nmea_rmc rmc;
	struct timex ntx;

	printf("monitor_tod_status\n");
	memset(&ntx, 0, sizeof(ntx));
	ntx.modes = ADJ_NANO;

	while (true) {
		if (pfd.fd == -1) {
			pfd.fd = serial_open(SERIALPORT, BAUD, 0, 0);
			if (pfd.fd == -1) {
				fprintf(stderr, "serial open timeout!\n");
				clock_nanosleep(CLOCK_MONOTONIC, 0, &tmo, NULL);
				continue;
			}
		}
		num = poll(&pfd, 1, NMEA_TMO);
		clock_gettime(CLOCK_MONOTONIC, &rxtime);
		adjtimex(&ntx);
		if (num < 0) {
			fprintf(stderr, "poll failed\n");
			break;
		}
		if (!num) {
			fprintf(stderr, "nmea source timed out\n");
			close(pfd.fd);
			pfd.fd = -1;
			continue;
		}
		if (pfd.revents & POLLERR) {
			fprintf(stderr, "nmea source socket error\n");
			close(pfd.fd);
			pfd.fd = -1;
			continue;
		}
		if (!(pfd.revents & (POLLIN | POLLPRI))) {
			continue;
		}
		cnt = read(pfd.fd, input, sizeof(input));
		if (cnt <= 0) {
			fprintf(stderr, "failed to read from nmea source\n");
			close(pfd.fd);
			pfd.fd = -1;
			continue;
		}
		ptr = input;
		do {
			if (!nmea_parse(np, ptr, cnt, &rmc, &parsed)) {
				pthread_mutex_lock(&t->mutex);
				t->local_monotime = rxtime; //t1: 计算机开机时间
				t->local_utctime.tv_sec = ntx.time.tv_sec; //t2: utc时间s
				t->local_utctime.tv_nsec = ntx.time.tv_usec; //t2: utc时间ns
				t->rmc_utctime = rmc.ts; //t3 远端卫星时间t3
				//fprintf(stderr, "utctime: %ld\n", t->rmc_utctime.tv_sec);
				t->rmc_fix_valid = rmc.fix_valid;
				pthread_mutex_unlock(&t->mutex);
			}
			cnt -= parsed;
			ptr += parsed;
		} while (cnt);
	}
	if (pfd.fd != -1) {
		close(pfd.fd);
	}
	return NULL;
}

char *global_time_to_nmea(void)
{
	const char *nmea_format = "$GPZDA,%H%M%S.00,%d,%m,%Y,00,00*6c\r\n";
	time_t current_time_sec;
	struct tm *utc_time;
	static char time_string[80]; //carefully!!! should static
	u64 sec;

	jl3xxx_ptp_get_tod_time(&sec, NULL);
	sec -= 37; // 补偿闰秒
	current_time_sec = sec;
	utc_time = gmtime(&current_time_sec);
	strftime(time_string, sizeof(time_string), nmea_format, utc_time);
	printf("nmea_format: %s\n", time_string);

	return time_string;
}

static bool get_pps_rise_intr_flag(void)
{
	u16 retval;

	retval = phy_c45_read(ETH_NAME, 0, 0x306);
	retval  = (retval & 0xffff) & BIT(12);

	return (bool)retval;

}

static void *monitor_send_status(void *arg)
{
	struct tod *t = arg;

	while (true) {
		pthread_mutex_lock(&t->mutex);
		t->is_valid = get_pps_rise_intr_flag();
		pthread_mutex_unlock(&t->mutex);
		usleep(200000);
	}
	return NULL;
}

static int pps_rise_abs_offset(int ns)
{
	if (ns < 500000000) {
		return ns;
	} else {
		return 1000000000 - ns;
	}
}

#define FREE_RUN_COUNT	10
#define ADJ_OFFSET_TO_FREQ	50000
#define ADJ_FREQ_TO_OFFSET	1000000
static int check_clock_freq(struct tod *t)
{
	if (t->adj.pid.free_run_count < FREE_RUN_COUNT) {
#if DEBUG
		printf("free_run_count: %d\n", t->adj.pid.free_run_count);
#endif
		if (pps_rise_abs_offset(t->adj.last_freq) <
		    pps_rise_abs_offset(t->adj.freq))
			t->adj.pid.pos_count++;
		else
			t->adj.pid.neg_count++;

		t->adj.last_freq = t->adj.freq;
		t->adj.pid.free_run_count++;
		return 1;
	}
	return 0;
}

static int adj_state(struct tod *t)
{
	switch (t->adj.state) {
	case OFFSET_ADJ:
#if DEBUG
		printf("++++OFFSET_ADJ+++++: %d --- %d\n",
			pps_rise_abs_offset(t->adj.offset), t->adj.offset);
#endif
#if 0
		if (pps_rise_abs_offset(t->adj.offset) >
		    pps_rise_abs_offset(t->adj.last_offset))
			t->adj.offset_dir = NEGATIVE;
		else
			t->adj.offset_dir = POSITIVE;

		t->adj.offset_adj(&t->adj, t->adj.offset);
		t->adj.last_offset = t->adj.offset;
		if (pps_rise_abs_offset(t->adj.offset) < ADJ_OFFSET_TO_FREQ)
			t->adj.state = FREQ_ADJ;
#endif
		break;
	case FREQ_ADJ:
#if DEBUG
		printf("++++FREQ_ADJ+++++: %d --- %d\n",
			pps_rise_abs_offset(t->adj.freq), t->adj.freq);
#endif
#if 1
		if (t->adj.freq < 500000000) {
			t->adj.pid.pos_flag = true;
		} else {
			t->adj.pid.pos_flag = false;
		}
#endif
		if (check_clock_freq(t))
			break;

		if (!t->adj.pid.init_flag) {
			if ((t->adj.pid.pos_count > FREE_RUN_COUNT/2) &&
			     !t->adj.pid.pos_flag)
				t->adj.freq_dir = POSITIVE;
			else
				t->adj.freq_dir = NEGATIVE;
			t->adj.pid.init_flag = true;
		}
		t->adj.freq_adj(&t->adj, pps_rise_abs_offset(t->adj.freq));
		t->adj.last_freq = t->adj.freq;

		if (pps_rise_abs_offset(t->adj.freq) > ADJ_FREQ_TO_OFFSET) {
			t->adj.freq_adj(&t->adj, 0);
			t->adj.state = OFFSET_ADJ;
		}
		break;
	case NONE_ADJ:
		break;
	default:
		printf("Free Run!\n");
		break;
	}
	return 0;
}

static void *monitor_adj_status(void *arg)
{
	struct tod *t = arg;
	struct jl3xxx_tod_op ops = {1, PTP_TOD_CAPTURE_TIMER, 0, 1, 0};
	u32 ns_lo = 0, ns_hi = 0, nano_seconds =0;
	u16 val = 0;

	while (true) {
		pthread_mutex_lock(&t->mutex);
		val = phy_c45_read(ETH_NAME, 0, 0x20a);
		if (val & JL3XXX_CAP_VALID) {
			/* config tod */
			phy_c45_write(ETH_NAME, 0, 0x312, tod_op_data(&ops));

			/* Read global time need command */
			/* need fix mdio indirect */
			phy_c45_write(ETH_NAME, JL3XXX_DEVAD0,
				      JL3XXX_READ_PULS, 0x8e0a);
			ns_hi = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0,
					     JL3XXX_READ_PULS_DATA);
			ns_lo = phy_c45_read(ETH_NAME, JL3XXX_DEVAD0,
					     JL3XXX_READ_PULS_DATA);
			nano_seconds = (ns_hi << 16) | ns_lo;
			phy_c45_write(ETH_NAME, 0, 0x20a,
				      val & ~JL3XXX_CAP_VALID);
			// init adj state
			t->adj.freq = nano_seconds;
			t->adj.offset = nano_seconds;
			adj_state(t);
		} else {
			printf("pps rise capture is not valid!\n");
		}
		pthread_mutex_unlock(&t->mutex);
		usleep(1000000);
	}
	return NULL;
}

static int jl3xxx_ptp_adjust_tod_freq(enum tod_out_flag positive, int ppm)
{
	struct jl3xxx_tod_op ops = {1, PTP_TOD_STORE_COMP, 2, 1, 0};
	int freq, tod_compensation;

	if (ppm < 0)
		freq = -ppm;
	else
		freq = ppm;

	/*TODO */
	//tod_compensation = freq | 0x80000000;
	if (positive == NEGATIVE) {
		tod_compensation = freq | 0x80000000;
#if DEBUG
		printf("negtive!!!\n");
#endif
	} else {
		tod_compensation = freq;
#if DEBUG
		printf("postive!!!\n");
#endif
	}

	phy_c45_write(ETH_NAME, 0, JL3XXX_TOD_COMP_WORD0,
		      tod_compensation & 0xffff);
	phy_c45_write(ETH_NAME, 0, JL3XXX_TOD_COMP_WORD1,
		     (tod_compensation >> 16) & 0xffff);

	wait_tod_done();

	/* tod control config 0*/
	phy_c45_write(ETH_NAME, 0, 0x312, tod_op_data(&ops));

	wait_tod_done();

	return 0;
}

static void pid_init(struct jl3xxx_pid *pid) {
	struct jl3xxx_pid pid_cfg = {
		.set_offset = 0,
		.actual_offset = 0,
		.err = 0,
		.err_last = 0,
		.expand = 0,
		.integral = 0,
		//kp=38,  kp(60%~70%) = (22 ~ 26.5)
		//ki=0.8, ki(150%~180%) = (1.2 ~ 1.45)
		//kd=0, kd(30%~50%) = (0)
#if 0
		.kp = 10,	//kp * 60%
		.ki = 1.2,	//ki * 150%
		.kd = 0.2,	//kd * 30%
#endif
		.kp = 10,	//kp * 60%
		.ki = 1.2,	//ki * 150%
		.kd = 0.2,	//kd * 30%
	};
	*pid = pid_cfg;
}

static int jl3xxx_ptp_adjust_tod_time(char *phydev, bool positive, int offset)
{
	struct jl3xxx_tod_op ops = {1, PTP_TOD_CAPTURE_TIMER, 0, 1, 0};
	u32 cur_ptp_glb_time_w0;
	u32 cur_ptp_glb_time_w1;
	u32 tod_load_point;
	u64 tod_s;
	u32 tod_ns;
	u64 tmp_ts;
	u64 future_tod_s;
	u64 future_tod_ns;
	u32 ns_lo, ns_hi;
	u64 s_lo, s_me, s_hi;

	/* config tod */
	phy_c45_write(phydev, 0, 0x312, tod_op_data(&ops));

	wait_tod_done();

	cur_ptp_glb_time_w0 = phy_c45_read(phydev, 0, JL3XXX_PTP_TOD_LOD_PTR_WORD0);
	cur_ptp_glb_time_w1 = phy_c45_read(phydev, 0, JL3XXX_PTP_TOD_LOD_PTR_WORD1);
	tod_load_point = ((cur_ptp_glb_time_w1 << 16) | cur_ptp_glb_time_w0) +
			   TOD_LOAD_DELAY/TAI_CLOCK_PERIOD;
	cur_ptp_glb_time_w1 = (tod_load_point >> 16);
	cur_ptp_glb_time_w0 = tod_load_point & 0xffff;
	phy_c45_write(phydev, 0, JL3XXX_PTP_TOD_LOD_PTR_WORD0,
		      cur_ptp_glb_time_w0);
	phy_c45_write(phydev, 0, JL3XXX_PTP_TOD_LOD_PTR_WORD1,
		      cur_ptp_glb_time_w1);

	//get tod time
	phy_c45_write(phydev, JL3XXX_DEVAD0,
		      JL3XXX_READ_PULS, 0x8f13);
	ns_hi = phy_c45_read(phydev, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
	ns_lo = phy_c45_read(phydev, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
	phy_c45_write(phydev, JL3XXX_DEVAD0,
		      JL3XXX_READ_PULS, 0x8f15);
	s_hi = phy_c45_read(phydev, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
	s_me = phy_c45_read(phydev, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);
	s_lo = phy_c45_read(phydev, JL3XXX_DEVAD0, JL3XXX_READ_PULS_DATA);

	tod_s = (s_hi << 32) | (s_me << 16) | s_lo;
	tod_ns = (ns_hi << 16) | ns_lo;

	tmp_ts = tod_s * 1000000000 + tod_ns + offset  + TOD_LOAD_DELAY;

	future_tod_s = tmp_ts / 1000000000;
	future_tod_ns = tmp_ts % 1000000000;
	//set tod time
	phy_c45_write(phydev, JL3XXX_DEVAD0, JL3XXX_TOD_SEC_WORD0, future_tod_s & 0xffff);
	phy_c45_write(phydev, JL3XXX_DEVAD0, JL3XXX_TOD_SEC_WORD1, (future_tod_s >> 16) & 0xffff);
	phy_c45_write(phydev, JL3XXX_DEVAD0, JL3XXX_TOD_SEC_WORD2, (future_tod_s >> 32) & 0xffff);

	phy_c45_write(phydev, JL3XXX_DEVAD0, JL3XXX_TOD_NS_WORD0, future_tod_ns & 0xffff);
	phy_c45_write(phydev, JL3XXX_DEVAD0, JL3XXX_TOD_NS_WORD1, (future_tod_ns >> 16) & 0xffff);

	/* config tod */
	ops.store_op = PTP_TOD_STORE_ALL;
	phy_c45_write(phydev, 0, 0x312, tod_op_data(&ops));

	wait_tod_done();

	return 0;
}

static void incremental_pid(struct phy_adj *adj,
			    enum tod_out_flag phase_adj, int ppm)
{
	struct jl3xxx_pid *pid = &adj->pid;

	pid->actual_offset = ppm;
	pid->err = pid->set_offset - pid->actual_offset;

	if ((pid->err > 20) && (pid->err < -20))
		pid->err = 0;

	pid->expand = pid->kp * (pid->err - pid->err_last) + pid->ki * pid->err +
		       pid->kd * (pid->err - 2 * pid->err_last + pid->err_prev);
	pid->err_prev = pid->err_last;
	pid->err_last = pid->err;

	printf("pid_set_offset: %d\n", (int)pid->expand);
	jl3xxx_ptp_adjust_tod_freq(phase_adj, (int)pid->expand);
}

static void pid_realize(struct phy_adj *adj,
			enum tod_out_flag phase_adj, int ppm){
	struct jl3xxx_pid *pid = &adj->pid;

	if (pid->pos_flag) {
#if DEBUG
		printf("pos_flag: true\n");
#endif
		ppm = -ppm;
	} else {
#if DEBUG
		ppm = ppm;
		printf("pos_flag: false\n");
#endif
	}
	pid->actual_offset = ppm;
	pid->err = pid->set_offset - pid->actual_offset;
	pid->integral += pid->err;
	pid->expand = pid->kp * pid->err + pid->ki * pid->integral +
		       pid->kd * (pid->err - pid->err_last);
	pid->err_last = pid->err;
	pid->actual_offset = pid->expand * 1;
#if DEBUG
	printf("pid_set_offset: %d\n", (int)pid->actual_offset);
#endif

	jl3xxx_ptp_adjust_tod_freq(phase_adj, (int)pid->actual_offset);
}

static void tod_sync_to_pps_init(void)
{
	u16 val;

	val = phy_c45_read(ETH_NAME, 0, 0x330);
	val = val | BIT(6);
	val = val | BIT(7);
	phy_c45_write(ETH_NAME, 0, 0x330, val);

	val = phy_c45_read(ETH_NAME, 0, 0x20a);
	val = val | BIT(12);
	val = val & ~BIT(13);
	phy_c45_write(ETH_NAME, 0, 0x20a, val);

	val = phy_c45_read(ETH_NAME, 0, 0x200);
	val = val & ~JL3XXX_MULTI_SYNC_MODE;
	val = val | JL3XXX_PPS_SYNC_MODE;
	phy_c45_write(ETH_NAME, 0, 0x200, val);
}

static void tod_and_pps_out_init(void)
{
	u16 val;

	val = phy_c45_read(ETH_NAME, 0, 0x330);
	//val = val & ~BIT(6);
	//val = val | BIT(7);

	val = val | BIT(3);
	val = val | BIT(4);
	val = val & ~BIT(5);
	phy_c45_write(ETH_NAME, 0, 0x330, val);

	val = phy_c45_read(ETH_NAME, 0, 0x20a);
	val = val | BIT(12);
	val = val & ~BIT(13);
	phy_c45_write(ETH_NAME, 0, 0x20a, val);

	val = phy_c45_read(ETH_NAME, 0, 0x200);
	val = val & ~JL3XXX_MULTI_SYNC_MODE;
	val = val | JL3XXX_PPS_SYNC_MODE;
	val = val | BIT(9);
	phy_c45_write(ETH_NAME, 0, 0x200, val);
}

/*  Test 10MHz(100ns) generate clock of ToD compensated Timer */
static void tai_trig_gen_init(void)
{
	u16 val;

	phy_c45_write(ETH_NAME, 0, 0x202, 0);
	phy_c45_write(ETH_NAME, 0, 0x203, 100);

	val = phy_c45_read(ETH_NAME, 0, 0x200);
	val = val | BIT(0);
#if 0
	val = val & ~BIT(1);
#else
	val = val | BIT(1);
#endif
#if 0
	val = val & ~BIT(11);
#else
	val = val | BIT(11);
#endif
	phy_c45_write(ETH_NAME, 0, 0x200, val);


	val = phy_c45_read(ETH_NAME, 0, 0x200);
	printf("test_log0: 0x%x\n", val);
}

static void tod_pps_rise_plus_init(void)
{
	u16 val;

	val = phy_c45_read(ETH_NAME, 0, 0x205);
	val = val | BIT(8);
	val = val | BIT(9);
	val = val | BIT(10);

	val = val | BIT(12);
	val = val | BIT(13);
	val = val | BIT(14);
	val = val | BIT(15);
	phy_c45_write(ETH_NAME, 0, 0x205, val);
}

static int phy_freq_adj(struct phy_adj *adj, int freq)
{
	pid_realize(adj, adj->freq_dir, freq);
	//incremental_pid(adj, adj->freq_dir, freq);

	return 0;
}

static int phy_offset_adj(struct phy_adj *adj, int offset)
{
	int ppm;

	if (adj->offset_dir == NEGATIVE)
		ppm = -offset;
	else if (adj->offset_dir == POSITIVE)
		ppm = offset;
	else
		printf("error enum!\n");

	return jl3xxx_ptp_adjust_tod_time(ETH_NAME, adj->offset_dir, ppm);
}

static void tod_init(struct tod *t)
{
	pid_init(&t->adj.pid);
	t->adj.freq_adj = phy_freq_adj;
	t->adj.offset_adj = phy_offset_adj;
	t->adj.state = OFFSET_ADJ;
#if 0
	phy_c45_write(ETH_NAME, 0, 0x202, 0);
	phy_c45_write(ETH_NAME, 0, 0x203, 100);
#else
	phy_c45_write(ETH_NAME, 0, 0x202, 0);
	phy_c45_write(ETH_NAME, 0, 0x203, 0);
#endif
	tod_sync_to_pps_init();
	t->adj.freq_adj(&t->adj, 0);
	t->adj.pid.init_flag = false;
}

static void tod_send_init(struct tod *t)
{
	pid_init(&t->adj.pid);
	t->adj.freq_adj = phy_freq_adj;
	t->adj.offset_adj = phy_offset_adj;
	t->adj.state = OFFSET_ADJ;
#if 1
	phy_c45_write(ETH_NAME, 0, 0x202, 0);
	phy_c45_write(ETH_NAME, 0, 0x203, 100);
#endif
	tod_sync_to_pps_init();
	//tod_and_pps_out_init();
	//tai_trig_gen_init();
	tod_pps_rise_plus_init();
	t->adj.freq_adj(&t->adj, 0);
	t->adj.pid.init_flag = false;
}

int tod_recv(void)
{
	//tod 输入过程
	//1.开启线程不断获取serial的状态
	//2.获取serial的数据
	//3.tod协议解析成time count
	//4.往phy里面配置time count
	struct tod *t;
	int err;
	t = calloc(1, sizeof(*t));

	tod_init(t);
	pthread_mutex_init(&t->mutex, NULL);
	err = pthread_create(&t->recv_worker, NULL, monitor_recv_status, t);
	if (err) {
		printf("failed to create recv_worker thread\n");
		free(t);
		return -1;
	}
	err = pthread_create(&t->adj_worker, NULL, monitor_adj_status, t);
	if (err) {
		printf("failed to create adj_worker thread\n");
		free(t);
		return -1;
	}
	while(true) {
		pthread_mutex_lock(&t->mutex);
		jl3xxx_ptp_set_pps_seconds(t->rmc_utctime.tv_sec);
		pthread_mutex_unlock(&t->mutex);
		sleep(2);
	}
	return 0;
}

int tod_send(void)
{
	//tod 输出过程
	//1.读出phy的time count
	//2.将count转换位tod格式
	//3.创建线程不断的通过serial发送数据出去
	struct tod *t;
	int fd = -1;
	int err;
	char *nmea_data;
	t = calloc(1, sizeof(*t));

	tod_send_init(t);
	pthread_mutex_init(&t->mutex, NULL);
	err = pthread_create(&t->send_worker, NULL, monitor_send_status, t);
	if (err) {
		printf("failed to create worker thread\n");
		free(t);
		return -1;
	}

	while (true) {
#if 0
		if (fd == -1) {
			fd = serial_open(SERIALPORT, BAUD, 0, 0);
			if (fd == -1) {
				fprintf(stderr, "serial open timeout!\n");
				sleep(2);	// wait for 1 second
				continue;
			}
		}
#endif

		if (t->is_valid) {
			nmea_data = global_time_to_nmea();
			write(fd, nmea_data, strlen(nmea_data));
			usleep(1000000); //wait isn't valid
		}
	}
	close(fd);

	return 0;

}

static void usage()
{
	fprintf(stderr, "make send or make recv");
}

int main(int argc, char **argv)
{
	if (3 < argc || argc < 2) {
		usage();
		return 0;
	}
	if (argv[1][0] == 's') {
		tod_send();
	} else if (argv[1][0] == 'r') {
		tod_recv();
	} else {
		usage();
	}
	return 0;
}
