#ifndef _MAIN_H
#define _MAIN_H

#include <pthread.h>
#include <poll.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/timex.h>
#include <unistd.h>
#include <errno.h>

#include "./serial/serial.h"
#include "./mdio/mdio-tool.h"
#include "./nmea/nmea.h"

enum tod_out_flag {
	NEGATIVE,
	POSITIVE,
};

enum adj_state {
	OFFSET_ADJ,
	FREQ_ADJ,
	NONE_ADJ,
};

typedef struct {
	int64_t ns;
} tmv_t;

struct jl3xxx_pid {
	float set_offset;	//定义设定值
	float actual_offset;	//定义实际值
	float err;		//定义偏差值
	float err_last;		//定义上一个偏差值
	float kp,ki,kd;		//定义比例、积分、微分系数
	float expand;		//定义扩大系数（控制执行器的变量）
	float integral;		//定义积分值
	int pos_freq_offset;	//正向频偏
	int neg_freq_offset;	//反向频偏
	int pos_count;		//正向计数
	int neg_count;		//反向计数
	bool pos_flag;
};

struct phy_adj {
	enum tod_out_flag freq_dir;
	enum tod_out_flag offset_dir;
	enum adj_state state;
	int last_freq;
	int last_offset;
	int offset;
	int freq;
	int (*freq_adj)(struct phy_adj *adj, int freq);
	int (*offset_adj)(struct phy_adj *adj, int offset);
	struct jl3xxx_pid pid;
	struct jl3xxx_pid neg_pid;
};

struct tod {
	enum tod_out_flag flag;
	pthread_t send_worker;
	pthread_t recv_worker;
	pthread_t adj_worker;
	bool is_valid;
	struct phy_adj adj;
	/* Protects anonymous struct fields, below, from concurrent access. */
	pthread_mutex_t mutex;

	struct {
		struct timespec local_monotime;
		struct timespec local_utctime;
		struct timespec rmc_utctime;
		bool rmc_fix_valid;
	};
};

#endif
