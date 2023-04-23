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

typedef struct {
	int64_t ns;
} tmv_t;

struct tod {
	enum tod_out_flag flag;
	pthread_t worker;
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
