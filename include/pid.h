#ifndef PID_H
#define PID_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
	NONE = 0,
	SGNCHNG = 1,
	WINDOW = 2,
	THRESHOLD = 3
} pid_wndp_mthd_e;

struct pid_params {
	double p;
	double i;
	double d;
};

struct pid_state {
	pid_wndp_mthd_e wndp_mthd;
	double setp;
	double err[2];
	double *iwin;
	size_t iwinlen;
	size_t iwinp;
	double ithresh;
	double errsum;
	double derr;
	unsigned long time[2];
};

// Reset a pid controller state with params. Rejects invalid configs including
// non-zero threshold or window len in a non-matching windup mode
int pid_init(struct pid_state *state, pid_wndp_mthd_e wdp_mthd, size_t iwinlen,
		double ithresh, double setp, double pos, unsigned long time);

// Eval- relies on external pos tracking and outputs the result directly
double pid_eval(struct pid_params params, struct pid_state *state, double pos,
		unsigned long time);

#endif

