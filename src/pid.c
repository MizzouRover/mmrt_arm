#include "pid.h"

void upd_errsum_none(struct pid_state *state, uint32_t dt);
void upd_errsum_sgnchng(struct pid_state *state, uint32_t dt);
void upd_errsum_window(struct pid_state *state, uint32_t dt);
void upd_errsum_threshold(struct pid_state *state, uint32_t dt);

void (*upd_errsum[])(struct pid_state *state, uint32_t dt) = {
	upd_errsum_none,
	upd_errsum_sgnchng,
	upd_errsum_window,
	upd_errsum_threshold
};

int pid_init(struct pid_state *state, pid_wndp_mthd_e wdp_mthd, size_t iwinlen,
		double ithresh, double setp, double pos, unsigned long time)
{
	if (wdp_mthd == NONE && (iwinlen != 0 || ithresh != 0)) {
		return -1;
	}
	if (wdp_mthd == SGNCHNG && (iwinlen != 0 || ithresh != 0)) {
		return -1;
	}
	if (wdp_mthd == WINDOW && (iwinlen == 0 || ithresh != 0)) {
		return -1;
	}
	if (wdp_mthd == THRESHOLD && (iwinlen != 0 || ithresh == 0)) {
		return -1;
	}

	state->setp = setp;
	state->err[0] = setp - pos;
	state->err[1] = 0.0;
	state->errsum = 0.0;
	state->iwinlen = iwinlen;
	state->iwinp = iwinlen;
	state->ithresh = ithresh;
	state->derr = 0.0;
	state->time[0] = time;
	state->time[1] = 0.0;

	return 0;
}

double pid_eval(struct pid_params params, struct pid_state *state, double pos,
		unsigned long time)
{
	state->time[1] = state->time[0];
	state->err[1] = state->err[0];

	state->time[0] = time;
	state->err[0] = state->setp - pos;

	uint32_t dt = state->time[0] - state->time[1];

	upd_errsum[state->wndp_mthd](state, dt);

	state->derr = (state->err[0] - state->err[1]) / dt;

	return	(params.p * state->err[0])
		+ (params.i * state->errsum)
		- (params.d * state->derr);
}

void upd_errsum_none(struct pid_state *state, uint32_t dt)
{
	state->errsum += state->err[0] * dt;
	return;
}

void upd_errsum_sgnchng(struct pid_state *state, uint32_t dt)
{
	if (	(state->err[1] > 0.0 && state->err[0] < 0.0) || 
		(state->err[1] < 0.0 && state->err[0] > 0.0)) {
		state->errsum = 0;
	} else {
		state->errsum += state->err[0] * dt;
	}
	return;
}

void upd_errsum_window(struct pid_state *state, uint32_t dt)
{
	state->iwin[state->iwinp] = state->err[0] * dt;
	state->errsum += state->iwin[state->iwinp];
	state->iwinp = (state->iwinp + 1) % state->iwinlen;
	state->errsum -= state->iwin[state->iwinp];
	return;
}

void upd_errsum_threshold(struct pid_state *state, uint32_t dt)
{
	if (state->err[0] > state->ithresh) {
		state->errsum = 0;
	} else {
		state->errsum += state->err[0];
	}
	return;
}
