#ifndef __KALMAN_H
#define __KALMAN_H

/** Repurposed from https://github.com/simondlevy/TinyEKF

\brief An Extended Kalman Filter for low-latency, high-reliability localization, communication, and control.

\author Felipe Campos <fmrcampos@berkeley.edu>, May 2019.

 */

//=========================== define ==========================================

#define LH_RAD_VAR 0.01f
#define IMU_ACCEL_VAR 0.1f
#define NUM_STATES 2
#define NUM_OBS 1

//=========================== typedef =========================================

typedef struct {

    int n;           /* number of state values */
    int m;           /* number of observables */

    double x[NUM_STATES];     /* state vector */

    double P[NUM_STATES][NUM_STATES];  /* prediction error covariance */
    double Q[NUM_STATES][NUM_STATES];  /* process noise covariance */
    double R[NUM_OBS][NUM_OBS];  /* measurement error covariance */

    double G[NUM_STATES][NUM_OBS];  /* Kalman gain; a.k.a. K */

    double F[NUM_STATES][NUM_STATES];  /* Jacobian of process model */
    double H[NUM_OBS][NUM_STATES];  /* Jacobian of measurement model */

    double Ht[NUM_STATES][NUM_OBS]; /* transpose of measurement Jacobian */
    double Ft[NUM_STATES][NUM_STATES]; /* transpose of process Jacobian */
    double Pp[NUM_STATES][NUM_STATES]; /* P, post-prediction, pre-update */

    double fx[NUM_STATES];   /* output of user defined f() state-transition function */
    double hx[NUM_OBS];   /* output of user defined h() measurement function */

    /* temporary storage */
    double tmp0[NUM_STATES][NUM_STATES];
    double tmp1[NUM_STATES][NUM_OBS];
    double tmp2[NUM_OBS][NUM_STATES];
    double tmp3[NUM_OBS][NUM_OBS];
    double tmp4[NUM_OBS][NUM_OBS];
    double tmp5[NUM_OBS]; 

} ekf_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

#endif
