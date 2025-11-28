#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3838225519793418139) {
   out_3838225519793418139[0] = delta_x[0] + nom_x[0];
   out_3838225519793418139[1] = delta_x[1] + nom_x[1];
   out_3838225519793418139[2] = delta_x[2] + nom_x[2];
   out_3838225519793418139[3] = delta_x[3] + nom_x[3];
   out_3838225519793418139[4] = delta_x[4] + nom_x[4];
   out_3838225519793418139[5] = delta_x[5] + nom_x[5];
   out_3838225519793418139[6] = delta_x[6] + nom_x[6];
   out_3838225519793418139[7] = delta_x[7] + nom_x[7];
   out_3838225519793418139[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1940590978769469803) {
   out_1940590978769469803[0] = -nom_x[0] + true_x[0];
   out_1940590978769469803[1] = -nom_x[1] + true_x[1];
   out_1940590978769469803[2] = -nom_x[2] + true_x[2];
   out_1940590978769469803[3] = -nom_x[3] + true_x[3];
   out_1940590978769469803[4] = -nom_x[4] + true_x[4];
   out_1940590978769469803[5] = -nom_x[5] + true_x[5];
   out_1940590978769469803[6] = -nom_x[6] + true_x[6];
   out_1940590978769469803[7] = -nom_x[7] + true_x[7];
   out_1940590978769469803[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7400348099938345675) {
   out_7400348099938345675[0] = 1.0;
   out_7400348099938345675[1] = 0.0;
   out_7400348099938345675[2] = 0.0;
   out_7400348099938345675[3] = 0.0;
   out_7400348099938345675[4] = 0.0;
   out_7400348099938345675[5] = 0.0;
   out_7400348099938345675[6] = 0.0;
   out_7400348099938345675[7] = 0.0;
   out_7400348099938345675[8] = 0.0;
   out_7400348099938345675[9] = 0.0;
   out_7400348099938345675[10] = 1.0;
   out_7400348099938345675[11] = 0.0;
   out_7400348099938345675[12] = 0.0;
   out_7400348099938345675[13] = 0.0;
   out_7400348099938345675[14] = 0.0;
   out_7400348099938345675[15] = 0.0;
   out_7400348099938345675[16] = 0.0;
   out_7400348099938345675[17] = 0.0;
   out_7400348099938345675[18] = 0.0;
   out_7400348099938345675[19] = 0.0;
   out_7400348099938345675[20] = 1.0;
   out_7400348099938345675[21] = 0.0;
   out_7400348099938345675[22] = 0.0;
   out_7400348099938345675[23] = 0.0;
   out_7400348099938345675[24] = 0.0;
   out_7400348099938345675[25] = 0.0;
   out_7400348099938345675[26] = 0.0;
   out_7400348099938345675[27] = 0.0;
   out_7400348099938345675[28] = 0.0;
   out_7400348099938345675[29] = 0.0;
   out_7400348099938345675[30] = 1.0;
   out_7400348099938345675[31] = 0.0;
   out_7400348099938345675[32] = 0.0;
   out_7400348099938345675[33] = 0.0;
   out_7400348099938345675[34] = 0.0;
   out_7400348099938345675[35] = 0.0;
   out_7400348099938345675[36] = 0.0;
   out_7400348099938345675[37] = 0.0;
   out_7400348099938345675[38] = 0.0;
   out_7400348099938345675[39] = 0.0;
   out_7400348099938345675[40] = 1.0;
   out_7400348099938345675[41] = 0.0;
   out_7400348099938345675[42] = 0.0;
   out_7400348099938345675[43] = 0.0;
   out_7400348099938345675[44] = 0.0;
   out_7400348099938345675[45] = 0.0;
   out_7400348099938345675[46] = 0.0;
   out_7400348099938345675[47] = 0.0;
   out_7400348099938345675[48] = 0.0;
   out_7400348099938345675[49] = 0.0;
   out_7400348099938345675[50] = 1.0;
   out_7400348099938345675[51] = 0.0;
   out_7400348099938345675[52] = 0.0;
   out_7400348099938345675[53] = 0.0;
   out_7400348099938345675[54] = 0.0;
   out_7400348099938345675[55] = 0.0;
   out_7400348099938345675[56] = 0.0;
   out_7400348099938345675[57] = 0.0;
   out_7400348099938345675[58] = 0.0;
   out_7400348099938345675[59] = 0.0;
   out_7400348099938345675[60] = 1.0;
   out_7400348099938345675[61] = 0.0;
   out_7400348099938345675[62] = 0.0;
   out_7400348099938345675[63] = 0.0;
   out_7400348099938345675[64] = 0.0;
   out_7400348099938345675[65] = 0.0;
   out_7400348099938345675[66] = 0.0;
   out_7400348099938345675[67] = 0.0;
   out_7400348099938345675[68] = 0.0;
   out_7400348099938345675[69] = 0.0;
   out_7400348099938345675[70] = 1.0;
   out_7400348099938345675[71] = 0.0;
   out_7400348099938345675[72] = 0.0;
   out_7400348099938345675[73] = 0.0;
   out_7400348099938345675[74] = 0.0;
   out_7400348099938345675[75] = 0.0;
   out_7400348099938345675[76] = 0.0;
   out_7400348099938345675[77] = 0.0;
   out_7400348099938345675[78] = 0.0;
   out_7400348099938345675[79] = 0.0;
   out_7400348099938345675[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4000178862490275103) {
   out_4000178862490275103[0] = state[0];
   out_4000178862490275103[1] = state[1];
   out_4000178862490275103[2] = state[2];
   out_4000178862490275103[3] = state[3];
   out_4000178862490275103[4] = state[4];
   out_4000178862490275103[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4000178862490275103[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4000178862490275103[7] = state[7];
   out_4000178862490275103[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9192624720502780241) {
   out_9192624720502780241[0] = 1;
   out_9192624720502780241[1] = 0;
   out_9192624720502780241[2] = 0;
   out_9192624720502780241[3] = 0;
   out_9192624720502780241[4] = 0;
   out_9192624720502780241[5] = 0;
   out_9192624720502780241[6] = 0;
   out_9192624720502780241[7] = 0;
   out_9192624720502780241[8] = 0;
   out_9192624720502780241[9] = 0;
   out_9192624720502780241[10] = 1;
   out_9192624720502780241[11] = 0;
   out_9192624720502780241[12] = 0;
   out_9192624720502780241[13] = 0;
   out_9192624720502780241[14] = 0;
   out_9192624720502780241[15] = 0;
   out_9192624720502780241[16] = 0;
   out_9192624720502780241[17] = 0;
   out_9192624720502780241[18] = 0;
   out_9192624720502780241[19] = 0;
   out_9192624720502780241[20] = 1;
   out_9192624720502780241[21] = 0;
   out_9192624720502780241[22] = 0;
   out_9192624720502780241[23] = 0;
   out_9192624720502780241[24] = 0;
   out_9192624720502780241[25] = 0;
   out_9192624720502780241[26] = 0;
   out_9192624720502780241[27] = 0;
   out_9192624720502780241[28] = 0;
   out_9192624720502780241[29] = 0;
   out_9192624720502780241[30] = 1;
   out_9192624720502780241[31] = 0;
   out_9192624720502780241[32] = 0;
   out_9192624720502780241[33] = 0;
   out_9192624720502780241[34] = 0;
   out_9192624720502780241[35] = 0;
   out_9192624720502780241[36] = 0;
   out_9192624720502780241[37] = 0;
   out_9192624720502780241[38] = 0;
   out_9192624720502780241[39] = 0;
   out_9192624720502780241[40] = 1;
   out_9192624720502780241[41] = 0;
   out_9192624720502780241[42] = 0;
   out_9192624720502780241[43] = 0;
   out_9192624720502780241[44] = 0;
   out_9192624720502780241[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9192624720502780241[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9192624720502780241[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9192624720502780241[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9192624720502780241[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9192624720502780241[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9192624720502780241[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9192624720502780241[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9192624720502780241[53] = -9.8100000000000005*dt;
   out_9192624720502780241[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9192624720502780241[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9192624720502780241[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9192624720502780241[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9192624720502780241[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9192624720502780241[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9192624720502780241[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9192624720502780241[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9192624720502780241[62] = 0;
   out_9192624720502780241[63] = 0;
   out_9192624720502780241[64] = 0;
   out_9192624720502780241[65] = 0;
   out_9192624720502780241[66] = 0;
   out_9192624720502780241[67] = 0;
   out_9192624720502780241[68] = 0;
   out_9192624720502780241[69] = 0;
   out_9192624720502780241[70] = 1;
   out_9192624720502780241[71] = 0;
   out_9192624720502780241[72] = 0;
   out_9192624720502780241[73] = 0;
   out_9192624720502780241[74] = 0;
   out_9192624720502780241[75] = 0;
   out_9192624720502780241[76] = 0;
   out_9192624720502780241[77] = 0;
   out_9192624720502780241[78] = 0;
   out_9192624720502780241[79] = 0;
   out_9192624720502780241[80] = 1;
}
void h_25(double *state, double *unused, double *out_7612834023020400077) {
   out_7612834023020400077[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7905682542138642387) {
   out_7905682542138642387[0] = 0;
   out_7905682542138642387[1] = 0;
   out_7905682542138642387[2] = 0;
   out_7905682542138642387[3] = 0;
   out_7905682542138642387[4] = 0;
   out_7905682542138642387[5] = 0;
   out_7905682542138642387[6] = 1;
   out_7905682542138642387[7] = 0;
   out_7905682542138642387[8] = 0;
}
void h_24(double *state, double *unused, double *out_4364324367018849338) {
   out_4364324367018849338[0] = state[4];
   out_4364324367018849338[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8750679150922337786) {
   out_8750679150922337786[0] = 0;
   out_8750679150922337786[1] = 0;
   out_8750679150922337786[2] = 0;
   out_8750679150922337786[3] = 0;
   out_8750679150922337786[4] = 1;
   out_8750679150922337786[5] = 0;
   out_8750679150922337786[6] = 0;
   out_8750679150922337786[7] = 0;
   out_8750679150922337786[8] = 0;
   out_8750679150922337786[9] = 0;
   out_8750679150922337786[10] = 0;
   out_8750679150922337786[11] = 0;
   out_8750679150922337786[12] = 0;
   out_8750679150922337786[13] = 0;
   out_8750679150922337786[14] = 1;
   out_8750679150922337786[15] = 0;
   out_8750679150922337786[16] = 0;
   out_8750679150922337786[17] = 0;
}
void h_30(double *state, double *unused, double *out_268438627733649108) {
   out_268438627733649108[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3624371190079292474) {
   out_3624371190079292474[0] = 0;
   out_3624371190079292474[1] = 0;
   out_3624371190079292474[2] = 0;
   out_3624371190079292474[3] = 0;
   out_3624371190079292474[4] = 1;
   out_3624371190079292474[5] = 0;
   out_3624371190079292474[6] = 0;
   out_3624371190079292474[7] = 0;
   out_3624371190079292474[8] = 0;
}
void h_26(double *state, double *unused, double *out_522224942736774154) {
   out_522224942736774154[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8562536606248954291) {
   out_8562536606248954291[0] = 0;
   out_8562536606248954291[1] = 0;
   out_8562536606248954291[2] = 0;
   out_8562536606248954291[3] = 0;
   out_8562536606248954291[4] = 0;
   out_8562536606248954291[5] = 0;
   out_8562536606248954291[6] = 0;
   out_8562536606248954291[7] = 1;
   out_8562536606248954291[8] = 0;
}
void h_27(double *state, double *unused, double *out_4434868533275302393) {
   out_4434868533275302393[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5799134501879717385) {
   out_5799134501879717385[0] = 0;
   out_5799134501879717385[1] = 0;
   out_5799134501879717385[2] = 0;
   out_5799134501879717385[3] = 1;
   out_5799134501879717385[4] = 0;
   out_5799134501879717385[5] = 0;
   out_5799134501879717385[6] = 0;
   out_5799134501879717385[7] = 0;
   out_5799134501879717385[8] = 0;
}
void h_29(double *state, double *unused, double *out_4061792147988437599) {
   out_4061792147988437599[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3114139845764900290) {
   out_3114139845764900290[0] = 0;
   out_3114139845764900290[1] = 1;
   out_3114139845764900290[2] = 0;
   out_3114139845764900290[3] = 0;
   out_3114139845764900290[4] = 0;
   out_3114139845764900290[5] = 0;
   out_3114139845764900290[6] = 0;
   out_3114139845764900290[7] = 0;
   out_3114139845764900290[8] = 0;
}
void h_28(double *state, double *unused, double *out_8369145669353292388) {
   out_8369145669353292388[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8196538862834430864) {
   out_8196538862834430864[0] = 1;
   out_8196538862834430864[1] = 0;
   out_8196538862834430864[2] = 0;
   out_8196538862834430864[3] = 0;
   out_8196538862834430864[4] = 0;
   out_8196538862834430864[5] = 0;
   out_8196538862834430864[6] = 0;
   out_8196538862834430864[7] = 0;
   out_8196538862834430864[8] = 0;
}
void h_31(double *state, double *unused, double *out_7196460522435129189) {
   out_7196460522435129189[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7936328504015602815) {
   out_7936328504015602815[0] = 0;
   out_7936328504015602815[1] = 0;
   out_7936328504015602815[2] = 0;
   out_7936328504015602815[3] = 0;
   out_7936328504015602815[4] = 0;
   out_7936328504015602815[5] = 0;
   out_7936328504015602815[6] = 0;
   out_7936328504015602815[7] = 0;
   out_7936328504015602815[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3838225519793418139) {
  err_fun(nom_x, delta_x, out_3838225519793418139);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1940590978769469803) {
  inv_err_fun(nom_x, true_x, out_1940590978769469803);
}
void car_H_mod_fun(double *state, double *out_7400348099938345675) {
  H_mod_fun(state, out_7400348099938345675);
}
void car_f_fun(double *state, double dt, double *out_4000178862490275103) {
  f_fun(state,  dt, out_4000178862490275103);
}
void car_F_fun(double *state, double dt, double *out_9192624720502780241) {
  F_fun(state,  dt, out_9192624720502780241);
}
void car_h_25(double *state, double *unused, double *out_7612834023020400077) {
  h_25(state, unused, out_7612834023020400077);
}
void car_H_25(double *state, double *unused, double *out_7905682542138642387) {
  H_25(state, unused, out_7905682542138642387);
}
void car_h_24(double *state, double *unused, double *out_4364324367018849338) {
  h_24(state, unused, out_4364324367018849338);
}
void car_H_24(double *state, double *unused, double *out_8750679150922337786) {
  H_24(state, unused, out_8750679150922337786);
}
void car_h_30(double *state, double *unused, double *out_268438627733649108) {
  h_30(state, unused, out_268438627733649108);
}
void car_H_30(double *state, double *unused, double *out_3624371190079292474) {
  H_30(state, unused, out_3624371190079292474);
}
void car_h_26(double *state, double *unused, double *out_522224942736774154) {
  h_26(state, unused, out_522224942736774154);
}
void car_H_26(double *state, double *unused, double *out_8562536606248954291) {
  H_26(state, unused, out_8562536606248954291);
}
void car_h_27(double *state, double *unused, double *out_4434868533275302393) {
  h_27(state, unused, out_4434868533275302393);
}
void car_H_27(double *state, double *unused, double *out_5799134501879717385) {
  H_27(state, unused, out_5799134501879717385);
}
void car_h_29(double *state, double *unused, double *out_4061792147988437599) {
  h_29(state, unused, out_4061792147988437599);
}
void car_H_29(double *state, double *unused, double *out_3114139845764900290) {
  H_29(state, unused, out_3114139845764900290);
}
void car_h_28(double *state, double *unused, double *out_8369145669353292388) {
  h_28(state, unused, out_8369145669353292388);
}
void car_H_28(double *state, double *unused, double *out_8196538862834430864) {
  H_28(state, unused, out_8196538862834430864);
}
void car_h_31(double *state, double *unused, double *out_7196460522435129189) {
  h_31(state, unused, out_7196460522435129189);
}
void car_H_31(double *state, double *unused, double *out_7936328504015602815) {
  H_31(state, unused, out_7936328504015602815);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
