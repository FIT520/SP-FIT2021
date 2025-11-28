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
void err_fun(double *nom_x, double *delta_x, double *out_5738968429187976759) {
   out_5738968429187976759[0] = delta_x[0] + nom_x[0];
   out_5738968429187976759[1] = delta_x[1] + nom_x[1];
   out_5738968429187976759[2] = delta_x[2] + nom_x[2];
   out_5738968429187976759[3] = delta_x[3] + nom_x[3];
   out_5738968429187976759[4] = delta_x[4] + nom_x[4];
   out_5738968429187976759[5] = delta_x[5] + nom_x[5];
   out_5738968429187976759[6] = delta_x[6] + nom_x[6];
   out_5738968429187976759[7] = delta_x[7] + nom_x[7];
   out_5738968429187976759[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8045877841605503682) {
   out_8045877841605503682[0] = -nom_x[0] + true_x[0];
   out_8045877841605503682[1] = -nom_x[1] + true_x[1];
   out_8045877841605503682[2] = -nom_x[2] + true_x[2];
   out_8045877841605503682[3] = -nom_x[3] + true_x[3];
   out_8045877841605503682[4] = -nom_x[4] + true_x[4];
   out_8045877841605503682[5] = -nom_x[5] + true_x[5];
   out_8045877841605503682[6] = -nom_x[6] + true_x[6];
   out_8045877841605503682[7] = -nom_x[7] + true_x[7];
   out_8045877841605503682[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_259521190058915161) {
   out_259521190058915161[0] = 1.0;
   out_259521190058915161[1] = 0.0;
   out_259521190058915161[2] = 0.0;
   out_259521190058915161[3] = 0.0;
   out_259521190058915161[4] = 0.0;
   out_259521190058915161[5] = 0.0;
   out_259521190058915161[6] = 0.0;
   out_259521190058915161[7] = 0.0;
   out_259521190058915161[8] = 0.0;
   out_259521190058915161[9] = 0.0;
   out_259521190058915161[10] = 1.0;
   out_259521190058915161[11] = 0.0;
   out_259521190058915161[12] = 0.0;
   out_259521190058915161[13] = 0.0;
   out_259521190058915161[14] = 0.0;
   out_259521190058915161[15] = 0.0;
   out_259521190058915161[16] = 0.0;
   out_259521190058915161[17] = 0.0;
   out_259521190058915161[18] = 0.0;
   out_259521190058915161[19] = 0.0;
   out_259521190058915161[20] = 1.0;
   out_259521190058915161[21] = 0.0;
   out_259521190058915161[22] = 0.0;
   out_259521190058915161[23] = 0.0;
   out_259521190058915161[24] = 0.0;
   out_259521190058915161[25] = 0.0;
   out_259521190058915161[26] = 0.0;
   out_259521190058915161[27] = 0.0;
   out_259521190058915161[28] = 0.0;
   out_259521190058915161[29] = 0.0;
   out_259521190058915161[30] = 1.0;
   out_259521190058915161[31] = 0.0;
   out_259521190058915161[32] = 0.0;
   out_259521190058915161[33] = 0.0;
   out_259521190058915161[34] = 0.0;
   out_259521190058915161[35] = 0.0;
   out_259521190058915161[36] = 0.0;
   out_259521190058915161[37] = 0.0;
   out_259521190058915161[38] = 0.0;
   out_259521190058915161[39] = 0.0;
   out_259521190058915161[40] = 1.0;
   out_259521190058915161[41] = 0.0;
   out_259521190058915161[42] = 0.0;
   out_259521190058915161[43] = 0.0;
   out_259521190058915161[44] = 0.0;
   out_259521190058915161[45] = 0.0;
   out_259521190058915161[46] = 0.0;
   out_259521190058915161[47] = 0.0;
   out_259521190058915161[48] = 0.0;
   out_259521190058915161[49] = 0.0;
   out_259521190058915161[50] = 1.0;
   out_259521190058915161[51] = 0.0;
   out_259521190058915161[52] = 0.0;
   out_259521190058915161[53] = 0.0;
   out_259521190058915161[54] = 0.0;
   out_259521190058915161[55] = 0.0;
   out_259521190058915161[56] = 0.0;
   out_259521190058915161[57] = 0.0;
   out_259521190058915161[58] = 0.0;
   out_259521190058915161[59] = 0.0;
   out_259521190058915161[60] = 1.0;
   out_259521190058915161[61] = 0.0;
   out_259521190058915161[62] = 0.0;
   out_259521190058915161[63] = 0.0;
   out_259521190058915161[64] = 0.0;
   out_259521190058915161[65] = 0.0;
   out_259521190058915161[66] = 0.0;
   out_259521190058915161[67] = 0.0;
   out_259521190058915161[68] = 0.0;
   out_259521190058915161[69] = 0.0;
   out_259521190058915161[70] = 1.0;
   out_259521190058915161[71] = 0.0;
   out_259521190058915161[72] = 0.0;
   out_259521190058915161[73] = 0.0;
   out_259521190058915161[74] = 0.0;
   out_259521190058915161[75] = 0.0;
   out_259521190058915161[76] = 0.0;
   out_259521190058915161[77] = 0.0;
   out_259521190058915161[78] = 0.0;
   out_259521190058915161[79] = 0.0;
   out_259521190058915161[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6851437618939961373) {
   out_6851437618939961373[0] = state[0];
   out_6851437618939961373[1] = state[1];
   out_6851437618939961373[2] = state[2];
   out_6851437618939961373[3] = state[3];
   out_6851437618939961373[4] = state[4];
   out_6851437618939961373[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6851437618939961373[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6851437618939961373[7] = state[7];
   out_6851437618939961373[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7680386554523918165) {
   out_7680386554523918165[0] = 1;
   out_7680386554523918165[1] = 0;
   out_7680386554523918165[2] = 0;
   out_7680386554523918165[3] = 0;
   out_7680386554523918165[4] = 0;
   out_7680386554523918165[5] = 0;
   out_7680386554523918165[6] = 0;
   out_7680386554523918165[7] = 0;
   out_7680386554523918165[8] = 0;
   out_7680386554523918165[9] = 0;
   out_7680386554523918165[10] = 1;
   out_7680386554523918165[11] = 0;
   out_7680386554523918165[12] = 0;
   out_7680386554523918165[13] = 0;
   out_7680386554523918165[14] = 0;
   out_7680386554523918165[15] = 0;
   out_7680386554523918165[16] = 0;
   out_7680386554523918165[17] = 0;
   out_7680386554523918165[18] = 0;
   out_7680386554523918165[19] = 0;
   out_7680386554523918165[20] = 1;
   out_7680386554523918165[21] = 0;
   out_7680386554523918165[22] = 0;
   out_7680386554523918165[23] = 0;
   out_7680386554523918165[24] = 0;
   out_7680386554523918165[25] = 0;
   out_7680386554523918165[26] = 0;
   out_7680386554523918165[27] = 0;
   out_7680386554523918165[28] = 0;
   out_7680386554523918165[29] = 0;
   out_7680386554523918165[30] = 1;
   out_7680386554523918165[31] = 0;
   out_7680386554523918165[32] = 0;
   out_7680386554523918165[33] = 0;
   out_7680386554523918165[34] = 0;
   out_7680386554523918165[35] = 0;
   out_7680386554523918165[36] = 0;
   out_7680386554523918165[37] = 0;
   out_7680386554523918165[38] = 0;
   out_7680386554523918165[39] = 0;
   out_7680386554523918165[40] = 1;
   out_7680386554523918165[41] = 0;
   out_7680386554523918165[42] = 0;
   out_7680386554523918165[43] = 0;
   out_7680386554523918165[44] = 0;
   out_7680386554523918165[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7680386554523918165[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7680386554523918165[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7680386554523918165[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7680386554523918165[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7680386554523918165[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7680386554523918165[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7680386554523918165[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7680386554523918165[53] = -9.8100000000000005*dt;
   out_7680386554523918165[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7680386554523918165[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7680386554523918165[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7680386554523918165[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7680386554523918165[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7680386554523918165[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7680386554523918165[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7680386554523918165[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7680386554523918165[62] = 0;
   out_7680386554523918165[63] = 0;
   out_7680386554523918165[64] = 0;
   out_7680386554523918165[65] = 0;
   out_7680386554523918165[66] = 0;
   out_7680386554523918165[67] = 0;
   out_7680386554523918165[68] = 0;
   out_7680386554523918165[69] = 0;
   out_7680386554523918165[70] = 1;
   out_7680386554523918165[71] = 0;
   out_7680386554523918165[72] = 0;
   out_7680386554523918165[73] = 0;
   out_7680386554523918165[74] = 0;
   out_7680386554523918165[75] = 0;
   out_7680386554523918165[76] = 0;
   out_7680386554523918165[77] = 0;
   out_7680386554523918165[78] = 0;
   out_7680386554523918165[79] = 0;
   out_7680386554523918165[80] = 1;
}
void h_25(double *state, double *unused, double *out_3477296093554491887) {
   out_3477296093554491887[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8994034657738399218) {
   out_8994034657738399218[0] = 0;
   out_8994034657738399218[1] = 0;
   out_8994034657738399218[2] = 0;
   out_8994034657738399218[3] = 0;
   out_8994034657738399218[4] = 0;
   out_8994034657738399218[5] = 0;
   out_8994034657738399218[6] = 1;
   out_8994034657738399218[7] = 0;
   out_8994034657738399218[8] = 0;
}
void h_24(double *state, double *unused, double *out_3675004507071379308) {
   out_3675004507071379308[0] = state[4];
   out_3675004507071379308[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4247525330771890184) {
   out_4247525330771890184[0] = 0;
   out_4247525330771890184[1] = 0;
   out_4247525330771890184[2] = 0;
   out_4247525330771890184[3] = 0;
   out_4247525330771890184[4] = 1;
   out_4247525330771890184[5] = 0;
   out_4247525330771890184[6] = 0;
   out_4247525330771890184[7] = 0;
   out_4247525330771890184[8] = 0;
   out_4247525330771890184[9] = 0;
   out_4247525330771890184[10] = 0;
   out_4247525330771890184[11] = 0;
   out_4247525330771890184[12] = 0;
   out_4247525330771890184[13] = 0;
   out_4247525330771890184[14] = 1;
   out_4247525330771890184[15] = 0;
   out_4247525330771890184[16] = 0;
   out_4247525330771890184[17] = 0;
}
void h_30(double *state, double *unused, double *out_4879260551704948855) {
   out_4879260551704948855[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2536019074479535643) {
   out_2536019074479535643[0] = 0;
   out_2536019074479535643[1] = 0;
   out_2536019074479535643[2] = 0;
   out_2536019074479535643[3] = 0;
   out_2536019074479535643[4] = 1;
   out_2536019074479535643[5] = 0;
   out_2536019074479535643[6] = 0;
   out_2536019074479535643[7] = 0;
   out_2536019074479535643[8] = 0;
}
void h_26(double *state, double *unused, double *out_8565810254343757116) {
   out_8565810254343757116[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5252531338864342994) {
   out_5252531338864342994[0] = 0;
   out_5252531338864342994[1] = 0;
   out_5252531338864342994[2] = 0;
   out_5252531338864342994[3] = 0;
   out_5252531338864342994[4] = 0;
   out_5252531338864342994[5] = 0;
   out_5252531338864342994[6] = 0;
   out_5252531338864342994[7] = 1;
   out_5252531338864342994[8] = 0;
}
void h_27(double *state, double *unused, double *out_4742701018348891141) {
   out_4742701018348891141[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4710782386279960554) {
   out_4710782386279960554[0] = 0;
   out_4710782386279960554[1] = 0;
   out_4710782386279960554[2] = 0;
   out_4710782386279960554[3] = 1;
   out_4710782386279960554[4] = 0;
   out_4710782386279960554[5] = 0;
   out_4710782386279960554[6] = 0;
   out_4710782386279960554[7] = 0;
   out_4710782386279960554[8] = 0;
}
void h_29(double *state, double *unused, double *out_5822770404606611290) {
   out_5822770404606611290[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6424145113149511587) {
   out_6424145113149511587[0] = 0;
   out_6424145113149511587[1] = 1;
   out_6424145113149511587[2] = 0;
   out_6424145113149511587[3] = 0;
   out_6424145113149511587[4] = 0;
   out_6424145113149511587[5] = 0;
   out_6424145113149511587[6] = 0;
   out_6424145113149511587[7] = 0;
   out_6424145113149511587[8] = 0;
}
void h_28(double *state, double *unused, double *out_8702369787699814830) {
   out_8702369787699814830[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6940199943490509455) {
   out_6940199943490509455[0] = 1;
   out_6940199943490509455[1] = 0;
   out_6940199943490509455[2] = 0;
   out_6940199943490509455[3] = 0;
   out_6940199943490509455[4] = 0;
   out_6940199943490509455[5] = 0;
   out_6940199943490509455[6] = 0;
   out_6940199943490509455[7] = 0;
   out_6940199943490509455[8] = 0;
}
void h_31(double *state, double *unused, double *out_3485422862729087140) {
   out_3485422862729087140[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9024680619615359646) {
   out_9024680619615359646[0] = 0;
   out_9024680619615359646[1] = 0;
   out_9024680619615359646[2] = 0;
   out_9024680619615359646[3] = 0;
   out_9024680619615359646[4] = 0;
   out_9024680619615359646[5] = 0;
   out_9024680619615359646[6] = 0;
   out_9024680619615359646[7] = 0;
   out_9024680619615359646[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5738968429187976759) {
  err_fun(nom_x, delta_x, out_5738968429187976759);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8045877841605503682) {
  inv_err_fun(nom_x, true_x, out_8045877841605503682);
}
void car_H_mod_fun(double *state, double *out_259521190058915161) {
  H_mod_fun(state, out_259521190058915161);
}
void car_f_fun(double *state, double dt, double *out_6851437618939961373) {
  f_fun(state,  dt, out_6851437618939961373);
}
void car_F_fun(double *state, double dt, double *out_7680386554523918165) {
  F_fun(state,  dt, out_7680386554523918165);
}
void car_h_25(double *state, double *unused, double *out_3477296093554491887) {
  h_25(state, unused, out_3477296093554491887);
}
void car_H_25(double *state, double *unused, double *out_8994034657738399218) {
  H_25(state, unused, out_8994034657738399218);
}
void car_h_24(double *state, double *unused, double *out_3675004507071379308) {
  h_24(state, unused, out_3675004507071379308);
}
void car_H_24(double *state, double *unused, double *out_4247525330771890184) {
  H_24(state, unused, out_4247525330771890184);
}
void car_h_30(double *state, double *unused, double *out_4879260551704948855) {
  h_30(state, unused, out_4879260551704948855);
}
void car_H_30(double *state, double *unused, double *out_2536019074479535643) {
  H_30(state, unused, out_2536019074479535643);
}
void car_h_26(double *state, double *unused, double *out_8565810254343757116) {
  h_26(state, unused, out_8565810254343757116);
}
void car_H_26(double *state, double *unused, double *out_5252531338864342994) {
  H_26(state, unused, out_5252531338864342994);
}
void car_h_27(double *state, double *unused, double *out_4742701018348891141) {
  h_27(state, unused, out_4742701018348891141);
}
void car_H_27(double *state, double *unused, double *out_4710782386279960554) {
  H_27(state, unused, out_4710782386279960554);
}
void car_h_29(double *state, double *unused, double *out_5822770404606611290) {
  h_29(state, unused, out_5822770404606611290);
}
void car_H_29(double *state, double *unused, double *out_6424145113149511587) {
  H_29(state, unused, out_6424145113149511587);
}
void car_h_28(double *state, double *unused, double *out_8702369787699814830) {
  h_28(state, unused, out_8702369787699814830);
}
void car_H_28(double *state, double *unused, double *out_6940199943490509455) {
  H_28(state, unused, out_6940199943490509455);
}
void car_h_31(double *state, double *unused, double *out_3485422862729087140) {
  h_31(state, unused, out_3485422862729087140);
}
void car_H_31(double *state, double *unused, double *out_9024680619615359646) {
  H_31(state, unused, out_9024680619615359646);
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
