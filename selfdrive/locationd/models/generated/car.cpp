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
void err_fun(double *nom_x, double *delta_x, double *out_2738061425187338454) {
   out_2738061425187338454[0] = delta_x[0] + nom_x[0];
   out_2738061425187338454[1] = delta_x[1] + nom_x[1];
   out_2738061425187338454[2] = delta_x[2] + nom_x[2];
   out_2738061425187338454[3] = delta_x[3] + nom_x[3];
   out_2738061425187338454[4] = delta_x[4] + nom_x[4];
   out_2738061425187338454[5] = delta_x[5] + nom_x[5];
   out_2738061425187338454[6] = delta_x[6] + nom_x[6];
   out_2738061425187338454[7] = delta_x[7] + nom_x[7];
   out_2738061425187338454[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2343897378972888895) {
   out_2343897378972888895[0] = -nom_x[0] + true_x[0];
   out_2343897378972888895[1] = -nom_x[1] + true_x[1];
   out_2343897378972888895[2] = -nom_x[2] + true_x[2];
   out_2343897378972888895[3] = -nom_x[3] + true_x[3];
   out_2343897378972888895[4] = -nom_x[4] + true_x[4];
   out_2343897378972888895[5] = -nom_x[5] + true_x[5];
   out_2343897378972888895[6] = -nom_x[6] + true_x[6];
   out_2343897378972888895[7] = -nom_x[7] + true_x[7];
   out_2343897378972888895[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2115378010922324888) {
   out_2115378010922324888[0] = 1.0;
   out_2115378010922324888[1] = 0.0;
   out_2115378010922324888[2] = 0.0;
   out_2115378010922324888[3] = 0.0;
   out_2115378010922324888[4] = 0.0;
   out_2115378010922324888[5] = 0.0;
   out_2115378010922324888[6] = 0.0;
   out_2115378010922324888[7] = 0.0;
   out_2115378010922324888[8] = 0.0;
   out_2115378010922324888[9] = 0.0;
   out_2115378010922324888[10] = 1.0;
   out_2115378010922324888[11] = 0.0;
   out_2115378010922324888[12] = 0.0;
   out_2115378010922324888[13] = 0.0;
   out_2115378010922324888[14] = 0.0;
   out_2115378010922324888[15] = 0.0;
   out_2115378010922324888[16] = 0.0;
   out_2115378010922324888[17] = 0.0;
   out_2115378010922324888[18] = 0.0;
   out_2115378010922324888[19] = 0.0;
   out_2115378010922324888[20] = 1.0;
   out_2115378010922324888[21] = 0.0;
   out_2115378010922324888[22] = 0.0;
   out_2115378010922324888[23] = 0.0;
   out_2115378010922324888[24] = 0.0;
   out_2115378010922324888[25] = 0.0;
   out_2115378010922324888[26] = 0.0;
   out_2115378010922324888[27] = 0.0;
   out_2115378010922324888[28] = 0.0;
   out_2115378010922324888[29] = 0.0;
   out_2115378010922324888[30] = 1.0;
   out_2115378010922324888[31] = 0.0;
   out_2115378010922324888[32] = 0.0;
   out_2115378010922324888[33] = 0.0;
   out_2115378010922324888[34] = 0.0;
   out_2115378010922324888[35] = 0.0;
   out_2115378010922324888[36] = 0.0;
   out_2115378010922324888[37] = 0.0;
   out_2115378010922324888[38] = 0.0;
   out_2115378010922324888[39] = 0.0;
   out_2115378010922324888[40] = 1.0;
   out_2115378010922324888[41] = 0.0;
   out_2115378010922324888[42] = 0.0;
   out_2115378010922324888[43] = 0.0;
   out_2115378010922324888[44] = 0.0;
   out_2115378010922324888[45] = 0.0;
   out_2115378010922324888[46] = 0.0;
   out_2115378010922324888[47] = 0.0;
   out_2115378010922324888[48] = 0.0;
   out_2115378010922324888[49] = 0.0;
   out_2115378010922324888[50] = 1.0;
   out_2115378010922324888[51] = 0.0;
   out_2115378010922324888[52] = 0.0;
   out_2115378010922324888[53] = 0.0;
   out_2115378010922324888[54] = 0.0;
   out_2115378010922324888[55] = 0.0;
   out_2115378010922324888[56] = 0.0;
   out_2115378010922324888[57] = 0.0;
   out_2115378010922324888[58] = 0.0;
   out_2115378010922324888[59] = 0.0;
   out_2115378010922324888[60] = 1.0;
   out_2115378010922324888[61] = 0.0;
   out_2115378010922324888[62] = 0.0;
   out_2115378010922324888[63] = 0.0;
   out_2115378010922324888[64] = 0.0;
   out_2115378010922324888[65] = 0.0;
   out_2115378010922324888[66] = 0.0;
   out_2115378010922324888[67] = 0.0;
   out_2115378010922324888[68] = 0.0;
   out_2115378010922324888[69] = 0.0;
   out_2115378010922324888[70] = 1.0;
   out_2115378010922324888[71] = 0.0;
   out_2115378010922324888[72] = 0.0;
   out_2115378010922324888[73] = 0.0;
   out_2115378010922324888[74] = 0.0;
   out_2115378010922324888[75] = 0.0;
   out_2115378010922324888[76] = 0.0;
   out_2115378010922324888[77] = 0.0;
   out_2115378010922324888[78] = 0.0;
   out_2115378010922324888[79] = 0.0;
   out_2115378010922324888[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_20827168147671157) {
   out_20827168147671157[0] = state[0];
   out_20827168147671157[1] = state[1];
   out_20827168147671157[2] = state[2];
   out_20827168147671157[3] = state[3];
   out_20827168147671157[4] = state[4];
   out_20827168147671157[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_20827168147671157[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_20827168147671157[7] = state[7];
   out_20827168147671157[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5766643830414072366) {
   out_5766643830414072366[0] = 1;
   out_5766643830414072366[1] = 0;
   out_5766643830414072366[2] = 0;
   out_5766643830414072366[3] = 0;
   out_5766643830414072366[4] = 0;
   out_5766643830414072366[5] = 0;
   out_5766643830414072366[6] = 0;
   out_5766643830414072366[7] = 0;
   out_5766643830414072366[8] = 0;
   out_5766643830414072366[9] = 0;
   out_5766643830414072366[10] = 1;
   out_5766643830414072366[11] = 0;
   out_5766643830414072366[12] = 0;
   out_5766643830414072366[13] = 0;
   out_5766643830414072366[14] = 0;
   out_5766643830414072366[15] = 0;
   out_5766643830414072366[16] = 0;
   out_5766643830414072366[17] = 0;
   out_5766643830414072366[18] = 0;
   out_5766643830414072366[19] = 0;
   out_5766643830414072366[20] = 1;
   out_5766643830414072366[21] = 0;
   out_5766643830414072366[22] = 0;
   out_5766643830414072366[23] = 0;
   out_5766643830414072366[24] = 0;
   out_5766643830414072366[25] = 0;
   out_5766643830414072366[26] = 0;
   out_5766643830414072366[27] = 0;
   out_5766643830414072366[28] = 0;
   out_5766643830414072366[29] = 0;
   out_5766643830414072366[30] = 1;
   out_5766643830414072366[31] = 0;
   out_5766643830414072366[32] = 0;
   out_5766643830414072366[33] = 0;
   out_5766643830414072366[34] = 0;
   out_5766643830414072366[35] = 0;
   out_5766643830414072366[36] = 0;
   out_5766643830414072366[37] = 0;
   out_5766643830414072366[38] = 0;
   out_5766643830414072366[39] = 0;
   out_5766643830414072366[40] = 1;
   out_5766643830414072366[41] = 0;
   out_5766643830414072366[42] = 0;
   out_5766643830414072366[43] = 0;
   out_5766643830414072366[44] = 0;
   out_5766643830414072366[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5766643830414072366[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5766643830414072366[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5766643830414072366[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5766643830414072366[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5766643830414072366[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5766643830414072366[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5766643830414072366[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5766643830414072366[53] = -9.8100000000000005*dt;
   out_5766643830414072366[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5766643830414072366[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5766643830414072366[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5766643830414072366[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5766643830414072366[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5766643830414072366[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5766643830414072366[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5766643830414072366[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5766643830414072366[62] = 0;
   out_5766643830414072366[63] = 0;
   out_5766643830414072366[64] = 0;
   out_5766643830414072366[65] = 0;
   out_5766643830414072366[66] = 0;
   out_5766643830414072366[67] = 0;
   out_5766643830414072366[68] = 0;
   out_5766643830414072366[69] = 0;
   out_5766643830414072366[70] = 1;
   out_5766643830414072366[71] = 0;
   out_5766643830414072366[72] = 0;
   out_5766643830414072366[73] = 0;
   out_5766643830414072366[74] = 0;
   out_5766643830414072366[75] = 0;
   out_5766643830414072366[76] = 0;
   out_5766643830414072366[77] = 0;
   out_5766643830414072366[78] = 0;
   out_5766643830414072366[79] = 0;
   out_5766643830414072366[80] = 1;
}
void h_25(double *state, double *unused, double *out_4459931352153924969) {
   out_4459931352153924969[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8410028029813366923) {
   out_8410028029813366923[0] = 0;
   out_8410028029813366923[1] = 0;
   out_8410028029813366923[2] = 0;
   out_8410028029813366923[3] = 0;
   out_8410028029813366923[4] = 0;
   out_8410028029813366923[5] = 0;
   out_8410028029813366923[6] = 1;
   out_8410028029813366923[7] = 0;
   out_8410028029813366923[8] = 0;
}
void h_24(double *state, double *unused, double *out_8671769548308578181) {
   out_8671769548308578181[0] = state[4];
   out_8671769548308578181[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7864066444890685127) {
   out_7864066444890685127[0] = 0;
   out_7864066444890685127[1] = 0;
   out_7864066444890685127[2] = 0;
   out_7864066444890685127[3] = 0;
   out_7864066444890685127[4] = 1;
   out_7864066444890685127[5] = 0;
   out_7864066444890685127[6] = 0;
   out_7864066444890685127[7] = 0;
   out_7864066444890685127[8] = 0;
   out_7864066444890685127[9] = 0;
   out_7864066444890685127[10] = 0;
   out_7864066444890685127[11] = 0;
   out_7864066444890685127[12] = 0;
   out_7864066444890685127[13] = 0;
   out_7864066444890685127[14] = 1;
   out_7864066444890685127[15] = 0;
   out_7864066444890685127[16] = 0;
   out_7864066444890685127[17] = 0;
}
void h_30(double *state, double *unused, double *out_7215977495205275711) {
   out_7215977495205275711[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5509019713768576495) {
   out_5509019713768576495[0] = 0;
   out_5509019713768576495[1] = 0;
   out_5509019713768576495[2] = 0;
   out_5509019713768576495[3] = 0;
   out_5509019713768576495[4] = 1;
   out_5509019713768576495[5] = 0;
   out_5509019713768576495[6] = 0;
   out_5509019713768576495[7] = 0;
   out_5509019713768576495[8] = 0;
}
void h_26(double *state, double *unused, double *out_8159487348106938146) {
   out_8159487348106938146[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6295212725022128469) {
   out_6295212725022128469[0] = 0;
   out_6295212725022128469[1] = 0;
   out_6295212725022128469[2] = 0;
   out_6295212725022128469[3] = 0;
   out_6295212725022128469[4] = 0;
   out_6295212725022128469[5] = 0;
   out_6295212725022128469[6] = 0;
   out_6295212725022128469[7] = 1;
   out_6295212725022128469[8] = 0;
}
void h_27(double *state, double *unused, double *out_541741915506920676) {
   out_541741915506920676[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3334256401968151584) {
   out_3334256401968151584[0] = 0;
   out_3334256401968151584[1] = 0;
   out_3334256401968151584[2] = 0;
   out_3334256401968151584[3] = 1;
   out_3334256401968151584[4] = 0;
   out_3334256401968151584[5] = 0;
   out_3334256401968151584[6] = 0;
   out_3334256401968151584[7] = 0;
   out_3334256401968151584[8] = 0;
}
void h_29(double *state, double *unused, double *out_4179191443760943026) {
   out_4179191443760943026[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6019251058082968679) {
   out_6019251058082968679[0] = 0;
   out_6019251058082968679[1] = 1;
   out_6019251058082968679[2] = 0;
   out_6019251058082968679[3] = 0;
   out_6019251058082968679[4] = 0;
   out_6019251058082968679[5] = 0;
   out_6019251058082968679[6] = 0;
   out_6019251058082968679[7] = 0;
   out_6019251058082968679[8] = 0;
}
void h_28(double *state, double *unused, double *out_1148705919228760284) {
   out_1148705919228760284[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7982881329648294930) {
   out_7982881329648294930[0] = 1;
   out_7982881329648294930[1] = 0;
   out_7982881329648294930[2] = 0;
   out_7982881329648294930[3] = 0;
   out_7982881329648294930[4] = 0;
   out_7982881329648294930[5] = 0;
   out_7982881329648294930[6] = 0;
   out_7982881329648294930[7] = 0;
   out_7982881329648294930[8] = 0;
}
void h_31(double *state, double *unused, double *out_7407657342509409930) {
   out_7407657342509409930[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5669004622788776993) {
   out_5669004622788776993[0] = 0;
   out_5669004622788776993[1] = 0;
   out_5669004622788776993[2] = 0;
   out_5669004622788776993[3] = 0;
   out_5669004622788776993[4] = 0;
   out_5669004622788776993[5] = 0;
   out_5669004622788776993[6] = 0;
   out_5669004622788776993[7] = 0;
   out_5669004622788776993[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2738061425187338454) {
  err_fun(nom_x, delta_x, out_2738061425187338454);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2343897378972888895) {
  inv_err_fun(nom_x, true_x, out_2343897378972888895);
}
void car_H_mod_fun(double *state, double *out_2115378010922324888) {
  H_mod_fun(state, out_2115378010922324888);
}
void car_f_fun(double *state, double dt, double *out_20827168147671157) {
  f_fun(state,  dt, out_20827168147671157);
}
void car_F_fun(double *state, double dt, double *out_5766643830414072366) {
  F_fun(state,  dt, out_5766643830414072366);
}
void car_h_25(double *state, double *unused, double *out_4459931352153924969) {
  h_25(state, unused, out_4459931352153924969);
}
void car_H_25(double *state, double *unused, double *out_8410028029813366923) {
  H_25(state, unused, out_8410028029813366923);
}
void car_h_24(double *state, double *unused, double *out_8671769548308578181) {
  h_24(state, unused, out_8671769548308578181);
}
void car_H_24(double *state, double *unused, double *out_7864066444890685127) {
  H_24(state, unused, out_7864066444890685127);
}
void car_h_30(double *state, double *unused, double *out_7215977495205275711) {
  h_30(state, unused, out_7215977495205275711);
}
void car_H_30(double *state, double *unused, double *out_5509019713768576495) {
  H_30(state, unused, out_5509019713768576495);
}
void car_h_26(double *state, double *unused, double *out_8159487348106938146) {
  h_26(state, unused, out_8159487348106938146);
}
void car_H_26(double *state, double *unused, double *out_6295212725022128469) {
  H_26(state, unused, out_6295212725022128469);
}
void car_h_27(double *state, double *unused, double *out_541741915506920676) {
  h_27(state, unused, out_541741915506920676);
}
void car_H_27(double *state, double *unused, double *out_3334256401968151584) {
  H_27(state, unused, out_3334256401968151584);
}
void car_h_29(double *state, double *unused, double *out_4179191443760943026) {
  h_29(state, unused, out_4179191443760943026);
}
void car_H_29(double *state, double *unused, double *out_6019251058082968679) {
  H_29(state, unused, out_6019251058082968679);
}
void car_h_28(double *state, double *unused, double *out_1148705919228760284) {
  h_28(state, unused, out_1148705919228760284);
}
void car_H_28(double *state, double *unused, double *out_7982881329648294930) {
  H_28(state, unused, out_7982881329648294930);
}
void car_h_31(double *state, double *unused, double *out_7407657342509409930) {
  h_31(state, unused, out_7407657342509409930);
}
void car_H_31(double *state, double *unused, double *out_5669004622788776993) {
  H_31(state, unused, out_5669004622788776993);
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
