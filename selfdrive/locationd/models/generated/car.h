#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3651118281197890389);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2655762840432959964);
void car_H_mod_fun(double *state, double *out_2052889645270536617);
void car_f_fun(double *state, double dt, double *out_1971980183116789588);
void car_F_fun(double *state, double dt, double *out_2299789279219106909);
void car_h_25(double *state, double *unused, double *out_5706842721188571505);
void car_H_25(double *state, double *unused, double *out_8549912605160566568);
void car_h_24(double *state, double *unused, double *out_3399406047959491884);
void car_H_24(double *state, double *unused, double *out_9020370087203905292);
void car_h_30(double *state, double *unused, double *out_3430874563495279677);
void car_H_30(double *state, double *unused, double *out_5369135138421376850);
void car_h_26(double *state, double *unused, double *out_7828826644776908689);
void car_H_26(double *state, double *unused, double *out_6155328149674928824);
void car_h_27(double *state, double *unused, double *out_4151738670716138703);
void car_H_27(double *state, double *unused, double *out_7592729209605320067);
void car_h_29(double *state, double *unused, double *out_3876544608431632814);
void car_H_29(double *state, double *unused, double *out_5879366482735769034);
void car_h_28(double *state, double *unused, double *out_7895963885579563767);
void car_H_28(double *state, double *unused, double *out_796967465666238460);
void car_h_31(double *state, double *unused, double *out_5431648658904065616);
void car_H_31(double *state, double *unused, double *out_8519266643283606140);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}