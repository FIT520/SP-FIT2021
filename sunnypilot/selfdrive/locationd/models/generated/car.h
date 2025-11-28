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
void car_err_fun(double *nom_x, double *delta_x, double *out_3838225519793418139);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1940590978769469803);
void car_H_mod_fun(double *state, double *out_7400348099938345675);
void car_f_fun(double *state, double dt, double *out_4000178862490275103);
void car_F_fun(double *state, double dt, double *out_9192624720502780241);
void car_h_25(double *state, double *unused, double *out_7612834023020400077);
void car_H_25(double *state, double *unused, double *out_7905682542138642387);
void car_h_24(double *state, double *unused, double *out_4364324367018849338);
void car_H_24(double *state, double *unused, double *out_8750679150922337786);
void car_h_30(double *state, double *unused, double *out_268438627733649108);
void car_H_30(double *state, double *unused, double *out_3624371190079292474);
void car_h_26(double *state, double *unused, double *out_522224942736774154);
void car_H_26(double *state, double *unused, double *out_8562536606248954291);
void car_h_27(double *state, double *unused, double *out_4434868533275302393);
void car_H_27(double *state, double *unused, double *out_5799134501879717385);
void car_h_29(double *state, double *unused, double *out_4061792147988437599);
void car_H_29(double *state, double *unused, double *out_3114139845764900290);
void car_h_28(double *state, double *unused, double *out_8369145669353292388);
void car_H_28(double *state, double *unused, double *out_8196538862834430864);
void car_h_31(double *state, double *unused, double *out_7196460522435129189);
void car_H_31(double *state, double *unused, double *out_7936328504015602815);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}