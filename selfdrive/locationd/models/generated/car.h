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
void car_err_fun(double *nom_x, double *delta_x, double *out_2738061425187338454);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2343897378972888895);
void car_H_mod_fun(double *state, double *out_2115378010922324888);
void car_f_fun(double *state, double dt, double *out_20827168147671157);
void car_F_fun(double *state, double dt, double *out_5766643830414072366);
void car_h_25(double *state, double *unused, double *out_4459931352153924969);
void car_H_25(double *state, double *unused, double *out_8410028029813366923);
void car_h_24(double *state, double *unused, double *out_8671769548308578181);
void car_H_24(double *state, double *unused, double *out_7864066444890685127);
void car_h_30(double *state, double *unused, double *out_7215977495205275711);
void car_H_30(double *state, double *unused, double *out_5509019713768576495);
void car_h_26(double *state, double *unused, double *out_8159487348106938146);
void car_H_26(double *state, double *unused, double *out_6295212725022128469);
void car_h_27(double *state, double *unused, double *out_541741915506920676);
void car_H_27(double *state, double *unused, double *out_3334256401968151584);
void car_h_29(double *state, double *unused, double *out_4179191443760943026);
void car_H_29(double *state, double *unused, double *out_6019251058082968679);
void car_h_28(double *state, double *unused, double *out_1148705919228760284);
void car_H_28(double *state, double *unused, double *out_7982881329648294930);
void car_h_31(double *state, double *unused, double *out_7407657342509409930);
void car_H_31(double *state, double *unused, double *out_5669004622788776993);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}