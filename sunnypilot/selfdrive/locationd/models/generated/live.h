#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1349625663821174402);
void live_err_fun(double *nom_x, double *delta_x, double *out_3313186634876651191);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5192838601747609643);
void live_H_mod_fun(double *state, double *out_3925736477630917702);
void live_f_fun(double *state, double dt, double *out_2283993206439198517);
void live_F_fun(double *state, double dt, double *out_2369571115315746141);
void live_h_4(double *state, double *unused, double *out_974292820450669372);
void live_H_4(double *state, double *unused, double *out_1239842579654216902);
void live_h_9(double *state, double *unused, double *out_5717672190248203947);
void live_H_9(double *state, double *unused, double *out_8527061514918664372);
void live_h_10(double *state, double *unused, double *out_27568656803775549);
void live_H_10(double *state, double *unused, double *out_463597886265871782);
void live_h_12(double *state, double *unused, double *out_8130703279585544366);
void live_H_12(double *state, double *unused, double *out_5141415797388516094);
void live_h_35(double *state, double *unused, double *out_3134749709432514371);
void live_H_35(double *state, double *unused, double *out_9004862020011192406);
void live_h_32(double *state, double *unused, double *out_4666491953647505827);
void live_H_32(double *state, double *unused, double *out_2532702750118829507);
void live_h_13(double *state, double *unused, double *out_5473035437882135819);
void live_H_13(double *state, double *unused, double *out_6607677163183617726);
void live_h_14(double *state, double *unused, double *out_5717672190248203947);
void live_H_14(double *state, double *unused, double *out_8527061514918664372);
void live_h_33(double *state, double *unused, double *out_7157895842031152209);
void live_H_33(double *state, double *unused, double *out_6291325049059501606);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}