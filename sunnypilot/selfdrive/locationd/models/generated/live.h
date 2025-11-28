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
void live_H(double *in_vec, double *out_7295563787497827470);
void live_err_fun(double *nom_x, double *delta_x, double *out_7079022252066649527);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6688878625595643349);
void live_H_mod_fun(double *state, double *out_5184111636797709605);
void live_f_fun(double *state, double dt, double *out_7421394147181675486);
void live_F_fun(double *state, double dt, double *out_6131813269527183934);
void live_h_4(double *state, double *unused, double *out_2105525024820073881);
void live_H_4(double *state, double *unused, double *out_8058696174864346102);
void live_h_9(double *state, double *unused, double *out_9174977010257973582);
void live_H_9(double *state, double *unused, double *out_3100828963580758044);
void live_h_10(double *state, double *unused, double *out_3752926828504477897);
void live_H_10(double *state, double *unused, double *out_4207661747253144288);
void live_h_12(double *state, double *unused, double *out_2145991482537534832);
void live_H_12(double *state, double *unused, double *out_5368591490813243719);
void live_h_35(double *state, double *unused, double *out_4949107422818642067);
void live_H_35(double *state, double *unused, double *out_2623028458488230010);
void live_h_32(double *state, double *unused, double *out_9141766520989641055);
void live_H_32(double *state, double *unused, double *out_5509509881336684506);
void live_h_13(double *state, double *unused, double *out_4932513073229844092);
void live_H_13(double *state, double *unused, double *out_4790124786535625632);
void live_h_14(double *state, double *unused, double *out_9174977010257973582);
void live_H_14(double *state, double *unused, double *out_3100828963580758044);
void live_h_33(double *state, double *unused, double *out_7269419265999335898);
void live_H_33(double *state, double *unused, double *out_527528546150627594);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}