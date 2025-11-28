#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7199008258596550885);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1195653358946799825);
void pose_H_mod_fun(double *state, double *out_4022220117762460438);
void pose_f_fun(double *state, double dt, double *out_573638959019262461);
void pose_F_fun(double *state, double dt, double *out_7727313238387066272);
void pose_h_4(double *state, double *unused, double *out_1574587782542937358);
void pose_H_4(double *state, double *unused, double *out_9174738502801541173);
void pose_h_10(double *state, double *unused, double *out_9067770934847269573);
void pose_H_10(double *state, double *unused, double *out_3107776501730799288);
void pose_h_13(double *state, double *unused, double *out_1089679770148776292);
void pose_H_13(double *state, double *unused, double *out_6059731745575677642);
void pose_h_14(double *state, double *unused, double *out_41555095050834232);
void pose_H_14(double *state, double *unused, double *out_6091950070506168877);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}