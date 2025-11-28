#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2799864307103698151);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5877455747313980100);
void pose_H_mod_fun(double *state, double *out_6139442128438511871);
void pose_f_fun(double *state, double dt, double *out_3038167595385286646);
void pose_F_fun(double *state, double dt, double *out_33138222388702694);
void pose_h_4(double *state, double *unused, double *out_3971679263271049865);
void pose_H_4(double *state, double *unused, double *out_2939032227337041847);
void pose_h_10(double *state, double *unused, double *out_2701029870014423504);
void pose_H_10(double *state, double *unused, double *out_161399328086974098);
void pose_h_13(double *state, double *unused, double *out_7398716565882038657);
void pose_H_13(double *state, double *unused, double *out_3503634147018885951);
void pose_h_14(double *state, double *unused, double *out_8045481680876496902);
void pose_H_14(double *state, double *unused, double *out_143756204958330449);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}