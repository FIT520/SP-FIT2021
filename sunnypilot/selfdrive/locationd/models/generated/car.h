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
void car_err_fun(double *nom_x, double *delta_x, double *out_5738968429187976759);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8045877841605503682);
void car_H_mod_fun(double *state, double *out_259521190058915161);
void car_f_fun(double *state, double dt, double *out_6851437618939961373);
void car_F_fun(double *state, double dt, double *out_7680386554523918165);
void car_h_25(double *state, double *unused, double *out_3477296093554491887);
void car_H_25(double *state, double *unused, double *out_8994034657738399218);
void car_h_24(double *state, double *unused, double *out_3675004507071379308);
void car_H_24(double *state, double *unused, double *out_4247525330771890184);
void car_h_30(double *state, double *unused, double *out_4879260551704948855);
void car_H_30(double *state, double *unused, double *out_2536019074479535643);
void car_h_26(double *state, double *unused, double *out_8565810254343757116);
void car_H_26(double *state, double *unused, double *out_5252531338864342994);
void car_h_27(double *state, double *unused, double *out_4742701018348891141);
void car_H_27(double *state, double *unused, double *out_4710782386279960554);
void car_h_29(double *state, double *unused, double *out_5822770404606611290);
void car_H_29(double *state, double *unused, double *out_6424145113149511587);
void car_h_28(double *state, double *unused, double *out_8702369787699814830);
void car_H_28(double *state, double *unused, double *out_6940199943490509455);
void car_h_31(double *state, double *unused, double *out_3485422862729087140);
void car_H_31(double *state, double *unused, double *out_9024680619615359646);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}