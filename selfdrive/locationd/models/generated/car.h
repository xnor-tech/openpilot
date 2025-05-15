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
void car_err_fun(double *nom_x, double *delta_x, double *out_7469656253560156198);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6624039366772895003);
void car_H_mod_fun(double *state, double *out_344573687507074959);
void car_f_fun(double *state, double dt, double *out_2533881819271106710);
void car_F_fun(double *state, double dt, double *out_2677838494581999961);
void car_h_25(double *state, double *unused, double *out_1278707427595684251);
void car_H_25(double *state, double *unused, double *out_4405445685606331007);
void car_h_24(double *state, double *unused, double *out_7823599996595568809);
void car_H_24(double *state, double *unused, double *out_6622906754426293716);
void car_h_30(double *state, double *unused, double *out_1121520759244555106);
void car_H_30(double *state, double *unused, double *out_122250644521277191);
void car_h_26(double *state, double *unused, double *out_7520562276658404252);
void car_H_26(double *state, double *unused, double *out_663942366732274783);
void car_h_27(double *state, double *unused, double *out_5468512726416157531);
void car_H_27(double *state, double *unused, double *out_2297013956321702102);
void car_h_29(double *state, double *unused, double *out_5743706788700663420);
void car_H_29(double *state, double *unused, double *out_387980699793114993);
void car_h_28(double *state, double *unused, double *out_7099945859734678997);
void car_H_28(double *state, double *unused, double *out_2351610971358441244);
void car_h_31(double *state, double *unused, double *out_3394844017673189766);
void car_H_31(double *state, double *unused, double *out_37734264498923307);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}