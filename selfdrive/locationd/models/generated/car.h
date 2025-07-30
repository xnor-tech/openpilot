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
void car_err_fun(double *nom_x, double *delta_x, double *out_2325115720415564338);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4599327397528124256);
void car_H_mod_fun(double *state, double *out_3603362443890924865);
void car_f_fun(double *state, double dt, double *out_3060737343829631213);
void car_F_fun(double *state, double dt, double *out_1388818207155273058);
void car_h_25(double *state, double *unused, double *out_5000231162990336464);
void car_H_25(double *state, double *unused, double *out_7126343093880416542);
void car_h_24(double *state, double *unused, double *out_3452317047508392249);
void car_H_24(double *state, double *unused, double *out_2379833766913907508);
void car_h_30(double *state, double *unused, double *out_1770604063360014472);
void car_H_30(double *state, double *unused, double *out_8802068021321886447);
void car_h_26(double *state, double *unused, double *out_827094210458352037);
void car_H_26(double *state, double *unused, double *out_3384839775006360318);
void car_h_27(double *state, double *unused, double *out_8444839643058369507);
void car_H_27(double *state, double *unused, double *out_6578473950137943230);
void car_h_29(double *state, double *unused, double *out_3231069204300831173);
void car_H_29(double *state, double *unused, double *out_8291836677007494263);
void car_h_28(double *state, double *unused, double *out_8311456595915501149);
void car_H_28(double *state, double *unused, double *out_5072508379632526779);
void car_h_31(double *state, double *unused, double *out_2052505172634851503);
void car_H_31(double *state, double *unused, double *out_7156989055757376970);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}