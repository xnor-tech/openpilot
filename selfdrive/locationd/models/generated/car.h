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
void car_err_fun(double *nom_x, double *delta_x, double *out_4700612312330196268);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1555638159944357333);
void car_H_mod_fun(double *state, double *out_5825955621798686393);
void car_f_fun(double *state, double dt, double *out_2916287295467583771);
void car_F_fun(double *state, double dt, double *out_6878950951447676039);
void car_h_25(double *state, double *unused, double *out_4274570641008688092);
void car_H_25(double *state, double *unused, double *out_7717096626726200383);
void car_h_24(double *state, double *unused, double *out_2799824959515052510);
void car_H_24(double *state, double *unused, double *out_5860832581955644381);
void car_h_30(double *state, double *unused, double *out_4431757309359817237);
void car_H_30(double *state, double *unused, double *out_6201951116855743035);
void car_h_26(double *state, double *unused, double *out_1394971257915484552);
void car_H_26(double *state, double *unused, double *out_6988144128109295009);
void car_h_27(double *state, double *unused, double *out_3374774532302646250);
void car_H_27(double *state, double *unused, double *out_4027187805055318124);
void car_h_29(double *state, double *unused, double *out_7012224060556668600);
void car_H_29(double *state, double *unused, double *out_6712182461170135219);
void car_h_28(double *state, double *unused, double *out_5793520145370500276);
void car_H_28(double *state, double *unused, double *out_8675812732735461470);
void car_h_31(double *state, double *unused, double *out_8214726661834494558);
void car_H_31(double *state, double *unused, double *out_6361936025875943533);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}