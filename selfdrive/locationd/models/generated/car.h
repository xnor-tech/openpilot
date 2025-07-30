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
void car_err_fun(double *nom_x, double *delta_x, double *out_2247958235058554131);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1704160864652915340);
void car_H_mod_fun(double *state, double *out_5921766340727629681);
void car_f_fun(double *state, double dt, double *out_7705802625381580173);
void car_F_fun(double *state, double dt, double *out_1155867711308474055);
void car_h_25(double *state, double *unused, double *out_5577598147182567834);
void car_H_25(double *state, double *unused, double *out_3271421885184607556);
void car_h_24(double *state, double *unused, double *out_3902841565486989302);
void car_H_24(double *state, double *unused, double *out_5444071484190107122);
void car_h_30(double *state, double *unused, double *out_66466683301832023);
void car_H_30(double *state, double *unused, double *out_753088926677358929);
void car_h_26(double *state, double *unused, double *out_4358894231996399510);
void car_H_26(double *state, double *unused, double *out_7012925204058663780);
void car_h_27(double *state, double *unused, double *out_2071747026128766508);
void car_H_27(double *state, double *unused, double *out_2927852238477783840);
void car_h_29(double *state, double *unused, double *out_5933430502557268160);
void car_H_29(double *state, double *unused, double *out_242857582362966745);
void car_h_28(double *state, double *unused, double *out_3053831119464064620);
void car_H_28(double *state, double *unused, double *out_5325256599432497319);
void car_h_31(double *state, double *unused, double *out_9215047675436590184);
void car_H_31(double *state, double *unused, double *out_7639133306292015256);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}