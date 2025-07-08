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
void car_err_fun(double *nom_x, double *delta_x, double *out_8162787069879679621);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4646430404185096575);
void car_H_mod_fun(double *state, double *out_8634146753423524100);
void car_f_fun(double *state, double dt, double *out_6186994550747992320);
void car_F_fun(double *state, double dt, double *out_1846682865561113223);
void car_h_25(double *state, double *unused, double *out_4028775186055664403);
void car_H_25(double *state, double *unused, double *out_4908905495101362676);
void car_h_24(double *state, double *unused, double *out_7860148482378442816);
void car_H_24(double *state, double *unused, double *out_2731691071494212703);
void car_h_30(double *state, double *unused, double *out_4107489310009488734);
void car_H_30(double *state, double *unused, double *out_2390572536594114049);
void car_h_26(double *state, double *unused, double *out_9109162577670334379);
void car_H_26(double *state, double *unused, double *out_8650408813975418900);
void car_h_27(double *state, double *unused, double *out_972898081605181242);
void car_H_27(double *state, double *unused, double *out_4565335848394538960);
void car_h_29(double *state, double *unused, double *out_7523762599658558750);
void car_H_29(double *state, double *unused, double *out_1880341192279721865);
void car_h_28(double *state, double *unused, double *out_6117788144720828779);
void car_H_28(double *state, double *unused, double *out_6962740209349252439);
void car_h_31(double *state, double *unused, double *out_2820532584691991347);
void car_H_31(double *state, double *unused, double *out_9170127157500781240);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}