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
void car_err_fun(double *nom_x, double *delta_x, double *out_2728203783553228331);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6677308708783646085);
void car_H_mod_fun(double *state, double *out_7958488234531987342);
void car_f_fun(double *state, double dt, double *out_1281405791405578487);
void car_F_fun(double *state, double dt, double *out_8559667741810286640);
void car_h_25(double *state, double *unused, double *out_7841400153888334023);
void car_H_25(double *state, double *unused, double *out_5321931309516561148);
void car_h_24(double *state, double *unused, double *out_6356517643470785223);
void car_H_24(double *state, double *unused, double *out_6734959557554969985);
void car_h_30(double *state, double *unused, double *out_8116594216172839912);
void car_H_30(double *state, double *unused, double *out_794234979388952950);
void car_h_26(double *state, double *unused, double *out_7725744536728014053);
void car_H_26(double *state, double *unused, double *out_1580427990642504924);
void car_h_27(double *state, double *unused, double *out_3813100946189485814);
void car_H_27(double *state, double *unused, double *out_1380528332411471961);
void car_h_29(double *state, double *unused, double *out_4083698800076538236);
void car_H_29(double *state, double *unused, double *out_1304466323703345134);
void car_h_28(double *state, double *unused, double *out_1188572321471359831);
void car_H_28(double *state, double *unused, double *out_3268096595268671385);
void car_h_31(double *state, double *unused, double *out_1965466443102675709);
void car_H_31(double *state, double *unused, double *out_954219888409153448);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}