#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4705197801635651257);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4283461203773073958);
void pose_H_mod_fun(double *state, double *out_3022793235977438013);
void pose_f_fun(double *state, double dt, double *out_3860426828173334801);
void pose_F_fun(double *state, double dt, double *out_2434308811775089426);
void pose_h_4(double *state, double *unused, double *out_480079106382461241);
void pose_H_4(double *state, double *unused, double *out_5921568430080383329);
void pose_h_10(double *state, double *unused, double *out_5059097033008134263);
void pose_H_10(double *state, double *unused, double *out_3858084587156191705);
void pose_h_13(double *state, double *unused, double *out_7754600168369625286);
void pose_H_13(double *state, double *unused, double *out_2709294604748050528);
void pose_h_14(double *state, double *unused, double *out_5590979122691983286);
void pose_H_14(double *state, double *unused, double *out_1958327573740898800);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}