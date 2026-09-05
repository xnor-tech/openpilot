#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1038245486379722605);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6881182548000141707);
void pose_H_mod_fun(double *state, double *out_4548887689658881741);
void pose_f_fun(double *state, double dt, double *out_8947099584724172048);
void pose_F_fun(double *state, double dt, double *out_7639000112762156858);
void pose_h_4(double *state, double *unused, double *out_7743631117813214181);
void pose_H_4(double *state, double *unused, double *out_3699910554686061615);
void pose_h_10(double *state, double *unused, double *out_7197434803533860657);
void pose_H_10(double *state, double *unused, double *out_7135399281517169195);
void pose_h_13(double *state, double *unused, double *out_3579424207122696687);
void pose_H_13(double *state, double *unused, double *out_6912184380018394416);
void pose_h_14(double *state, double *unused, double *out_6838227868173306480);
void pose_H_14(double *state, double *unused, double *out_7663151411025546144);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}