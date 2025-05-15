#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2762310420758368335);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4946458637261953472);
void pose_H_mod_fun(double *state, double *out_4775082209311112640);
void pose_f_fun(double *state, double dt, double *out_8301426414689752095);
void pose_F_fun(double *state, double dt, double *out_3550420757029977421);
void pose_h_4(double *state, double *unused, double *out_6563430256492903056);
void pose_H_4(double *state, double *unused, double *out_4169279456746708702);
void pose_h_10(double *state, double *unused, double *out_779779828848067105);
void pose_H_10(double *state, double *unused, double *out_1944061734434103698);
void pose_h_13(double *state, double *unused, double *out_8926596776619908930);
void pose_H_13(double *state, double *unused, double *out_957005631414375901);
void pose_h_14(double *state, double *unused, double *out_1164946293048257452);
void pose_H_14(double *state, double *unused, double *out_7252067889042080998);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}