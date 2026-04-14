#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6197317699744068790);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8495458674857675788);
void pose_H_mod_fun(double *state, double *out_7217558117937058871);
void pose_f_fun(double *state, double dt, double *out_9009937482874928922);
void pose_F_fun(double *state, double dt, double *out_5828535467002130009);
void pose_h_4(double *state, double *unused, double *out_437538278502993915);
void pose_H_4(double *state, double *unused, double *out_6614558918843176381);
void pose_h_10(double *state, double *unused, double *out_3185345221758858216);
void pose_H_10(double *state, double *unused, double *out_2487017710939471116);
void pose_h_13(double *state, double *unused, double *out_8589966826250431028);
void pose_H_13(double *state, double *unused, double *out_996072289473524548);
void pose_h_14(double *state, double *unused, double *out_3652909299986341134);
void pose_H_14(double *state, double *unused, double *out_2651318062503691852);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}