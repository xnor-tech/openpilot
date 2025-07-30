#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_235430937466886576);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2450675293636270791);
void pose_H_mod_fun(double *state, double *out_530647733167696860);
void pose_f_fun(double *state, double dt, double *out_9203828947486336506);
void pose_F_fun(double *state, double dt, double *out_4423182636226594510);
void pose_h_4(double *state, double *unused, double *out_4884014442507138413);
void pose_H_4(double *state, double *unused, double *out_2350346459073993721);
void pose_h_10(double *state, double *unused, double *out_6338696543292411763);
void pose_H_10(double *state, double *unused, double *out_2306543916110426020);
void pose_h_13(double *state, double *unused, double *out_8804592808846089421);
void pose_H_13(double *state, double *unused, double *out_861927366258339080);
void pose_h_14(double *state, double *unused, double *out_5486189001180777325);
void pose_H_14(double *state, double *unused, double *out_1612894397265490808);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}