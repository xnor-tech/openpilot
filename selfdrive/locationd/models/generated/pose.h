#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_9147823097001100337);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6976768038486665276);
void pose_H_mod_fun(double *state, double *out_2524355098589883270);
void pose_f_fun(double *state, double dt, double *out_2560858857775655991);
void pose_F_fun(double *state, double dt, double *out_6720701162118872845);
void pose_h_4(double *state, double *unused, double *out_4511036304417573681);
void pose_H_4(double *state, double *unused, double *out_7722084091146443579);
void pose_h_10(double *state, double *unused, double *out_2196926179427311242);
void pose_H_10(double *state, double *unused, double *out_93714966278450490);
void pose_h_13(double *state, double *unused, double *out_6528786097031275213);
void pose_H_13(double *state, double *unused, double *out_111452882829742650);
void pose_h_14(double *state, double *unused, double *out_4619261727091989870);
void pose_H_14(double *state, double *unused, double *out_3758843234806959050);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}