#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8353524553736122963);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8983699723827009156);
void pose_H_mod_fun(double *state, double *out_2968799295363009369);
void pose_f_fun(double *state, double dt, double *out_4017585182654342073);
void pose_F_fun(double *state, double dt, double *out_6528773045854873690);
void pose_h_4(double *state, double *unused, double *out_2292486705639675299);
void pose_H_4(double *state, double *unused, double *out_8072824320030371515);
void pose_h_10(double *state, double *unused, double *out_3625759173810396060);
void pose_H_10(double *state, double *unused, double *out_4958904843940480188);
void pose_h_13(double *state, double *unused, double *out_2109651969206804491);
void pose_H_13(double *state, double *unused, double *out_2763288545362479172);
void pose_h_14(double *state, double *unused, double *out_2205317381569854105);
void pose_H_14(double *state, double *unused, double *out_4990035887734999219);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}