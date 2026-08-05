#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4833332473031117184);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3480076499285872836);
void pose_H_mod_fun(double *state, double *out_1421822751067225790);
void pose_f_fun(double *state, double dt, double *out_3631502472880784260);
void pose_F_fun(double *state, double dt, double *out_1790462606867554083);
void pose_h_4(double *state, double *unused, double *out_8428208003072760504);
void pose_H_4(double *state, double *unused, double *out_7522538914990595552);
void pose_h_10(double *state, double *unused, double *out_5381702994242187932);
void pose_H_10(double *state, double *unused, double *out_4110512254011380820);
void pose_h_13(double *state, double *unused, double *out_1537117227709853335);
void pose_H_13(double *state, double *unused, double *out_4310265089658262751);
void pose_h_14(double *state, double *unused, double *out_4391412999592827342);
void pose_H_14(double *state, double *unused, double *out_7841416726423583768);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}