#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2691520434775550490);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2397415840402016948);
void pose_H_mod_fun(double *state, double *out_2601610123313846998);
void pose_f_fun(double *state, double dt, double *out_4760475200250568711);
void pose_F_fun(double *state, double dt, double *out_3582442533271693884);
void pose_h_4(double *state, double *unused, double *out_2509617360494600185);
void pose_H_4(double *state, double *unused, double *out_7754128508352927733);
void pose_h_10(double *state, double *unused, double *out_8277461064165376406);
void pose_H_10(double *state, double *unused, double *out_1041335713495420488);
void pose_h_13(double *state, double *unused, double *out_5178744377390213500);
void pose_H_13(double *state, double *unused, double *out_3920373045050403709);
void pose_h_14(double *state, double *unused, double *out_7113706599292876475);
void pose_H_14(double *state, double *unused, double *out_4671340076057555437);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}