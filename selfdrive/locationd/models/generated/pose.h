#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_945005396424872191);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4622954603925097103);
void pose_H_mod_fun(double *state, double *out_3244724203827213501);
void pose_f_fun(double *state, double dt, double *out_5671114693680624888);
void pose_F_fun(double *state, double dt, double *out_8302195571783514857);
void pose_h_4(double *state, double *unused, double *out_1554668460974419477);
void pose_H_4(double *state, double *unused, double *out_44314302725743477);
void pose_h_10(double *state, double *unused, double *out_2262676311495278110);
void pose_H_10(double *state, double *unused, double *out_3242013822092943294);
void pose_h_13(double *state, double *unused, double *out_8592105098224939032);
void pose_H_13(double *state, double *unused, double *out_7654945511042444406);
void pose_h_14(double *state, double *unused, double *out_4909758137732599256);
void pose_H_14(double *state, double *unused, double *out_3038474129569628819);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}