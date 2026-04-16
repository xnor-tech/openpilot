#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4779041233668355035);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6150076879653088119);
void pose_H_mod_fun(double *state, double *out_6193522935070858792);
void pose_f_fun(double *state, double dt, double *out_3399450163802279159);
void pose_F_fun(double *state, double dt, double *out_7117793343438049369);
void pose_h_4(double *state, double *unused, double *out_7151678483254898760);
void pose_H_4(double *state, double *unused, double *out_5324698458947972018);
void pose_h_10(double *state, double *unused, double *out_2988941107398924586);
void pose_H_10(double *state, double *unused, double *out_7064205239493264161);
void pose_h_13(double *state, double *unused, double *out_8132861882635179526);
void pose_H_13(double *state, double *unused, double *out_2112424633615639217);
void pose_h_14(double *state, double *unused, double *out_7493722489878846780);
void pose_H_14(double *state, double *unused, double *out_5640899799481839174);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}