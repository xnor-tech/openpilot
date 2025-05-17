#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4864162633936306843);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9066787371279446197);
void pose_H_mod_fun(double *state, double *out_8494019208454287783);
void pose_f_fun(double *state, double dt, double *out_7093516037157220813);
void pose_F_fun(double *state, double dt, double *out_3635578863113740513);
void pose_h_4(double *state, double *unused, double *out_8924931421822357051);
void pose_H_4(double *state, double *unused, double *out_9198563863200551226);
void pose_h_10(double *state, double *unused, double *out_457631932626616900);
void pose_H_10(double *state, double *unused, double *out_234197665491246895);
void pose_h_13(double *state, double *unused, double *out_5982858872629872588);
void pose_H_13(double *state, double *unused, double *out_5986290037868218425);
void pose_h_14(double *state, double *unused, double *out_8441296205878297134);
void pose_H_14(double *state, double *unused, double *out_5235323006861066697);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}