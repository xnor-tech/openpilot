#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_871712568218649107);
void live_err_fun(double *nom_x, double *delta_x, double *out_8503236614380067032);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4916294092235343299);
void live_H_mod_fun(double *state, double *out_8576549074293786362);
void live_f_fun(double *state, double dt, double *out_3433285011158640522);
void live_F_fun(double *state, double dt, double *out_6783427363923154403);
void live_h_4(double *state, double *unused, double *out_6755430735170836570);
void live_H_4(double *state, double *unused, double *out_7430751619963087523);
void live_h_9(double *state, double *unused, double *out_5425329145443550723);
void live_H_9(double *state, double *unused, double *out_7189561973333496878);
void live_h_10(double *state, double *unused, double *out_3318153889851115375);
void live_H_10(double *state, double *unused, double *out_8092790845840902208);
void live_h_12(double *state, double *unused, double *out_5933520425269038577);
void live_H_12(double *state, double *unused, double *out_2411295211931125728);
void live_h_35(double *state, double *unused, double *out_558091817221700268);
void live_H_35(double *state, double *unused, double *out_4064089562590480147);
void live_h_32(double *state, double *unused, double *out_102557069949095671);
void live_H_32(double *state, double *unused, double *out_21646125506442953);
void live_h_13(double *state, double *unused, double *out_5013775764841320191);
void live_H_13(double *state, double *unused, double *out_1977267102934445986);
void live_h_14(double *state, double *unused, double *out_5425329145443550723);
void live_H_14(double *state, double *unused, double *out_7189561973333496878);
void live_h_33(double *state, double *unused, double *out_533947481512312488);
void live_H_33(double *state, double *unused, double *out_913532557951622543);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}