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
void live_H(double *in_vec, double *out_5302544321594479534);
void live_err_fun(double *nom_x, double *delta_x, double *out_6624298405754516592);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6558760121132932202);
void live_H_mod_fun(double *state, double *out_3786036583950803330);
void live_f_fun(double *state, double dt, double *out_3075981791437892926);
void live_F_fun(double *state, double dt, double *out_4027562194415646780);
void live_h_4(double *state, double *unused, double *out_8780624752193779551);
void live_H_4(double *state, double *unused, double *out_2362955627251342819);
void live_h_9(double *state, double *unused, double *out_6116139294226424418);
void live_H_9(double *state, double *unused, double *out_2121765980621752174);
void live_h_10(double *state, double *unused, double *out_2930689772553497704);
void live_H_10(double *state, double *unused, double *out_8895818423172635159);
void live_h_12(double *state, double *unused, double *out_1600955695003018221);
void live_H_12(double *state, double *unused, double *out_2656500780780618976);
void live_h_35(double *state, double *unused, double *out_7412121649927701893);
void live_H_35(double *state, double *unused, double *out_1003706430121264557);
void live_h_32(double *state, double *unused, double *out_5348960732313578027);
void live_H_32(double *state, double *unused, double *out_8185721977524050278);
void live_h_13(double *state, double *unused, double *out_2664720871132804126);
void live_H_13(double *state, double *unused, double *out_6977257645613394591);
void live_h_14(double *state, double *unused, double *out_6116139294226424418);
void live_H_14(double *state, double *unused, double *out_2121765980621752174);
void live_h_33(double *state, double *unused, double *out_808389070225430007);
void live_H_33(double *state, double *unused, double *out_4154263434760122161);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}