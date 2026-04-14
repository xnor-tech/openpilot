#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8605324444285015125);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5007064384226221423);
void car_H_mod_fun(double *state, double *out_4661944159559162272);
void car_f_fun(double *state, double dt, double *out_8472733918882151860);
void car_F_fun(double *state, double dt, double *out_9120236176575515967);
void car_h_25(double *state, double *unused, double *out_7967351267121302732);
void car_H_25(double *state, double *unused, double *out_4156609717358865560);
void car_h_24(double *state, double *unused, double *out_8185183297069036004);
void car_H_24(double *state, double *unused, double *out_7200926927363213752);
void car_h_30(double *state, double *unused, double *out_8242545329405808621);
void car_H_30(double *state, double *unused, double *out_4285948664502105630);
void car_h_26(double *state, double *unused, double *out_3805157226889893849);
void car_H_26(double *state, double *unused, double *out_7898113036232921784);
void car_h_27(double *state, double *unused, double *out_7297447274010470293);
void car_H_27(double *state, double *unused, double *out_6460711976302530541);
void car_h_29(double *state, double *unused, double *out_5205759277961475936);
void car_H_29(double *state, double *unused, double *out_3775717320187713446);
void car_h_28(double *state, double *unused, double *out_6291555153765214756);
void car_H_28(double *state, double *unused, double *out_8858116337257244020);
void car_h_31(double *state, double *unused, double *out_1341675521904397702);
void car_H_31(double *state, double *unused, double *out_4125963755481905132);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}