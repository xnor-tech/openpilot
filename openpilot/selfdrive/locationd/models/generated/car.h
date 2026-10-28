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
void car_err_fun(double *nom_x, double *delta_x, double *out_860827029368682551);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8858530749727553508);
void car_H_mod_fun(double *state, double *out_3714675639127096598);
void car_f_fun(double *state, double dt, double *out_4730484258686567608);
void car_F_fun(double *state, double dt, double *out_8913108992009854130);
void car_h_25(double *state, double *unused, double *out_1867293235320502834);
void car_H_25(double *state, double *unused, double *out_6919591383865390944);
void car_h_24(double *state, double *unused, double *out_3213342894231868787);
void car_H_24(double *state, double *unused, double *out_3366230609664718215);
void car_h_30(double *state, double *unused, double *out_4577317895700990437);
void car_H_30(double *state, double *unused, double *out_9008819731336912045);
void car_h_26(double *state, double *unused, double *out_5664044544057454764);
void car_H_26(double *state, double *unused, double *out_3178088064991334720);
void car_h_27(double *state, double *unused, double *out_8266334752734351980);
void car_H_27(double *state, double *unused, double *out_7263161030572214660);
void car_h_29(double *state, double *unused, double *out_8724536114892913642);
void car_H_29(double *state, double *unused, double *out_8498588387022519861);
void car_h_28(double *state, double *unused, double *out_7663018868057874463);
void car_H_28(double *state, double *unused, double *out_4865756669617501181);
void car_h_31(double *state, double *unused, double *out_5504742763574525184);
void car_H_31(double *state, double *unused, double *out_2551879962757983244);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}