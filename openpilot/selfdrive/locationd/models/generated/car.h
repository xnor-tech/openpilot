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
void car_err_fun(double *nom_x, double *delta_x, double *out_8681067850503655264);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_938197407851416607);
void car_H_mod_fun(double *state, double *out_424706054398540845);
void car_f_fun(double *state, double dt, double *out_6136556308702405802);
void car_F_fun(double *state, double dt, double *out_8341093536950357909);
void car_h_25(double *state, double *unused, double *out_2179954959961391454);
void car_H_25(double *state, double *unused, double *out_879633055793430775);
void car_h_24(double *state, double *unused, double *out_5873442620998829456);
void car_H_24(double *state, double *unused, double *out_7080665735644592201);
void car_h_30(double *state, double *unused, double *out_5817404488215413804);
void car_H_30(double *state, double *unused, double *out_5407329385921038973);
void car_h_26(double *state, double *unused, double *out_6760914341117076239);
void car_H_26(double *state, double *unused, double *out_4621136374667486999);
void car_h_27(double *state, double *unused, double *out_856831091482941231);
void car_H_27(double *state, double *unused, double *out_3183735314737095756);
void car_h_29(double *state, double *unused, double *out_581637029198435342);
void car_H_29(double *state, double *unused, double *out_4897098041606646789);
void car_h_28(double *state, double *unused, double *out_4144842176177904414);
void car_H_28(double *state, double *unused, double *out_2933467770041320538);
void car_h_31(double *state, double *unused, double *out_4407872966514903709);
void car_H_31(double *state, double *unused, double *out_848987093916470347);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}