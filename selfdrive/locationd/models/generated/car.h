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
void car_err_fun(double *nom_x, double *delta_x, double *out_8231570846104287187);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3172369976442532592);
void car_H_mod_fun(double *state, double *out_3076533359310538874);
void car_f_fun(double *state, double dt, double *out_4555482552254326600);
void car_F_fun(double *state, double dt, double *out_3328421804098371723);
void car_h_25(double *state, double *unused, double *out_8332545656712404133);
void car_H_25(double *state, double *unused, double *out_7871364202238407983);
void car_h_24(double *state, double *unused, double *out_8057594001447114148);
void car_H_24(double *state, double *unused, double *out_8402730272465644067);
void car_h_30(double *state, double *unused, double *out_7731127499099040579);
void car_H_30(double *state, double *unused, double *out_5353031243731159356);
void car_h_26(double *state, double *unused, double *out_8895494501810979159);
void car_H_26(double *state, double *unused, double *out_6833876552597087409);
void car_h_27(double *state, double *unused, double *out_8935861541388881650);
void car_H_27(double *state, double *unused, double *out_7527794555531584267);
void car_h_29(double *state, double *unused, double *out_9211055603673387539);
void car_H_29(double *state, double *unused, double *out_4842799899416767172);
void car_h_28(double *state, double *unused, double *out_7849134893032417323);
void car_H_28(double *state, double *unused, double *out_8521545157223253870);
void car_h_31(double *state, double *unused, double *out_4695096128458381783);
void car_H_31(double *state, double *unused, double *out_6207668450363735933);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}