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
void car_err_fun(double *nom_x, double *delta_x, double *out_7098692201925828218);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8974564382236626677);
void car_H_mod_fun(double *state, double *out_4637657594296017115);
void car_f_fun(double *state, double dt, double *out_5625388813302586479);
void car_F_fun(double *state, double dt, double *out_3506019992486908746);
void car_h_25(double *state, double *unused, double *out_1659760556682213941);
void car_H_25(double *state, double *unused, double *out_2386459148589029399);
void car_h_24(double *state, double *unused, double *out_8231109947655627744);
void car_H_24(double *state, double *unused, double *out_6841606037480165826);
void car_h_30(double *state, double *unused, double *out_4852978595674001882);
void car_H_30(double *state, double *unused, double *out_6914155478716637597);
void car_h_26(double *state, double *unused, double *out_4167564816766474560);
void car_H_26(double *state, double *unused, double *out_6127962467463085623);
void car_h_27(double *state, double *unused, double *out_6057712637963842953);
void car_H_27(double *state, double *unused, double *out_9088918790517062508);
void car_h_29(double *state, double *unused, double *out_5900525969612713808);
void car_H_29(double *state, double *unused, double *out_6403924134402245413);
void car_h_28(double *state, double *unused, double *out_3067473329497799704);
void car_H_28(double *state, double *unused, double *out_6960420922237775629);
void car_h_31(double *state, double *unused, double *out_2506670762931880475);
void car_H_31(double *state, double *unused, double *out_6754170569696437099);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}