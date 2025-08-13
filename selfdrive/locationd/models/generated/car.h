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
void car_err_fun(double *nom_x, double *delta_x, double *out_2612944280414334461);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5198894641976812856);
void car_H_mod_fun(double *state, double *out_4469793830309029803);
void car_f_fun(double *state, double dt, double *out_4731745578040490156);
void car_F_fun(double *state, double dt, double *out_8512268445394675783);
void car_h_25(double *state, double *unused, double *out_1706109444363025734);
void car_H_25(double *state, double *unused, double *out_946813180319538126);
void car_h_24(double *state, double *unused, double *out_6287632968672598585);
void car_H_24(double *state, double *unused, double *out_2611112418482785010);
void car_h_30(double *state, double *unused, double *out_9166668208611914059);
void car_H_30(double *state, double *unused, double *out_5969877161172078629);
void car_h_26(double *state, double *unused, double *out_4278402597436791912);
void car_H_26(double *state, double *unused, double *out_4688316499193594350);
void car_h_27(double *state, double *unused, double *out_2649619297264688169);
void car_H_27(double *state, double *unused, double *out_3795113849371653718);
void car_h_29(double *state, double *unused, double *out_4100250689893571336);
void car_H_29(double *state, double *unused, double *out_2081751122502102685);
void car_h_28(double *state, double *unused, double *out_5254024618073385820);
void car_H_28(double *state, double *unused, double *out_3000647894567427889);
void car_h_31(double *state, double *unused, double *out_6933768032355516150);
void car_H_31(double *state, double *unused, double *out_916167218442577698);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}