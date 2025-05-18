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
void car_err_fun(double *nom_x, double *delta_x, double *out_2728693116701389222);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3002832702974128596);
void car_H_mod_fun(double *state, double *out_7934990902020908970);
void car_f_fun(double *state, double dt, double *out_2890300663064821146);
void car_F_fun(double *state, double dt, double *out_6771328482541706301);
void car_h_25(double *state, double *unused, double *out_850067438180914005);
void car_H_25(double *state, double *unused, double *out_7059682157681905990);
void car_h_24(double *state, double *unused, double *out_347899436057359381);
void car_H_24(double *state, double *unused, double *out_3501756558879582854);
void car_h_30(double *state, double *unused, double *out_6610491326067974320);
void car_H_30(double *state, double *unused, double *out_4541349199174657363);
void car_h_26(double *state, double *unused, double *out_6834579479980731651);
void car_H_26(double *state, double *unused, double *out_7645558597153589402);
void car_h_27(double *state, double *unused, double *out_1754337882627783646);
void car_H_27(double *state, double *unused, double *out_2317755127990714146);
void car_h_29(double *state, double *unused, double *out_1608818058407128675);
void car_H_29(double *state, double *unused, double *out_4031117854860265179);
void car_h_28(double *state, double *unused, double *out_4908230852940097215);
void car_H_28(double *state, double *unused, double *out_9113516871929795753);
void car_h_31(double *state, double *unused, double *out_1007254106532043150);
void car_H_31(double *state, double *unused, double *out_7019350494920237926);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}