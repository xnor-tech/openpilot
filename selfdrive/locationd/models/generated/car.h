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
void car_err_fun(double *nom_x, double *delta_x, double *out_2045986475089525769);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4725156432634517583);
void car_H_mod_fun(double *state, double *out_3827119867446887139);
void car_f_fun(double *state, double dt, double *out_864076176166956704);
void car_F_fun(double *state, double dt, double *out_6878955300494497585);
void car_h_25(double *state, double *unused, double *out_983273569991112921);
void car_H_25(double *state, double *unused, double *out_967710975480981970);
void car_h_24(double *state, double *unused, double *out_5323618623202364012);
void car_H_24(double *state, double *unused, double *out_3140360574486481536);
void car_h_30(double *state, double *unused, double *out_3552231351687777287);
void car_H_30(double *state, double *unused, double *out_1550621983026266657);
void car_h_26(double *state, double *unused, double *out_2948915467011299770);
void car_H_26(double *state, double *unused, double *out_2336814994279818631);
void car_h_27(double *state, double *unused, double *out_2333527436501608963);
void car_H_27(double *state, double *unused, double *out_624141328774158254);
void car_h_29(double *state, double *unused, double *out_5873969309581297887);
void car_H_29(double *state, double *unused, double *out_2060853327340658841);
void car_h_28(double *state, double *unused, double *out_270877884307088688);
void car_H_28(double *state, double *unused, double *out_3021545689728871733);
void car_h_31(double *state, double *unused, double *out_2878409906437388731);
void car_H_31(double *state, double *unused, double *out_5335422396588389670);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}