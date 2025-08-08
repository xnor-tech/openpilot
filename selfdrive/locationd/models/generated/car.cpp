#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2247958235058554131) {
   out_2247958235058554131[0] = delta_x[0] + nom_x[0];
   out_2247958235058554131[1] = delta_x[1] + nom_x[1];
   out_2247958235058554131[2] = delta_x[2] + nom_x[2];
   out_2247958235058554131[3] = delta_x[3] + nom_x[3];
   out_2247958235058554131[4] = delta_x[4] + nom_x[4];
   out_2247958235058554131[5] = delta_x[5] + nom_x[5];
   out_2247958235058554131[6] = delta_x[6] + nom_x[6];
   out_2247958235058554131[7] = delta_x[7] + nom_x[7];
   out_2247958235058554131[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1704160864652915340) {
   out_1704160864652915340[0] = -nom_x[0] + true_x[0];
   out_1704160864652915340[1] = -nom_x[1] + true_x[1];
   out_1704160864652915340[2] = -nom_x[2] + true_x[2];
   out_1704160864652915340[3] = -nom_x[3] + true_x[3];
   out_1704160864652915340[4] = -nom_x[4] + true_x[4];
   out_1704160864652915340[5] = -nom_x[5] + true_x[5];
   out_1704160864652915340[6] = -nom_x[6] + true_x[6];
   out_1704160864652915340[7] = -nom_x[7] + true_x[7];
   out_1704160864652915340[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5921766340727629681) {
   out_5921766340727629681[0] = 1.0;
   out_5921766340727629681[1] = 0.0;
   out_5921766340727629681[2] = 0.0;
   out_5921766340727629681[3] = 0.0;
   out_5921766340727629681[4] = 0.0;
   out_5921766340727629681[5] = 0.0;
   out_5921766340727629681[6] = 0.0;
   out_5921766340727629681[7] = 0.0;
   out_5921766340727629681[8] = 0.0;
   out_5921766340727629681[9] = 0.0;
   out_5921766340727629681[10] = 1.0;
   out_5921766340727629681[11] = 0.0;
   out_5921766340727629681[12] = 0.0;
   out_5921766340727629681[13] = 0.0;
   out_5921766340727629681[14] = 0.0;
   out_5921766340727629681[15] = 0.0;
   out_5921766340727629681[16] = 0.0;
   out_5921766340727629681[17] = 0.0;
   out_5921766340727629681[18] = 0.0;
   out_5921766340727629681[19] = 0.0;
   out_5921766340727629681[20] = 1.0;
   out_5921766340727629681[21] = 0.0;
   out_5921766340727629681[22] = 0.0;
   out_5921766340727629681[23] = 0.0;
   out_5921766340727629681[24] = 0.0;
   out_5921766340727629681[25] = 0.0;
   out_5921766340727629681[26] = 0.0;
   out_5921766340727629681[27] = 0.0;
   out_5921766340727629681[28] = 0.0;
   out_5921766340727629681[29] = 0.0;
   out_5921766340727629681[30] = 1.0;
   out_5921766340727629681[31] = 0.0;
   out_5921766340727629681[32] = 0.0;
   out_5921766340727629681[33] = 0.0;
   out_5921766340727629681[34] = 0.0;
   out_5921766340727629681[35] = 0.0;
   out_5921766340727629681[36] = 0.0;
   out_5921766340727629681[37] = 0.0;
   out_5921766340727629681[38] = 0.0;
   out_5921766340727629681[39] = 0.0;
   out_5921766340727629681[40] = 1.0;
   out_5921766340727629681[41] = 0.0;
   out_5921766340727629681[42] = 0.0;
   out_5921766340727629681[43] = 0.0;
   out_5921766340727629681[44] = 0.0;
   out_5921766340727629681[45] = 0.0;
   out_5921766340727629681[46] = 0.0;
   out_5921766340727629681[47] = 0.0;
   out_5921766340727629681[48] = 0.0;
   out_5921766340727629681[49] = 0.0;
   out_5921766340727629681[50] = 1.0;
   out_5921766340727629681[51] = 0.0;
   out_5921766340727629681[52] = 0.0;
   out_5921766340727629681[53] = 0.0;
   out_5921766340727629681[54] = 0.0;
   out_5921766340727629681[55] = 0.0;
   out_5921766340727629681[56] = 0.0;
   out_5921766340727629681[57] = 0.0;
   out_5921766340727629681[58] = 0.0;
   out_5921766340727629681[59] = 0.0;
   out_5921766340727629681[60] = 1.0;
   out_5921766340727629681[61] = 0.0;
   out_5921766340727629681[62] = 0.0;
   out_5921766340727629681[63] = 0.0;
   out_5921766340727629681[64] = 0.0;
   out_5921766340727629681[65] = 0.0;
   out_5921766340727629681[66] = 0.0;
   out_5921766340727629681[67] = 0.0;
   out_5921766340727629681[68] = 0.0;
   out_5921766340727629681[69] = 0.0;
   out_5921766340727629681[70] = 1.0;
   out_5921766340727629681[71] = 0.0;
   out_5921766340727629681[72] = 0.0;
   out_5921766340727629681[73] = 0.0;
   out_5921766340727629681[74] = 0.0;
   out_5921766340727629681[75] = 0.0;
   out_5921766340727629681[76] = 0.0;
   out_5921766340727629681[77] = 0.0;
   out_5921766340727629681[78] = 0.0;
   out_5921766340727629681[79] = 0.0;
   out_5921766340727629681[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7705802625381580173) {
   out_7705802625381580173[0] = state[0];
   out_7705802625381580173[1] = state[1];
   out_7705802625381580173[2] = state[2];
   out_7705802625381580173[3] = state[3];
   out_7705802625381580173[4] = state[4];
   out_7705802625381580173[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7705802625381580173[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7705802625381580173[7] = state[7];
   out_7705802625381580173[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1155867711308474055) {
   out_1155867711308474055[0] = 1;
   out_1155867711308474055[1] = 0;
   out_1155867711308474055[2] = 0;
   out_1155867711308474055[3] = 0;
   out_1155867711308474055[4] = 0;
   out_1155867711308474055[5] = 0;
   out_1155867711308474055[6] = 0;
   out_1155867711308474055[7] = 0;
   out_1155867711308474055[8] = 0;
   out_1155867711308474055[9] = 0;
   out_1155867711308474055[10] = 1;
   out_1155867711308474055[11] = 0;
   out_1155867711308474055[12] = 0;
   out_1155867711308474055[13] = 0;
   out_1155867711308474055[14] = 0;
   out_1155867711308474055[15] = 0;
   out_1155867711308474055[16] = 0;
   out_1155867711308474055[17] = 0;
   out_1155867711308474055[18] = 0;
   out_1155867711308474055[19] = 0;
   out_1155867711308474055[20] = 1;
   out_1155867711308474055[21] = 0;
   out_1155867711308474055[22] = 0;
   out_1155867711308474055[23] = 0;
   out_1155867711308474055[24] = 0;
   out_1155867711308474055[25] = 0;
   out_1155867711308474055[26] = 0;
   out_1155867711308474055[27] = 0;
   out_1155867711308474055[28] = 0;
   out_1155867711308474055[29] = 0;
   out_1155867711308474055[30] = 1;
   out_1155867711308474055[31] = 0;
   out_1155867711308474055[32] = 0;
   out_1155867711308474055[33] = 0;
   out_1155867711308474055[34] = 0;
   out_1155867711308474055[35] = 0;
   out_1155867711308474055[36] = 0;
   out_1155867711308474055[37] = 0;
   out_1155867711308474055[38] = 0;
   out_1155867711308474055[39] = 0;
   out_1155867711308474055[40] = 1;
   out_1155867711308474055[41] = 0;
   out_1155867711308474055[42] = 0;
   out_1155867711308474055[43] = 0;
   out_1155867711308474055[44] = 0;
   out_1155867711308474055[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1155867711308474055[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1155867711308474055[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1155867711308474055[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1155867711308474055[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1155867711308474055[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1155867711308474055[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1155867711308474055[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1155867711308474055[53] = -9.8000000000000007*dt;
   out_1155867711308474055[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1155867711308474055[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1155867711308474055[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1155867711308474055[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1155867711308474055[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1155867711308474055[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1155867711308474055[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1155867711308474055[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1155867711308474055[62] = 0;
   out_1155867711308474055[63] = 0;
   out_1155867711308474055[64] = 0;
   out_1155867711308474055[65] = 0;
   out_1155867711308474055[66] = 0;
   out_1155867711308474055[67] = 0;
   out_1155867711308474055[68] = 0;
   out_1155867711308474055[69] = 0;
   out_1155867711308474055[70] = 1;
   out_1155867711308474055[71] = 0;
   out_1155867711308474055[72] = 0;
   out_1155867711308474055[73] = 0;
   out_1155867711308474055[74] = 0;
   out_1155867711308474055[75] = 0;
   out_1155867711308474055[76] = 0;
   out_1155867711308474055[77] = 0;
   out_1155867711308474055[78] = 0;
   out_1155867711308474055[79] = 0;
   out_1155867711308474055[80] = 1;
}
void h_25(double *state, double *unused, double *out_5577598147182567834) {
   out_5577598147182567834[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3271421885184607556) {
   out_3271421885184607556[0] = 0;
   out_3271421885184607556[1] = 0;
   out_3271421885184607556[2] = 0;
   out_3271421885184607556[3] = 0;
   out_3271421885184607556[4] = 0;
   out_3271421885184607556[5] = 0;
   out_3271421885184607556[6] = 1;
   out_3271421885184607556[7] = 0;
   out_3271421885184607556[8] = 0;
}
void h_24(double *state, double *unused, double *out_3902841565486989302) {
   out_3902841565486989302[0] = state[4];
   out_3902841565486989302[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5444071484190107122) {
   out_5444071484190107122[0] = 0;
   out_5444071484190107122[1] = 0;
   out_5444071484190107122[2] = 0;
   out_5444071484190107122[3] = 0;
   out_5444071484190107122[4] = 1;
   out_5444071484190107122[5] = 0;
   out_5444071484190107122[6] = 0;
   out_5444071484190107122[7] = 0;
   out_5444071484190107122[8] = 0;
   out_5444071484190107122[9] = 0;
   out_5444071484190107122[10] = 0;
   out_5444071484190107122[11] = 0;
   out_5444071484190107122[12] = 0;
   out_5444071484190107122[13] = 0;
   out_5444071484190107122[14] = 1;
   out_5444071484190107122[15] = 0;
   out_5444071484190107122[16] = 0;
   out_5444071484190107122[17] = 0;
}
void h_30(double *state, double *unused, double *out_66466683301832023) {
   out_66466683301832023[0] = state[4];
}
void H_30(double *state, double *unused, double *out_753088926677358929) {
   out_753088926677358929[0] = 0;
   out_753088926677358929[1] = 0;
   out_753088926677358929[2] = 0;
   out_753088926677358929[3] = 0;
   out_753088926677358929[4] = 1;
   out_753088926677358929[5] = 0;
   out_753088926677358929[6] = 0;
   out_753088926677358929[7] = 0;
   out_753088926677358929[8] = 0;
}
void h_26(double *state, double *unused, double *out_4358894231996399510) {
   out_4358894231996399510[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7012925204058663780) {
   out_7012925204058663780[0] = 0;
   out_7012925204058663780[1] = 0;
   out_7012925204058663780[2] = 0;
   out_7012925204058663780[3] = 0;
   out_7012925204058663780[4] = 0;
   out_7012925204058663780[5] = 0;
   out_7012925204058663780[6] = 0;
   out_7012925204058663780[7] = 1;
   out_7012925204058663780[8] = 0;
}
void h_27(double *state, double *unused, double *out_2071747026128766508) {
   out_2071747026128766508[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2927852238477783840) {
   out_2927852238477783840[0] = 0;
   out_2927852238477783840[1] = 0;
   out_2927852238477783840[2] = 0;
   out_2927852238477783840[3] = 1;
   out_2927852238477783840[4] = 0;
   out_2927852238477783840[5] = 0;
   out_2927852238477783840[6] = 0;
   out_2927852238477783840[7] = 0;
   out_2927852238477783840[8] = 0;
}
void h_29(double *state, double *unused, double *out_5933430502557268160) {
   out_5933430502557268160[0] = state[1];
}
void H_29(double *state, double *unused, double *out_242857582362966745) {
   out_242857582362966745[0] = 0;
   out_242857582362966745[1] = 1;
   out_242857582362966745[2] = 0;
   out_242857582362966745[3] = 0;
   out_242857582362966745[4] = 0;
   out_242857582362966745[5] = 0;
   out_242857582362966745[6] = 0;
   out_242857582362966745[7] = 0;
   out_242857582362966745[8] = 0;
}
void h_28(double *state, double *unused, double *out_3053831119464064620) {
   out_3053831119464064620[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5325256599432497319) {
   out_5325256599432497319[0] = 1;
   out_5325256599432497319[1] = 0;
   out_5325256599432497319[2] = 0;
   out_5325256599432497319[3] = 0;
   out_5325256599432497319[4] = 0;
   out_5325256599432497319[5] = 0;
   out_5325256599432497319[6] = 0;
   out_5325256599432497319[7] = 0;
   out_5325256599432497319[8] = 0;
}
void h_31(double *state, double *unused, double *out_9215047675436590184) {
   out_9215047675436590184[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7639133306292015256) {
   out_7639133306292015256[0] = 0;
   out_7639133306292015256[1] = 0;
   out_7639133306292015256[2] = 0;
   out_7639133306292015256[3] = 0;
   out_7639133306292015256[4] = 0;
   out_7639133306292015256[5] = 0;
   out_7639133306292015256[6] = 0;
   out_7639133306292015256[7] = 0;
   out_7639133306292015256[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2247958235058554131) {
  err_fun(nom_x, delta_x, out_2247958235058554131);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1704160864652915340) {
  inv_err_fun(nom_x, true_x, out_1704160864652915340);
}
void car_H_mod_fun(double *state, double *out_5921766340727629681) {
  H_mod_fun(state, out_5921766340727629681);
}
void car_f_fun(double *state, double dt, double *out_7705802625381580173) {
  f_fun(state,  dt, out_7705802625381580173);
}
void car_F_fun(double *state, double dt, double *out_1155867711308474055) {
  F_fun(state,  dt, out_1155867711308474055);
}
void car_h_25(double *state, double *unused, double *out_5577598147182567834) {
  h_25(state, unused, out_5577598147182567834);
}
void car_H_25(double *state, double *unused, double *out_3271421885184607556) {
  H_25(state, unused, out_3271421885184607556);
}
void car_h_24(double *state, double *unused, double *out_3902841565486989302) {
  h_24(state, unused, out_3902841565486989302);
}
void car_H_24(double *state, double *unused, double *out_5444071484190107122) {
  H_24(state, unused, out_5444071484190107122);
}
void car_h_30(double *state, double *unused, double *out_66466683301832023) {
  h_30(state, unused, out_66466683301832023);
}
void car_H_30(double *state, double *unused, double *out_753088926677358929) {
  H_30(state, unused, out_753088926677358929);
}
void car_h_26(double *state, double *unused, double *out_4358894231996399510) {
  h_26(state, unused, out_4358894231996399510);
}
void car_H_26(double *state, double *unused, double *out_7012925204058663780) {
  H_26(state, unused, out_7012925204058663780);
}
void car_h_27(double *state, double *unused, double *out_2071747026128766508) {
  h_27(state, unused, out_2071747026128766508);
}
void car_H_27(double *state, double *unused, double *out_2927852238477783840) {
  H_27(state, unused, out_2927852238477783840);
}
void car_h_29(double *state, double *unused, double *out_5933430502557268160) {
  h_29(state, unused, out_5933430502557268160);
}
void car_H_29(double *state, double *unused, double *out_242857582362966745) {
  H_29(state, unused, out_242857582362966745);
}
void car_h_28(double *state, double *unused, double *out_3053831119464064620) {
  h_28(state, unused, out_3053831119464064620);
}
void car_H_28(double *state, double *unused, double *out_5325256599432497319) {
  H_28(state, unused, out_5325256599432497319);
}
void car_h_31(double *state, double *unused, double *out_9215047675436590184) {
  h_31(state, unused, out_9215047675436590184);
}
void car_H_31(double *state, double *unused, double *out_7639133306292015256) {
  H_31(state, unused, out_7639133306292015256);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
