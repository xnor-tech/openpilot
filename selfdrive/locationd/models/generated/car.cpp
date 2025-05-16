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
void err_fun(double *nom_x, double *delta_x, double *out_4700612312330196268) {
   out_4700612312330196268[0] = delta_x[0] + nom_x[0];
   out_4700612312330196268[1] = delta_x[1] + nom_x[1];
   out_4700612312330196268[2] = delta_x[2] + nom_x[2];
   out_4700612312330196268[3] = delta_x[3] + nom_x[3];
   out_4700612312330196268[4] = delta_x[4] + nom_x[4];
   out_4700612312330196268[5] = delta_x[5] + nom_x[5];
   out_4700612312330196268[6] = delta_x[6] + nom_x[6];
   out_4700612312330196268[7] = delta_x[7] + nom_x[7];
   out_4700612312330196268[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1555638159944357333) {
   out_1555638159944357333[0] = -nom_x[0] + true_x[0];
   out_1555638159944357333[1] = -nom_x[1] + true_x[1];
   out_1555638159944357333[2] = -nom_x[2] + true_x[2];
   out_1555638159944357333[3] = -nom_x[3] + true_x[3];
   out_1555638159944357333[4] = -nom_x[4] + true_x[4];
   out_1555638159944357333[5] = -nom_x[5] + true_x[5];
   out_1555638159944357333[6] = -nom_x[6] + true_x[6];
   out_1555638159944357333[7] = -nom_x[7] + true_x[7];
   out_1555638159944357333[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5825955621798686393) {
   out_5825955621798686393[0] = 1.0;
   out_5825955621798686393[1] = 0.0;
   out_5825955621798686393[2] = 0.0;
   out_5825955621798686393[3] = 0.0;
   out_5825955621798686393[4] = 0.0;
   out_5825955621798686393[5] = 0.0;
   out_5825955621798686393[6] = 0.0;
   out_5825955621798686393[7] = 0.0;
   out_5825955621798686393[8] = 0.0;
   out_5825955621798686393[9] = 0.0;
   out_5825955621798686393[10] = 1.0;
   out_5825955621798686393[11] = 0.0;
   out_5825955621798686393[12] = 0.0;
   out_5825955621798686393[13] = 0.0;
   out_5825955621798686393[14] = 0.0;
   out_5825955621798686393[15] = 0.0;
   out_5825955621798686393[16] = 0.0;
   out_5825955621798686393[17] = 0.0;
   out_5825955621798686393[18] = 0.0;
   out_5825955621798686393[19] = 0.0;
   out_5825955621798686393[20] = 1.0;
   out_5825955621798686393[21] = 0.0;
   out_5825955621798686393[22] = 0.0;
   out_5825955621798686393[23] = 0.0;
   out_5825955621798686393[24] = 0.0;
   out_5825955621798686393[25] = 0.0;
   out_5825955621798686393[26] = 0.0;
   out_5825955621798686393[27] = 0.0;
   out_5825955621798686393[28] = 0.0;
   out_5825955621798686393[29] = 0.0;
   out_5825955621798686393[30] = 1.0;
   out_5825955621798686393[31] = 0.0;
   out_5825955621798686393[32] = 0.0;
   out_5825955621798686393[33] = 0.0;
   out_5825955621798686393[34] = 0.0;
   out_5825955621798686393[35] = 0.0;
   out_5825955621798686393[36] = 0.0;
   out_5825955621798686393[37] = 0.0;
   out_5825955621798686393[38] = 0.0;
   out_5825955621798686393[39] = 0.0;
   out_5825955621798686393[40] = 1.0;
   out_5825955621798686393[41] = 0.0;
   out_5825955621798686393[42] = 0.0;
   out_5825955621798686393[43] = 0.0;
   out_5825955621798686393[44] = 0.0;
   out_5825955621798686393[45] = 0.0;
   out_5825955621798686393[46] = 0.0;
   out_5825955621798686393[47] = 0.0;
   out_5825955621798686393[48] = 0.0;
   out_5825955621798686393[49] = 0.0;
   out_5825955621798686393[50] = 1.0;
   out_5825955621798686393[51] = 0.0;
   out_5825955621798686393[52] = 0.0;
   out_5825955621798686393[53] = 0.0;
   out_5825955621798686393[54] = 0.0;
   out_5825955621798686393[55] = 0.0;
   out_5825955621798686393[56] = 0.0;
   out_5825955621798686393[57] = 0.0;
   out_5825955621798686393[58] = 0.0;
   out_5825955621798686393[59] = 0.0;
   out_5825955621798686393[60] = 1.0;
   out_5825955621798686393[61] = 0.0;
   out_5825955621798686393[62] = 0.0;
   out_5825955621798686393[63] = 0.0;
   out_5825955621798686393[64] = 0.0;
   out_5825955621798686393[65] = 0.0;
   out_5825955621798686393[66] = 0.0;
   out_5825955621798686393[67] = 0.0;
   out_5825955621798686393[68] = 0.0;
   out_5825955621798686393[69] = 0.0;
   out_5825955621798686393[70] = 1.0;
   out_5825955621798686393[71] = 0.0;
   out_5825955621798686393[72] = 0.0;
   out_5825955621798686393[73] = 0.0;
   out_5825955621798686393[74] = 0.0;
   out_5825955621798686393[75] = 0.0;
   out_5825955621798686393[76] = 0.0;
   out_5825955621798686393[77] = 0.0;
   out_5825955621798686393[78] = 0.0;
   out_5825955621798686393[79] = 0.0;
   out_5825955621798686393[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2916287295467583771) {
   out_2916287295467583771[0] = state[0];
   out_2916287295467583771[1] = state[1];
   out_2916287295467583771[2] = state[2];
   out_2916287295467583771[3] = state[3];
   out_2916287295467583771[4] = state[4];
   out_2916287295467583771[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2916287295467583771[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2916287295467583771[7] = state[7];
   out_2916287295467583771[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6878950951447676039) {
   out_6878950951447676039[0] = 1;
   out_6878950951447676039[1] = 0;
   out_6878950951447676039[2] = 0;
   out_6878950951447676039[3] = 0;
   out_6878950951447676039[4] = 0;
   out_6878950951447676039[5] = 0;
   out_6878950951447676039[6] = 0;
   out_6878950951447676039[7] = 0;
   out_6878950951447676039[8] = 0;
   out_6878950951447676039[9] = 0;
   out_6878950951447676039[10] = 1;
   out_6878950951447676039[11] = 0;
   out_6878950951447676039[12] = 0;
   out_6878950951447676039[13] = 0;
   out_6878950951447676039[14] = 0;
   out_6878950951447676039[15] = 0;
   out_6878950951447676039[16] = 0;
   out_6878950951447676039[17] = 0;
   out_6878950951447676039[18] = 0;
   out_6878950951447676039[19] = 0;
   out_6878950951447676039[20] = 1;
   out_6878950951447676039[21] = 0;
   out_6878950951447676039[22] = 0;
   out_6878950951447676039[23] = 0;
   out_6878950951447676039[24] = 0;
   out_6878950951447676039[25] = 0;
   out_6878950951447676039[26] = 0;
   out_6878950951447676039[27] = 0;
   out_6878950951447676039[28] = 0;
   out_6878950951447676039[29] = 0;
   out_6878950951447676039[30] = 1;
   out_6878950951447676039[31] = 0;
   out_6878950951447676039[32] = 0;
   out_6878950951447676039[33] = 0;
   out_6878950951447676039[34] = 0;
   out_6878950951447676039[35] = 0;
   out_6878950951447676039[36] = 0;
   out_6878950951447676039[37] = 0;
   out_6878950951447676039[38] = 0;
   out_6878950951447676039[39] = 0;
   out_6878950951447676039[40] = 1;
   out_6878950951447676039[41] = 0;
   out_6878950951447676039[42] = 0;
   out_6878950951447676039[43] = 0;
   out_6878950951447676039[44] = 0;
   out_6878950951447676039[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6878950951447676039[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6878950951447676039[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6878950951447676039[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6878950951447676039[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6878950951447676039[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6878950951447676039[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6878950951447676039[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6878950951447676039[53] = -9.8000000000000007*dt;
   out_6878950951447676039[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6878950951447676039[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6878950951447676039[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6878950951447676039[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6878950951447676039[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6878950951447676039[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6878950951447676039[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6878950951447676039[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6878950951447676039[62] = 0;
   out_6878950951447676039[63] = 0;
   out_6878950951447676039[64] = 0;
   out_6878950951447676039[65] = 0;
   out_6878950951447676039[66] = 0;
   out_6878950951447676039[67] = 0;
   out_6878950951447676039[68] = 0;
   out_6878950951447676039[69] = 0;
   out_6878950951447676039[70] = 1;
   out_6878950951447676039[71] = 0;
   out_6878950951447676039[72] = 0;
   out_6878950951447676039[73] = 0;
   out_6878950951447676039[74] = 0;
   out_6878950951447676039[75] = 0;
   out_6878950951447676039[76] = 0;
   out_6878950951447676039[77] = 0;
   out_6878950951447676039[78] = 0;
   out_6878950951447676039[79] = 0;
   out_6878950951447676039[80] = 1;
}
void h_25(double *state, double *unused, double *out_4274570641008688092) {
   out_4274570641008688092[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7717096626726200383) {
   out_7717096626726200383[0] = 0;
   out_7717096626726200383[1] = 0;
   out_7717096626726200383[2] = 0;
   out_7717096626726200383[3] = 0;
   out_7717096626726200383[4] = 0;
   out_7717096626726200383[5] = 0;
   out_7717096626726200383[6] = 1;
   out_7717096626726200383[7] = 0;
   out_7717096626726200383[8] = 0;
}
void h_24(double *state, double *unused, double *out_2799824959515052510) {
   out_2799824959515052510[0] = state[4];
   out_2799824959515052510[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5860832581955644381) {
   out_5860832581955644381[0] = 0;
   out_5860832581955644381[1] = 0;
   out_5860832581955644381[2] = 0;
   out_5860832581955644381[3] = 0;
   out_5860832581955644381[4] = 1;
   out_5860832581955644381[5] = 0;
   out_5860832581955644381[6] = 0;
   out_5860832581955644381[7] = 0;
   out_5860832581955644381[8] = 0;
   out_5860832581955644381[9] = 0;
   out_5860832581955644381[10] = 0;
   out_5860832581955644381[11] = 0;
   out_5860832581955644381[12] = 0;
   out_5860832581955644381[13] = 0;
   out_5860832581955644381[14] = 1;
   out_5860832581955644381[15] = 0;
   out_5860832581955644381[16] = 0;
   out_5860832581955644381[17] = 0;
}
void h_30(double *state, double *unused, double *out_4431757309359817237) {
   out_4431757309359817237[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6201951116855743035) {
   out_6201951116855743035[0] = 0;
   out_6201951116855743035[1] = 0;
   out_6201951116855743035[2] = 0;
   out_6201951116855743035[3] = 0;
   out_6201951116855743035[4] = 1;
   out_6201951116855743035[5] = 0;
   out_6201951116855743035[6] = 0;
   out_6201951116855743035[7] = 0;
   out_6201951116855743035[8] = 0;
}
void h_26(double *state, double *unused, double *out_1394971257915484552) {
   out_1394971257915484552[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6988144128109295009) {
   out_6988144128109295009[0] = 0;
   out_6988144128109295009[1] = 0;
   out_6988144128109295009[2] = 0;
   out_6988144128109295009[3] = 0;
   out_6988144128109295009[4] = 0;
   out_6988144128109295009[5] = 0;
   out_6988144128109295009[6] = 0;
   out_6988144128109295009[7] = 1;
   out_6988144128109295009[8] = 0;
}
void h_27(double *state, double *unused, double *out_3374774532302646250) {
   out_3374774532302646250[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4027187805055318124) {
   out_4027187805055318124[0] = 0;
   out_4027187805055318124[1] = 0;
   out_4027187805055318124[2] = 0;
   out_4027187805055318124[3] = 1;
   out_4027187805055318124[4] = 0;
   out_4027187805055318124[5] = 0;
   out_4027187805055318124[6] = 0;
   out_4027187805055318124[7] = 0;
   out_4027187805055318124[8] = 0;
}
void h_29(double *state, double *unused, double *out_7012224060556668600) {
   out_7012224060556668600[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6712182461170135219) {
   out_6712182461170135219[0] = 0;
   out_6712182461170135219[1] = 1;
   out_6712182461170135219[2] = 0;
   out_6712182461170135219[3] = 0;
   out_6712182461170135219[4] = 0;
   out_6712182461170135219[5] = 0;
   out_6712182461170135219[6] = 0;
   out_6712182461170135219[7] = 0;
   out_6712182461170135219[8] = 0;
}
void h_28(double *state, double *unused, double *out_5793520145370500276) {
   out_5793520145370500276[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8675812732735461470) {
   out_8675812732735461470[0] = 1;
   out_8675812732735461470[1] = 0;
   out_8675812732735461470[2] = 0;
   out_8675812732735461470[3] = 0;
   out_8675812732735461470[4] = 0;
   out_8675812732735461470[5] = 0;
   out_8675812732735461470[6] = 0;
   out_8675812732735461470[7] = 0;
   out_8675812732735461470[8] = 0;
}
void h_31(double *state, double *unused, double *out_8214726661834494558) {
   out_8214726661834494558[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6361936025875943533) {
   out_6361936025875943533[0] = 0;
   out_6361936025875943533[1] = 0;
   out_6361936025875943533[2] = 0;
   out_6361936025875943533[3] = 0;
   out_6361936025875943533[4] = 0;
   out_6361936025875943533[5] = 0;
   out_6361936025875943533[6] = 0;
   out_6361936025875943533[7] = 0;
   out_6361936025875943533[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4700612312330196268) {
  err_fun(nom_x, delta_x, out_4700612312330196268);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1555638159944357333) {
  inv_err_fun(nom_x, true_x, out_1555638159944357333);
}
void car_H_mod_fun(double *state, double *out_5825955621798686393) {
  H_mod_fun(state, out_5825955621798686393);
}
void car_f_fun(double *state, double dt, double *out_2916287295467583771) {
  f_fun(state,  dt, out_2916287295467583771);
}
void car_F_fun(double *state, double dt, double *out_6878950951447676039) {
  F_fun(state,  dt, out_6878950951447676039);
}
void car_h_25(double *state, double *unused, double *out_4274570641008688092) {
  h_25(state, unused, out_4274570641008688092);
}
void car_H_25(double *state, double *unused, double *out_7717096626726200383) {
  H_25(state, unused, out_7717096626726200383);
}
void car_h_24(double *state, double *unused, double *out_2799824959515052510) {
  h_24(state, unused, out_2799824959515052510);
}
void car_H_24(double *state, double *unused, double *out_5860832581955644381) {
  H_24(state, unused, out_5860832581955644381);
}
void car_h_30(double *state, double *unused, double *out_4431757309359817237) {
  h_30(state, unused, out_4431757309359817237);
}
void car_H_30(double *state, double *unused, double *out_6201951116855743035) {
  H_30(state, unused, out_6201951116855743035);
}
void car_h_26(double *state, double *unused, double *out_1394971257915484552) {
  h_26(state, unused, out_1394971257915484552);
}
void car_H_26(double *state, double *unused, double *out_6988144128109295009) {
  H_26(state, unused, out_6988144128109295009);
}
void car_h_27(double *state, double *unused, double *out_3374774532302646250) {
  h_27(state, unused, out_3374774532302646250);
}
void car_H_27(double *state, double *unused, double *out_4027187805055318124) {
  H_27(state, unused, out_4027187805055318124);
}
void car_h_29(double *state, double *unused, double *out_7012224060556668600) {
  h_29(state, unused, out_7012224060556668600);
}
void car_H_29(double *state, double *unused, double *out_6712182461170135219) {
  H_29(state, unused, out_6712182461170135219);
}
void car_h_28(double *state, double *unused, double *out_5793520145370500276) {
  h_28(state, unused, out_5793520145370500276);
}
void car_H_28(double *state, double *unused, double *out_8675812732735461470) {
  H_28(state, unused, out_8675812732735461470);
}
void car_h_31(double *state, double *unused, double *out_8214726661834494558) {
  h_31(state, unused, out_8214726661834494558);
}
void car_H_31(double *state, double *unused, double *out_6361936025875943533) {
  H_31(state, unused, out_6361936025875943533);
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
