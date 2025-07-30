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
void err_fun(double *nom_x, double *delta_x, double *out_2325115720415564338) {
   out_2325115720415564338[0] = delta_x[0] + nom_x[0];
   out_2325115720415564338[1] = delta_x[1] + nom_x[1];
   out_2325115720415564338[2] = delta_x[2] + nom_x[2];
   out_2325115720415564338[3] = delta_x[3] + nom_x[3];
   out_2325115720415564338[4] = delta_x[4] + nom_x[4];
   out_2325115720415564338[5] = delta_x[5] + nom_x[5];
   out_2325115720415564338[6] = delta_x[6] + nom_x[6];
   out_2325115720415564338[7] = delta_x[7] + nom_x[7];
   out_2325115720415564338[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4599327397528124256) {
   out_4599327397528124256[0] = -nom_x[0] + true_x[0];
   out_4599327397528124256[1] = -nom_x[1] + true_x[1];
   out_4599327397528124256[2] = -nom_x[2] + true_x[2];
   out_4599327397528124256[3] = -nom_x[3] + true_x[3];
   out_4599327397528124256[4] = -nom_x[4] + true_x[4];
   out_4599327397528124256[5] = -nom_x[5] + true_x[5];
   out_4599327397528124256[6] = -nom_x[6] + true_x[6];
   out_4599327397528124256[7] = -nom_x[7] + true_x[7];
   out_4599327397528124256[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3603362443890924865) {
   out_3603362443890924865[0] = 1.0;
   out_3603362443890924865[1] = 0.0;
   out_3603362443890924865[2] = 0.0;
   out_3603362443890924865[3] = 0.0;
   out_3603362443890924865[4] = 0.0;
   out_3603362443890924865[5] = 0.0;
   out_3603362443890924865[6] = 0.0;
   out_3603362443890924865[7] = 0.0;
   out_3603362443890924865[8] = 0.0;
   out_3603362443890924865[9] = 0.0;
   out_3603362443890924865[10] = 1.0;
   out_3603362443890924865[11] = 0.0;
   out_3603362443890924865[12] = 0.0;
   out_3603362443890924865[13] = 0.0;
   out_3603362443890924865[14] = 0.0;
   out_3603362443890924865[15] = 0.0;
   out_3603362443890924865[16] = 0.0;
   out_3603362443890924865[17] = 0.0;
   out_3603362443890924865[18] = 0.0;
   out_3603362443890924865[19] = 0.0;
   out_3603362443890924865[20] = 1.0;
   out_3603362443890924865[21] = 0.0;
   out_3603362443890924865[22] = 0.0;
   out_3603362443890924865[23] = 0.0;
   out_3603362443890924865[24] = 0.0;
   out_3603362443890924865[25] = 0.0;
   out_3603362443890924865[26] = 0.0;
   out_3603362443890924865[27] = 0.0;
   out_3603362443890924865[28] = 0.0;
   out_3603362443890924865[29] = 0.0;
   out_3603362443890924865[30] = 1.0;
   out_3603362443890924865[31] = 0.0;
   out_3603362443890924865[32] = 0.0;
   out_3603362443890924865[33] = 0.0;
   out_3603362443890924865[34] = 0.0;
   out_3603362443890924865[35] = 0.0;
   out_3603362443890924865[36] = 0.0;
   out_3603362443890924865[37] = 0.0;
   out_3603362443890924865[38] = 0.0;
   out_3603362443890924865[39] = 0.0;
   out_3603362443890924865[40] = 1.0;
   out_3603362443890924865[41] = 0.0;
   out_3603362443890924865[42] = 0.0;
   out_3603362443890924865[43] = 0.0;
   out_3603362443890924865[44] = 0.0;
   out_3603362443890924865[45] = 0.0;
   out_3603362443890924865[46] = 0.0;
   out_3603362443890924865[47] = 0.0;
   out_3603362443890924865[48] = 0.0;
   out_3603362443890924865[49] = 0.0;
   out_3603362443890924865[50] = 1.0;
   out_3603362443890924865[51] = 0.0;
   out_3603362443890924865[52] = 0.0;
   out_3603362443890924865[53] = 0.0;
   out_3603362443890924865[54] = 0.0;
   out_3603362443890924865[55] = 0.0;
   out_3603362443890924865[56] = 0.0;
   out_3603362443890924865[57] = 0.0;
   out_3603362443890924865[58] = 0.0;
   out_3603362443890924865[59] = 0.0;
   out_3603362443890924865[60] = 1.0;
   out_3603362443890924865[61] = 0.0;
   out_3603362443890924865[62] = 0.0;
   out_3603362443890924865[63] = 0.0;
   out_3603362443890924865[64] = 0.0;
   out_3603362443890924865[65] = 0.0;
   out_3603362443890924865[66] = 0.0;
   out_3603362443890924865[67] = 0.0;
   out_3603362443890924865[68] = 0.0;
   out_3603362443890924865[69] = 0.0;
   out_3603362443890924865[70] = 1.0;
   out_3603362443890924865[71] = 0.0;
   out_3603362443890924865[72] = 0.0;
   out_3603362443890924865[73] = 0.0;
   out_3603362443890924865[74] = 0.0;
   out_3603362443890924865[75] = 0.0;
   out_3603362443890924865[76] = 0.0;
   out_3603362443890924865[77] = 0.0;
   out_3603362443890924865[78] = 0.0;
   out_3603362443890924865[79] = 0.0;
   out_3603362443890924865[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3060737343829631213) {
   out_3060737343829631213[0] = state[0];
   out_3060737343829631213[1] = state[1];
   out_3060737343829631213[2] = state[2];
   out_3060737343829631213[3] = state[3];
   out_3060737343829631213[4] = state[4];
   out_3060737343829631213[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3060737343829631213[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3060737343829631213[7] = state[7];
   out_3060737343829631213[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1388818207155273058) {
   out_1388818207155273058[0] = 1;
   out_1388818207155273058[1] = 0;
   out_1388818207155273058[2] = 0;
   out_1388818207155273058[3] = 0;
   out_1388818207155273058[4] = 0;
   out_1388818207155273058[5] = 0;
   out_1388818207155273058[6] = 0;
   out_1388818207155273058[7] = 0;
   out_1388818207155273058[8] = 0;
   out_1388818207155273058[9] = 0;
   out_1388818207155273058[10] = 1;
   out_1388818207155273058[11] = 0;
   out_1388818207155273058[12] = 0;
   out_1388818207155273058[13] = 0;
   out_1388818207155273058[14] = 0;
   out_1388818207155273058[15] = 0;
   out_1388818207155273058[16] = 0;
   out_1388818207155273058[17] = 0;
   out_1388818207155273058[18] = 0;
   out_1388818207155273058[19] = 0;
   out_1388818207155273058[20] = 1;
   out_1388818207155273058[21] = 0;
   out_1388818207155273058[22] = 0;
   out_1388818207155273058[23] = 0;
   out_1388818207155273058[24] = 0;
   out_1388818207155273058[25] = 0;
   out_1388818207155273058[26] = 0;
   out_1388818207155273058[27] = 0;
   out_1388818207155273058[28] = 0;
   out_1388818207155273058[29] = 0;
   out_1388818207155273058[30] = 1;
   out_1388818207155273058[31] = 0;
   out_1388818207155273058[32] = 0;
   out_1388818207155273058[33] = 0;
   out_1388818207155273058[34] = 0;
   out_1388818207155273058[35] = 0;
   out_1388818207155273058[36] = 0;
   out_1388818207155273058[37] = 0;
   out_1388818207155273058[38] = 0;
   out_1388818207155273058[39] = 0;
   out_1388818207155273058[40] = 1;
   out_1388818207155273058[41] = 0;
   out_1388818207155273058[42] = 0;
   out_1388818207155273058[43] = 0;
   out_1388818207155273058[44] = 0;
   out_1388818207155273058[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1388818207155273058[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1388818207155273058[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1388818207155273058[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1388818207155273058[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1388818207155273058[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1388818207155273058[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1388818207155273058[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1388818207155273058[53] = -9.8000000000000007*dt;
   out_1388818207155273058[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1388818207155273058[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1388818207155273058[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1388818207155273058[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1388818207155273058[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1388818207155273058[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1388818207155273058[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1388818207155273058[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1388818207155273058[62] = 0;
   out_1388818207155273058[63] = 0;
   out_1388818207155273058[64] = 0;
   out_1388818207155273058[65] = 0;
   out_1388818207155273058[66] = 0;
   out_1388818207155273058[67] = 0;
   out_1388818207155273058[68] = 0;
   out_1388818207155273058[69] = 0;
   out_1388818207155273058[70] = 1;
   out_1388818207155273058[71] = 0;
   out_1388818207155273058[72] = 0;
   out_1388818207155273058[73] = 0;
   out_1388818207155273058[74] = 0;
   out_1388818207155273058[75] = 0;
   out_1388818207155273058[76] = 0;
   out_1388818207155273058[77] = 0;
   out_1388818207155273058[78] = 0;
   out_1388818207155273058[79] = 0;
   out_1388818207155273058[80] = 1;
}
void h_25(double *state, double *unused, double *out_5000231162990336464) {
   out_5000231162990336464[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7126343093880416542) {
   out_7126343093880416542[0] = 0;
   out_7126343093880416542[1] = 0;
   out_7126343093880416542[2] = 0;
   out_7126343093880416542[3] = 0;
   out_7126343093880416542[4] = 0;
   out_7126343093880416542[5] = 0;
   out_7126343093880416542[6] = 1;
   out_7126343093880416542[7] = 0;
   out_7126343093880416542[8] = 0;
}
void h_24(double *state, double *unused, double *out_3452317047508392249) {
   out_3452317047508392249[0] = state[4];
   out_3452317047508392249[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2379833766913907508) {
   out_2379833766913907508[0] = 0;
   out_2379833766913907508[1] = 0;
   out_2379833766913907508[2] = 0;
   out_2379833766913907508[3] = 0;
   out_2379833766913907508[4] = 1;
   out_2379833766913907508[5] = 0;
   out_2379833766913907508[6] = 0;
   out_2379833766913907508[7] = 0;
   out_2379833766913907508[8] = 0;
   out_2379833766913907508[9] = 0;
   out_2379833766913907508[10] = 0;
   out_2379833766913907508[11] = 0;
   out_2379833766913907508[12] = 0;
   out_2379833766913907508[13] = 0;
   out_2379833766913907508[14] = 1;
   out_2379833766913907508[15] = 0;
   out_2379833766913907508[16] = 0;
   out_2379833766913907508[17] = 0;
}
void h_30(double *state, double *unused, double *out_1770604063360014472) {
   out_1770604063360014472[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8802068021321886447) {
   out_8802068021321886447[0] = 0;
   out_8802068021321886447[1] = 0;
   out_8802068021321886447[2] = 0;
   out_8802068021321886447[3] = 0;
   out_8802068021321886447[4] = 1;
   out_8802068021321886447[5] = 0;
   out_8802068021321886447[6] = 0;
   out_8802068021321886447[7] = 0;
   out_8802068021321886447[8] = 0;
}
void h_26(double *state, double *unused, double *out_827094210458352037) {
   out_827094210458352037[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3384839775006360318) {
   out_3384839775006360318[0] = 0;
   out_3384839775006360318[1] = 0;
   out_3384839775006360318[2] = 0;
   out_3384839775006360318[3] = 0;
   out_3384839775006360318[4] = 0;
   out_3384839775006360318[5] = 0;
   out_3384839775006360318[6] = 0;
   out_3384839775006360318[7] = 1;
   out_3384839775006360318[8] = 0;
}
void h_27(double *state, double *unused, double *out_8444839643058369507) {
   out_8444839643058369507[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6578473950137943230) {
   out_6578473950137943230[0] = 0;
   out_6578473950137943230[1] = 0;
   out_6578473950137943230[2] = 0;
   out_6578473950137943230[3] = 1;
   out_6578473950137943230[4] = 0;
   out_6578473950137943230[5] = 0;
   out_6578473950137943230[6] = 0;
   out_6578473950137943230[7] = 0;
   out_6578473950137943230[8] = 0;
}
void h_29(double *state, double *unused, double *out_3231069204300831173) {
   out_3231069204300831173[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8291836677007494263) {
   out_8291836677007494263[0] = 0;
   out_8291836677007494263[1] = 1;
   out_8291836677007494263[2] = 0;
   out_8291836677007494263[3] = 0;
   out_8291836677007494263[4] = 0;
   out_8291836677007494263[5] = 0;
   out_8291836677007494263[6] = 0;
   out_8291836677007494263[7] = 0;
   out_8291836677007494263[8] = 0;
}
void h_28(double *state, double *unused, double *out_8311456595915501149) {
   out_8311456595915501149[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5072508379632526779) {
   out_5072508379632526779[0] = 1;
   out_5072508379632526779[1] = 0;
   out_5072508379632526779[2] = 0;
   out_5072508379632526779[3] = 0;
   out_5072508379632526779[4] = 0;
   out_5072508379632526779[5] = 0;
   out_5072508379632526779[6] = 0;
   out_5072508379632526779[7] = 0;
   out_5072508379632526779[8] = 0;
}
void h_31(double *state, double *unused, double *out_2052505172634851503) {
   out_2052505172634851503[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7156989055757376970) {
   out_7156989055757376970[0] = 0;
   out_7156989055757376970[1] = 0;
   out_7156989055757376970[2] = 0;
   out_7156989055757376970[3] = 0;
   out_7156989055757376970[4] = 0;
   out_7156989055757376970[5] = 0;
   out_7156989055757376970[6] = 0;
   out_7156989055757376970[7] = 0;
   out_7156989055757376970[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2325115720415564338) {
  err_fun(nom_x, delta_x, out_2325115720415564338);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4599327397528124256) {
  inv_err_fun(nom_x, true_x, out_4599327397528124256);
}
void car_H_mod_fun(double *state, double *out_3603362443890924865) {
  H_mod_fun(state, out_3603362443890924865);
}
void car_f_fun(double *state, double dt, double *out_3060737343829631213) {
  f_fun(state,  dt, out_3060737343829631213);
}
void car_F_fun(double *state, double dt, double *out_1388818207155273058) {
  F_fun(state,  dt, out_1388818207155273058);
}
void car_h_25(double *state, double *unused, double *out_5000231162990336464) {
  h_25(state, unused, out_5000231162990336464);
}
void car_H_25(double *state, double *unused, double *out_7126343093880416542) {
  H_25(state, unused, out_7126343093880416542);
}
void car_h_24(double *state, double *unused, double *out_3452317047508392249) {
  h_24(state, unused, out_3452317047508392249);
}
void car_H_24(double *state, double *unused, double *out_2379833766913907508) {
  H_24(state, unused, out_2379833766913907508);
}
void car_h_30(double *state, double *unused, double *out_1770604063360014472) {
  h_30(state, unused, out_1770604063360014472);
}
void car_H_30(double *state, double *unused, double *out_8802068021321886447) {
  H_30(state, unused, out_8802068021321886447);
}
void car_h_26(double *state, double *unused, double *out_827094210458352037) {
  h_26(state, unused, out_827094210458352037);
}
void car_H_26(double *state, double *unused, double *out_3384839775006360318) {
  H_26(state, unused, out_3384839775006360318);
}
void car_h_27(double *state, double *unused, double *out_8444839643058369507) {
  h_27(state, unused, out_8444839643058369507);
}
void car_H_27(double *state, double *unused, double *out_6578473950137943230) {
  H_27(state, unused, out_6578473950137943230);
}
void car_h_29(double *state, double *unused, double *out_3231069204300831173) {
  h_29(state, unused, out_3231069204300831173);
}
void car_H_29(double *state, double *unused, double *out_8291836677007494263) {
  H_29(state, unused, out_8291836677007494263);
}
void car_h_28(double *state, double *unused, double *out_8311456595915501149) {
  h_28(state, unused, out_8311456595915501149);
}
void car_H_28(double *state, double *unused, double *out_5072508379632526779) {
  H_28(state, unused, out_5072508379632526779);
}
void car_h_31(double *state, double *unused, double *out_2052505172634851503) {
  h_31(state, unused, out_2052505172634851503);
}
void car_H_31(double *state, double *unused, double *out_7156989055757376970) {
  H_31(state, unused, out_7156989055757376970);
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
