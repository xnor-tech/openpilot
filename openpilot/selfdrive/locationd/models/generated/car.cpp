#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

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
void err_fun(double *nom_x, double *delta_x, double *out_860827029368682551) {
   out_860827029368682551[0] = delta_x[0] + nom_x[0];
   out_860827029368682551[1] = delta_x[1] + nom_x[1];
   out_860827029368682551[2] = delta_x[2] + nom_x[2];
   out_860827029368682551[3] = delta_x[3] + nom_x[3];
   out_860827029368682551[4] = delta_x[4] + nom_x[4];
   out_860827029368682551[5] = delta_x[5] + nom_x[5];
   out_860827029368682551[6] = delta_x[6] + nom_x[6];
   out_860827029368682551[7] = delta_x[7] + nom_x[7];
   out_860827029368682551[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8858530749727553508) {
   out_8858530749727553508[0] = -nom_x[0] + true_x[0];
   out_8858530749727553508[1] = -nom_x[1] + true_x[1];
   out_8858530749727553508[2] = -nom_x[2] + true_x[2];
   out_8858530749727553508[3] = -nom_x[3] + true_x[3];
   out_8858530749727553508[4] = -nom_x[4] + true_x[4];
   out_8858530749727553508[5] = -nom_x[5] + true_x[5];
   out_8858530749727553508[6] = -nom_x[6] + true_x[6];
   out_8858530749727553508[7] = -nom_x[7] + true_x[7];
   out_8858530749727553508[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3714675639127096598) {
   out_3714675639127096598[0] = 1.0;
   out_3714675639127096598[1] = 0.0;
   out_3714675639127096598[2] = 0.0;
   out_3714675639127096598[3] = 0.0;
   out_3714675639127096598[4] = 0.0;
   out_3714675639127096598[5] = 0.0;
   out_3714675639127096598[6] = 0.0;
   out_3714675639127096598[7] = 0.0;
   out_3714675639127096598[8] = 0.0;
   out_3714675639127096598[9] = 0.0;
   out_3714675639127096598[10] = 1.0;
   out_3714675639127096598[11] = 0.0;
   out_3714675639127096598[12] = 0.0;
   out_3714675639127096598[13] = 0.0;
   out_3714675639127096598[14] = 0.0;
   out_3714675639127096598[15] = 0.0;
   out_3714675639127096598[16] = 0.0;
   out_3714675639127096598[17] = 0.0;
   out_3714675639127096598[18] = 0.0;
   out_3714675639127096598[19] = 0.0;
   out_3714675639127096598[20] = 1.0;
   out_3714675639127096598[21] = 0.0;
   out_3714675639127096598[22] = 0.0;
   out_3714675639127096598[23] = 0.0;
   out_3714675639127096598[24] = 0.0;
   out_3714675639127096598[25] = 0.0;
   out_3714675639127096598[26] = 0.0;
   out_3714675639127096598[27] = 0.0;
   out_3714675639127096598[28] = 0.0;
   out_3714675639127096598[29] = 0.0;
   out_3714675639127096598[30] = 1.0;
   out_3714675639127096598[31] = 0.0;
   out_3714675639127096598[32] = 0.0;
   out_3714675639127096598[33] = 0.0;
   out_3714675639127096598[34] = 0.0;
   out_3714675639127096598[35] = 0.0;
   out_3714675639127096598[36] = 0.0;
   out_3714675639127096598[37] = 0.0;
   out_3714675639127096598[38] = 0.0;
   out_3714675639127096598[39] = 0.0;
   out_3714675639127096598[40] = 1.0;
   out_3714675639127096598[41] = 0.0;
   out_3714675639127096598[42] = 0.0;
   out_3714675639127096598[43] = 0.0;
   out_3714675639127096598[44] = 0.0;
   out_3714675639127096598[45] = 0.0;
   out_3714675639127096598[46] = 0.0;
   out_3714675639127096598[47] = 0.0;
   out_3714675639127096598[48] = 0.0;
   out_3714675639127096598[49] = 0.0;
   out_3714675639127096598[50] = 1.0;
   out_3714675639127096598[51] = 0.0;
   out_3714675639127096598[52] = 0.0;
   out_3714675639127096598[53] = 0.0;
   out_3714675639127096598[54] = 0.0;
   out_3714675639127096598[55] = 0.0;
   out_3714675639127096598[56] = 0.0;
   out_3714675639127096598[57] = 0.0;
   out_3714675639127096598[58] = 0.0;
   out_3714675639127096598[59] = 0.0;
   out_3714675639127096598[60] = 1.0;
   out_3714675639127096598[61] = 0.0;
   out_3714675639127096598[62] = 0.0;
   out_3714675639127096598[63] = 0.0;
   out_3714675639127096598[64] = 0.0;
   out_3714675639127096598[65] = 0.0;
   out_3714675639127096598[66] = 0.0;
   out_3714675639127096598[67] = 0.0;
   out_3714675639127096598[68] = 0.0;
   out_3714675639127096598[69] = 0.0;
   out_3714675639127096598[70] = 1.0;
   out_3714675639127096598[71] = 0.0;
   out_3714675639127096598[72] = 0.0;
   out_3714675639127096598[73] = 0.0;
   out_3714675639127096598[74] = 0.0;
   out_3714675639127096598[75] = 0.0;
   out_3714675639127096598[76] = 0.0;
   out_3714675639127096598[77] = 0.0;
   out_3714675639127096598[78] = 0.0;
   out_3714675639127096598[79] = 0.0;
   out_3714675639127096598[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4730484258686567608) {
   out_4730484258686567608[0] = state[0];
   out_4730484258686567608[1] = state[1];
   out_4730484258686567608[2] = state[2];
   out_4730484258686567608[3] = state[3];
   out_4730484258686567608[4] = state[4];
   out_4730484258686567608[5] = dt*(stiffness_front*(-state[2] - state[3] + state[7])*state[0]/state[1] + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/state[4] + ((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4])*state[6] - 9.8100000000000005*state[8]) + state[5];
   out_4730484258686567608[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4730484258686567608[7] = state[7];
   out_4730484258686567608[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8913108992009854130) {
   out_8913108992009854130[0] = 1;
   out_8913108992009854130[1] = 0;
   out_8913108992009854130[2] = 0;
   out_8913108992009854130[3] = 0;
   out_8913108992009854130[4] = 0;
   out_8913108992009854130[5] = 0;
   out_8913108992009854130[6] = 0;
   out_8913108992009854130[7] = 0;
   out_8913108992009854130[8] = 0;
   out_8913108992009854130[9] = 0;
   out_8913108992009854130[10] = 1;
   out_8913108992009854130[11] = 0;
   out_8913108992009854130[12] = 0;
   out_8913108992009854130[13] = 0;
   out_8913108992009854130[14] = 0;
   out_8913108992009854130[15] = 0;
   out_8913108992009854130[16] = 0;
   out_8913108992009854130[17] = 0;
   out_8913108992009854130[18] = 0;
   out_8913108992009854130[19] = 0;
   out_8913108992009854130[20] = 1;
   out_8913108992009854130[21] = 0;
   out_8913108992009854130[22] = 0;
   out_8913108992009854130[23] = 0;
   out_8913108992009854130[24] = 0;
   out_8913108992009854130[25] = 0;
   out_8913108992009854130[26] = 0;
   out_8913108992009854130[27] = 0;
   out_8913108992009854130[28] = 0;
   out_8913108992009854130[29] = 0;
   out_8913108992009854130[30] = 1;
   out_8913108992009854130[31] = 0;
   out_8913108992009854130[32] = 0;
   out_8913108992009854130[33] = 0;
   out_8913108992009854130[34] = 0;
   out_8913108992009854130[35] = 0;
   out_8913108992009854130[36] = 0;
   out_8913108992009854130[37] = 0;
   out_8913108992009854130[38] = 0;
   out_8913108992009854130[39] = 0;
   out_8913108992009854130[40] = 1;
   out_8913108992009854130[41] = 0;
   out_8913108992009854130[42] = 0;
   out_8913108992009854130[43] = 0;
   out_8913108992009854130[44] = 0;
   out_8913108992009854130[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/state[1] + (-stiffness_front - stiffness_rear)*state[5]/state[4] + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/state[4]);
   out_8913108992009854130[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/pow(state[1], 2);
   out_8913108992009854130[47] = -dt*stiffness_front*state[0]/state[1];
   out_8913108992009854130[48] = -dt*stiffness_front*state[0]/state[1];
   out_8913108992009854130[49] = dt*(-(-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/pow(state[4], 2) + (-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/pow(state[4], 2) - 1)*state[6]);
   out_8913108992009854130[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/state[4] + 1;
   out_8913108992009854130[51] = dt*((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4]);
   out_8913108992009854130[52] = dt*stiffness_front*state[0]/state[1];
   out_8913108992009854130[53] = -9.8100000000000005*dt;
   out_8913108992009854130[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8913108992009854130[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8913108992009854130[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8913108992009854130[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8913108992009854130[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8913108992009854130[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8913108992009854130[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8913108992009854130[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8913108992009854130[62] = 0;
   out_8913108992009854130[63] = 0;
   out_8913108992009854130[64] = 0;
   out_8913108992009854130[65] = 0;
   out_8913108992009854130[66] = 0;
   out_8913108992009854130[67] = 0;
   out_8913108992009854130[68] = 0;
   out_8913108992009854130[69] = 0;
   out_8913108992009854130[70] = 1;
   out_8913108992009854130[71] = 0;
   out_8913108992009854130[72] = 0;
   out_8913108992009854130[73] = 0;
   out_8913108992009854130[74] = 0;
   out_8913108992009854130[75] = 0;
   out_8913108992009854130[76] = 0;
   out_8913108992009854130[77] = 0;
   out_8913108992009854130[78] = 0;
   out_8913108992009854130[79] = 0;
   out_8913108992009854130[80] = 1;
}
void h_25(double *state, double *unused, double *out_1867293235320502834) {
   out_1867293235320502834[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6919591383865390944) {
   out_6919591383865390944[0] = 0;
   out_6919591383865390944[1] = 0;
   out_6919591383865390944[2] = 0;
   out_6919591383865390944[3] = 0;
   out_6919591383865390944[4] = 0;
   out_6919591383865390944[5] = 0;
   out_6919591383865390944[6] = 1;
   out_6919591383865390944[7] = 0;
   out_6919591383865390944[8] = 0;
}
void h_24(double *state, double *unused, double *out_3213342894231868787) {
   out_3213342894231868787[0] = state[4];
   out_3213342894231868787[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3366230609664718215) {
   out_3366230609664718215[0] = 0;
   out_3366230609664718215[1] = 0;
   out_3366230609664718215[2] = 0;
   out_3366230609664718215[3] = 0;
   out_3366230609664718215[4] = 1;
   out_3366230609664718215[5] = 0;
   out_3366230609664718215[6] = 0;
   out_3366230609664718215[7] = 0;
   out_3366230609664718215[8] = 0;
   out_3366230609664718215[9] = 0;
   out_3366230609664718215[10] = 0;
   out_3366230609664718215[11] = 0;
   out_3366230609664718215[12] = 0;
   out_3366230609664718215[13] = 0;
   out_3366230609664718215[14] = 1;
   out_3366230609664718215[15] = 0;
   out_3366230609664718215[16] = 0;
   out_3366230609664718215[17] = 0;
}
void h_30(double *state, double *unused, double *out_4577317895700990437) {
   out_4577317895700990437[0] = state[4];
}
void H_30(double *state, double *unused, double *out_9008819731336912045) {
   out_9008819731336912045[0] = 0;
   out_9008819731336912045[1] = 0;
   out_9008819731336912045[2] = 0;
   out_9008819731336912045[3] = 0;
   out_9008819731336912045[4] = 1;
   out_9008819731336912045[5] = 0;
   out_9008819731336912045[6] = 0;
   out_9008819731336912045[7] = 0;
   out_9008819731336912045[8] = 0;
}
void h_26(double *state, double *unused, double *out_5664044544057454764) {
   out_5664044544057454764[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3178088064991334720) {
   out_3178088064991334720[0] = 0;
   out_3178088064991334720[1] = 0;
   out_3178088064991334720[2] = 0;
   out_3178088064991334720[3] = 0;
   out_3178088064991334720[4] = 0;
   out_3178088064991334720[5] = 0;
   out_3178088064991334720[6] = 0;
   out_3178088064991334720[7] = 1;
   out_3178088064991334720[8] = 0;
}
void h_27(double *state, double *unused, double *out_8266334752734351980) {
   out_8266334752734351980[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7263161030572214660) {
   out_7263161030572214660[0] = 0;
   out_7263161030572214660[1] = 0;
   out_7263161030572214660[2] = 0;
   out_7263161030572214660[3] = 1;
   out_7263161030572214660[4] = 0;
   out_7263161030572214660[5] = 0;
   out_7263161030572214660[6] = 0;
   out_7263161030572214660[7] = 0;
   out_7263161030572214660[8] = 0;
}
void h_29(double *state, double *unused, double *out_8724536114892913642) {
   out_8724536114892913642[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8498588387022519861) {
   out_8498588387022519861[0] = 0;
   out_8498588387022519861[1] = 1;
   out_8498588387022519861[2] = 0;
   out_8498588387022519861[3] = 0;
   out_8498588387022519861[4] = 0;
   out_8498588387022519861[5] = 0;
   out_8498588387022519861[6] = 0;
   out_8498588387022519861[7] = 0;
   out_8498588387022519861[8] = 0;
}
void h_28(double *state, double *unused, double *out_7663018868057874463) {
   out_7663018868057874463[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4865756669617501181) {
   out_4865756669617501181[0] = 1;
   out_4865756669617501181[1] = 0;
   out_4865756669617501181[2] = 0;
   out_4865756669617501181[3] = 0;
   out_4865756669617501181[4] = 0;
   out_4865756669617501181[5] = 0;
   out_4865756669617501181[6] = 0;
   out_4865756669617501181[7] = 0;
   out_4865756669617501181[8] = 0;
}
void h_31(double *state, double *unused, double *out_5504742763574525184) {
   out_5504742763574525184[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2551879962757983244) {
   out_2551879962757983244[0] = 0;
   out_2551879962757983244[1] = 0;
   out_2551879962757983244[2] = 0;
   out_2551879962757983244[3] = 0;
   out_2551879962757983244[4] = 0;
   out_2551879962757983244[5] = 0;
   out_2551879962757983244[6] = 0;
   out_2551879962757983244[7] = 0;
   out_2551879962757983244[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_860827029368682551) {
  err_fun(nom_x, delta_x, out_860827029368682551);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8858530749727553508) {
  inv_err_fun(nom_x, true_x, out_8858530749727553508);
}
void car_H_mod_fun(double *state, double *out_3714675639127096598) {
  H_mod_fun(state, out_3714675639127096598);
}
void car_f_fun(double *state, double dt, double *out_4730484258686567608) {
  f_fun(state,  dt, out_4730484258686567608);
}
void car_F_fun(double *state, double dt, double *out_8913108992009854130) {
  F_fun(state,  dt, out_8913108992009854130);
}
void car_h_25(double *state, double *unused, double *out_1867293235320502834) {
  h_25(state, unused, out_1867293235320502834);
}
void car_H_25(double *state, double *unused, double *out_6919591383865390944) {
  H_25(state, unused, out_6919591383865390944);
}
void car_h_24(double *state, double *unused, double *out_3213342894231868787) {
  h_24(state, unused, out_3213342894231868787);
}
void car_H_24(double *state, double *unused, double *out_3366230609664718215) {
  H_24(state, unused, out_3366230609664718215);
}
void car_h_30(double *state, double *unused, double *out_4577317895700990437) {
  h_30(state, unused, out_4577317895700990437);
}
void car_H_30(double *state, double *unused, double *out_9008819731336912045) {
  H_30(state, unused, out_9008819731336912045);
}
void car_h_26(double *state, double *unused, double *out_5664044544057454764) {
  h_26(state, unused, out_5664044544057454764);
}
void car_H_26(double *state, double *unused, double *out_3178088064991334720) {
  H_26(state, unused, out_3178088064991334720);
}
void car_h_27(double *state, double *unused, double *out_8266334752734351980) {
  h_27(state, unused, out_8266334752734351980);
}
void car_H_27(double *state, double *unused, double *out_7263161030572214660) {
  H_27(state, unused, out_7263161030572214660);
}
void car_h_29(double *state, double *unused, double *out_8724536114892913642) {
  h_29(state, unused, out_8724536114892913642);
}
void car_H_29(double *state, double *unused, double *out_8498588387022519861) {
  H_29(state, unused, out_8498588387022519861);
}
void car_h_28(double *state, double *unused, double *out_7663018868057874463) {
  h_28(state, unused, out_7663018868057874463);
}
void car_H_28(double *state, double *unused, double *out_4865756669617501181) {
  H_28(state, unused, out_4865756669617501181);
}
void car_h_31(double *state, double *unused, double *out_5504742763574525184) {
  h_31(state, unused, out_5504742763574525184);
}
void car_H_31(double *state, double *unused, double *out_2551879962757983244) {
  H_31(state, unused, out_2551879962757983244);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
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
