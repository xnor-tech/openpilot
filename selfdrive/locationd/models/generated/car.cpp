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
void err_fun(double *nom_x, double *delta_x, double *out_7469656253560156198) {
   out_7469656253560156198[0] = delta_x[0] + nom_x[0];
   out_7469656253560156198[1] = delta_x[1] + nom_x[1];
   out_7469656253560156198[2] = delta_x[2] + nom_x[2];
   out_7469656253560156198[3] = delta_x[3] + nom_x[3];
   out_7469656253560156198[4] = delta_x[4] + nom_x[4];
   out_7469656253560156198[5] = delta_x[5] + nom_x[5];
   out_7469656253560156198[6] = delta_x[6] + nom_x[6];
   out_7469656253560156198[7] = delta_x[7] + nom_x[7];
   out_7469656253560156198[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6624039366772895003) {
   out_6624039366772895003[0] = -nom_x[0] + true_x[0];
   out_6624039366772895003[1] = -nom_x[1] + true_x[1];
   out_6624039366772895003[2] = -nom_x[2] + true_x[2];
   out_6624039366772895003[3] = -nom_x[3] + true_x[3];
   out_6624039366772895003[4] = -nom_x[4] + true_x[4];
   out_6624039366772895003[5] = -nom_x[5] + true_x[5];
   out_6624039366772895003[6] = -nom_x[6] + true_x[6];
   out_6624039366772895003[7] = -nom_x[7] + true_x[7];
   out_6624039366772895003[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_344573687507074959) {
   out_344573687507074959[0] = 1.0;
   out_344573687507074959[1] = 0.0;
   out_344573687507074959[2] = 0.0;
   out_344573687507074959[3] = 0.0;
   out_344573687507074959[4] = 0.0;
   out_344573687507074959[5] = 0.0;
   out_344573687507074959[6] = 0.0;
   out_344573687507074959[7] = 0.0;
   out_344573687507074959[8] = 0.0;
   out_344573687507074959[9] = 0.0;
   out_344573687507074959[10] = 1.0;
   out_344573687507074959[11] = 0.0;
   out_344573687507074959[12] = 0.0;
   out_344573687507074959[13] = 0.0;
   out_344573687507074959[14] = 0.0;
   out_344573687507074959[15] = 0.0;
   out_344573687507074959[16] = 0.0;
   out_344573687507074959[17] = 0.0;
   out_344573687507074959[18] = 0.0;
   out_344573687507074959[19] = 0.0;
   out_344573687507074959[20] = 1.0;
   out_344573687507074959[21] = 0.0;
   out_344573687507074959[22] = 0.0;
   out_344573687507074959[23] = 0.0;
   out_344573687507074959[24] = 0.0;
   out_344573687507074959[25] = 0.0;
   out_344573687507074959[26] = 0.0;
   out_344573687507074959[27] = 0.0;
   out_344573687507074959[28] = 0.0;
   out_344573687507074959[29] = 0.0;
   out_344573687507074959[30] = 1.0;
   out_344573687507074959[31] = 0.0;
   out_344573687507074959[32] = 0.0;
   out_344573687507074959[33] = 0.0;
   out_344573687507074959[34] = 0.0;
   out_344573687507074959[35] = 0.0;
   out_344573687507074959[36] = 0.0;
   out_344573687507074959[37] = 0.0;
   out_344573687507074959[38] = 0.0;
   out_344573687507074959[39] = 0.0;
   out_344573687507074959[40] = 1.0;
   out_344573687507074959[41] = 0.0;
   out_344573687507074959[42] = 0.0;
   out_344573687507074959[43] = 0.0;
   out_344573687507074959[44] = 0.0;
   out_344573687507074959[45] = 0.0;
   out_344573687507074959[46] = 0.0;
   out_344573687507074959[47] = 0.0;
   out_344573687507074959[48] = 0.0;
   out_344573687507074959[49] = 0.0;
   out_344573687507074959[50] = 1.0;
   out_344573687507074959[51] = 0.0;
   out_344573687507074959[52] = 0.0;
   out_344573687507074959[53] = 0.0;
   out_344573687507074959[54] = 0.0;
   out_344573687507074959[55] = 0.0;
   out_344573687507074959[56] = 0.0;
   out_344573687507074959[57] = 0.0;
   out_344573687507074959[58] = 0.0;
   out_344573687507074959[59] = 0.0;
   out_344573687507074959[60] = 1.0;
   out_344573687507074959[61] = 0.0;
   out_344573687507074959[62] = 0.0;
   out_344573687507074959[63] = 0.0;
   out_344573687507074959[64] = 0.0;
   out_344573687507074959[65] = 0.0;
   out_344573687507074959[66] = 0.0;
   out_344573687507074959[67] = 0.0;
   out_344573687507074959[68] = 0.0;
   out_344573687507074959[69] = 0.0;
   out_344573687507074959[70] = 1.0;
   out_344573687507074959[71] = 0.0;
   out_344573687507074959[72] = 0.0;
   out_344573687507074959[73] = 0.0;
   out_344573687507074959[74] = 0.0;
   out_344573687507074959[75] = 0.0;
   out_344573687507074959[76] = 0.0;
   out_344573687507074959[77] = 0.0;
   out_344573687507074959[78] = 0.0;
   out_344573687507074959[79] = 0.0;
   out_344573687507074959[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2533881819271106710) {
   out_2533881819271106710[0] = state[0];
   out_2533881819271106710[1] = state[1];
   out_2533881819271106710[2] = state[2];
   out_2533881819271106710[3] = state[3];
   out_2533881819271106710[4] = state[4];
   out_2533881819271106710[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2533881819271106710[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2533881819271106710[7] = state[7];
   out_2533881819271106710[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2677838494581999961) {
   out_2677838494581999961[0] = 1;
   out_2677838494581999961[1] = 0;
   out_2677838494581999961[2] = 0;
   out_2677838494581999961[3] = 0;
   out_2677838494581999961[4] = 0;
   out_2677838494581999961[5] = 0;
   out_2677838494581999961[6] = 0;
   out_2677838494581999961[7] = 0;
   out_2677838494581999961[8] = 0;
   out_2677838494581999961[9] = 0;
   out_2677838494581999961[10] = 1;
   out_2677838494581999961[11] = 0;
   out_2677838494581999961[12] = 0;
   out_2677838494581999961[13] = 0;
   out_2677838494581999961[14] = 0;
   out_2677838494581999961[15] = 0;
   out_2677838494581999961[16] = 0;
   out_2677838494581999961[17] = 0;
   out_2677838494581999961[18] = 0;
   out_2677838494581999961[19] = 0;
   out_2677838494581999961[20] = 1;
   out_2677838494581999961[21] = 0;
   out_2677838494581999961[22] = 0;
   out_2677838494581999961[23] = 0;
   out_2677838494581999961[24] = 0;
   out_2677838494581999961[25] = 0;
   out_2677838494581999961[26] = 0;
   out_2677838494581999961[27] = 0;
   out_2677838494581999961[28] = 0;
   out_2677838494581999961[29] = 0;
   out_2677838494581999961[30] = 1;
   out_2677838494581999961[31] = 0;
   out_2677838494581999961[32] = 0;
   out_2677838494581999961[33] = 0;
   out_2677838494581999961[34] = 0;
   out_2677838494581999961[35] = 0;
   out_2677838494581999961[36] = 0;
   out_2677838494581999961[37] = 0;
   out_2677838494581999961[38] = 0;
   out_2677838494581999961[39] = 0;
   out_2677838494581999961[40] = 1;
   out_2677838494581999961[41] = 0;
   out_2677838494581999961[42] = 0;
   out_2677838494581999961[43] = 0;
   out_2677838494581999961[44] = 0;
   out_2677838494581999961[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2677838494581999961[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2677838494581999961[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2677838494581999961[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2677838494581999961[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2677838494581999961[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2677838494581999961[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2677838494581999961[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2677838494581999961[53] = -9.8000000000000007*dt;
   out_2677838494581999961[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2677838494581999961[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2677838494581999961[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2677838494581999961[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2677838494581999961[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2677838494581999961[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2677838494581999961[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2677838494581999961[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2677838494581999961[62] = 0;
   out_2677838494581999961[63] = 0;
   out_2677838494581999961[64] = 0;
   out_2677838494581999961[65] = 0;
   out_2677838494581999961[66] = 0;
   out_2677838494581999961[67] = 0;
   out_2677838494581999961[68] = 0;
   out_2677838494581999961[69] = 0;
   out_2677838494581999961[70] = 1;
   out_2677838494581999961[71] = 0;
   out_2677838494581999961[72] = 0;
   out_2677838494581999961[73] = 0;
   out_2677838494581999961[74] = 0;
   out_2677838494581999961[75] = 0;
   out_2677838494581999961[76] = 0;
   out_2677838494581999961[77] = 0;
   out_2677838494581999961[78] = 0;
   out_2677838494581999961[79] = 0;
   out_2677838494581999961[80] = 1;
}
void h_25(double *state, double *unused, double *out_1278707427595684251) {
   out_1278707427595684251[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4405445685606331007) {
   out_4405445685606331007[0] = 0;
   out_4405445685606331007[1] = 0;
   out_4405445685606331007[2] = 0;
   out_4405445685606331007[3] = 0;
   out_4405445685606331007[4] = 0;
   out_4405445685606331007[5] = 0;
   out_4405445685606331007[6] = 1;
   out_4405445685606331007[7] = 0;
   out_4405445685606331007[8] = 0;
}
void h_24(double *state, double *unused, double *out_7823599996595568809) {
   out_7823599996595568809[0] = state[4];
   out_7823599996595568809[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6622906754426293716) {
   out_6622906754426293716[0] = 0;
   out_6622906754426293716[1] = 0;
   out_6622906754426293716[2] = 0;
   out_6622906754426293716[3] = 0;
   out_6622906754426293716[4] = 1;
   out_6622906754426293716[5] = 0;
   out_6622906754426293716[6] = 0;
   out_6622906754426293716[7] = 0;
   out_6622906754426293716[8] = 0;
   out_6622906754426293716[9] = 0;
   out_6622906754426293716[10] = 0;
   out_6622906754426293716[11] = 0;
   out_6622906754426293716[12] = 0;
   out_6622906754426293716[13] = 0;
   out_6622906754426293716[14] = 1;
   out_6622906754426293716[15] = 0;
   out_6622906754426293716[16] = 0;
   out_6622906754426293716[17] = 0;
}
void h_30(double *state, double *unused, double *out_1121520759244555106) {
   out_1121520759244555106[0] = state[4];
}
void H_30(double *state, double *unused, double *out_122250644521277191) {
   out_122250644521277191[0] = 0;
   out_122250644521277191[1] = 0;
   out_122250644521277191[2] = 0;
   out_122250644521277191[3] = 0;
   out_122250644521277191[4] = 1;
   out_122250644521277191[5] = 0;
   out_122250644521277191[6] = 0;
   out_122250644521277191[7] = 0;
   out_122250644521277191[8] = 0;
}
void h_26(double *state, double *unused, double *out_7520562276658404252) {
   out_7520562276658404252[0] = state[7];
}
void H_26(double *state, double *unused, double *out_663942366732274783) {
   out_663942366732274783[0] = 0;
   out_663942366732274783[1] = 0;
   out_663942366732274783[2] = 0;
   out_663942366732274783[3] = 0;
   out_663942366732274783[4] = 0;
   out_663942366732274783[5] = 0;
   out_663942366732274783[6] = 0;
   out_663942366732274783[7] = 1;
   out_663942366732274783[8] = 0;
}
void h_27(double *state, double *unused, double *out_5468512726416157531) {
   out_5468512726416157531[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2297013956321702102) {
   out_2297013956321702102[0] = 0;
   out_2297013956321702102[1] = 0;
   out_2297013956321702102[2] = 0;
   out_2297013956321702102[3] = 1;
   out_2297013956321702102[4] = 0;
   out_2297013956321702102[5] = 0;
   out_2297013956321702102[6] = 0;
   out_2297013956321702102[7] = 0;
   out_2297013956321702102[8] = 0;
}
void h_29(double *state, double *unused, double *out_5743706788700663420) {
   out_5743706788700663420[0] = state[1];
}
void H_29(double *state, double *unused, double *out_387980699793114993) {
   out_387980699793114993[0] = 0;
   out_387980699793114993[1] = 1;
   out_387980699793114993[2] = 0;
   out_387980699793114993[3] = 0;
   out_387980699793114993[4] = 0;
   out_387980699793114993[5] = 0;
   out_387980699793114993[6] = 0;
   out_387980699793114993[7] = 0;
   out_387980699793114993[8] = 0;
}
void h_28(double *state, double *unused, double *out_7099945859734678997) {
   out_7099945859734678997[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2351610971358441244) {
   out_2351610971358441244[0] = 1;
   out_2351610971358441244[1] = 0;
   out_2351610971358441244[2] = 0;
   out_2351610971358441244[3] = 0;
   out_2351610971358441244[4] = 0;
   out_2351610971358441244[5] = 0;
   out_2351610971358441244[6] = 0;
   out_2351610971358441244[7] = 0;
   out_2351610971358441244[8] = 0;
}
void h_31(double *state, double *unused, double *out_3394844017673189766) {
   out_3394844017673189766[0] = state[8];
}
void H_31(double *state, double *unused, double *out_37734264498923307) {
   out_37734264498923307[0] = 0;
   out_37734264498923307[1] = 0;
   out_37734264498923307[2] = 0;
   out_37734264498923307[3] = 0;
   out_37734264498923307[4] = 0;
   out_37734264498923307[5] = 0;
   out_37734264498923307[6] = 0;
   out_37734264498923307[7] = 0;
   out_37734264498923307[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7469656253560156198) {
  err_fun(nom_x, delta_x, out_7469656253560156198);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6624039366772895003) {
  inv_err_fun(nom_x, true_x, out_6624039366772895003);
}
void car_H_mod_fun(double *state, double *out_344573687507074959) {
  H_mod_fun(state, out_344573687507074959);
}
void car_f_fun(double *state, double dt, double *out_2533881819271106710) {
  f_fun(state,  dt, out_2533881819271106710);
}
void car_F_fun(double *state, double dt, double *out_2677838494581999961) {
  F_fun(state,  dt, out_2677838494581999961);
}
void car_h_25(double *state, double *unused, double *out_1278707427595684251) {
  h_25(state, unused, out_1278707427595684251);
}
void car_H_25(double *state, double *unused, double *out_4405445685606331007) {
  H_25(state, unused, out_4405445685606331007);
}
void car_h_24(double *state, double *unused, double *out_7823599996595568809) {
  h_24(state, unused, out_7823599996595568809);
}
void car_H_24(double *state, double *unused, double *out_6622906754426293716) {
  H_24(state, unused, out_6622906754426293716);
}
void car_h_30(double *state, double *unused, double *out_1121520759244555106) {
  h_30(state, unused, out_1121520759244555106);
}
void car_H_30(double *state, double *unused, double *out_122250644521277191) {
  H_30(state, unused, out_122250644521277191);
}
void car_h_26(double *state, double *unused, double *out_7520562276658404252) {
  h_26(state, unused, out_7520562276658404252);
}
void car_H_26(double *state, double *unused, double *out_663942366732274783) {
  H_26(state, unused, out_663942366732274783);
}
void car_h_27(double *state, double *unused, double *out_5468512726416157531) {
  h_27(state, unused, out_5468512726416157531);
}
void car_H_27(double *state, double *unused, double *out_2297013956321702102) {
  H_27(state, unused, out_2297013956321702102);
}
void car_h_29(double *state, double *unused, double *out_5743706788700663420) {
  h_29(state, unused, out_5743706788700663420);
}
void car_H_29(double *state, double *unused, double *out_387980699793114993) {
  H_29(state, unused, out_387980699793114993);
}
void car_h_28(double *state, double *unused, double *out_7099945859734678997) {
  h_28(state, unused, out_7099945859734678997);
}
void car_H_28(double *state, double *unused, double *out_2351610971358441244) {
  H_28(state, unused, out_2351610971358441244);
}
void car_h_31(double *state, double *unused, double *out_3394844017673189766) {
  h_31(state, unused, out_3394844017673189766);
}
void car_H_31(double *state, double *unused, double *out_37734264498923307) {
  H_31(state, unused, out_37734264498923307);
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
