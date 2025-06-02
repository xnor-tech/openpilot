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
void err_fun(double *nom_x, double *delta_x, double *out_7098692201925828218) {
   out_7098692201925828218[0] = delta_x[0] + nom_x[0];
   out_7098692201925828218[1] = delta_x[1] + nom_x[1];
   out_7098692201925828218[2] = delta_x[2] + nom_x[2];
   out_7098692201925828218[3] = delta_x[3] + nom_x[3];
   out_7098692201925828218[4] = delta_x[4] + nom_x[4];
   out_7098692201925828218[5] = delta_x[5] + nom_x[5];
   out_7098692201925828218[6] = delta_x[6] + nom_x[6];
   out_7098692201925828218[7] = delta_x[7] + nom_x[7];
   out_7098692201925828218[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8974564382236626677) {
   out_8974564382236626677[0] = -nom_x[0] + true_x[0];
   out_8974564382236626677[1] = -nom_x[1] + true_x[1];
   out_8974564382236626677[2] = -nom_x[2] + true_x[2];
   out_8974564382236626677[3] = -nom_x[3] + true_x[3];
   out_8974564382236626677[4] = -nom_x[4] + true_x[4];
   out_8974564382236626677[5] = -nom_x[5] + true_x[5];
   out_8974564382236626677[6] = -nom_x[6] + true_x[6];
   out_8974564382236626677[7] = -nom_x[7] + true_x[7];
   out_8974564382236626677[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4637657594296017115) {
   out_4637657594296017115[0] = 1.0;
   out_4637657594296017115[1] = 0.0;
   out_4637657594296017115[2] = 0.0;
   out_4637657594296017115[3] = 0.0;
   out_4637657594296017115[4] = 0.0;
   out_4637657594296017115[5] = 0.0;
   out_4637657594296017115[6] = 0.0;
   out_4637657594296017115[7] = 0.0;
   out_4637657594296017115[8] = 0.0;
   out_4637657594296017115[9] = 0.0;
   out_4637657594296017115[10] = 1.0;
   out_4637657594296017115[11] = 0.0;
   out_4637657594296017115[12] = 0.0;
   out_4637657594296017115[13] = 0.0;
   out_4637657594296017115[14] = 0.0;
   out_4637657594296017115[15] = 0.0;
   out_4637657594296017115[16] = 0.0;
   out_4637657594296017115[17] = 0.0;
   out_4637657594296017115[18] = 0.0;
   out_4637657594296017115[19] = 0.0;
   out_4637657594296017115[20] = 1.0;
   out_4637657594296017115[21] = 0.0;
   out_4637657594296017115[22] = 0.0;
   out_4637657594296017115[23] = 0.0;
   out_4637657594296017115[24] = 0.0;
   out_4637657594296017115[25] = 0.0;
   out_4637657594296017115[26] = 0.0;
   out_4637657594296017115[27] = 0.0;
   out_4637657594296017115[28] = 0.0;
   out_4637657594296017115[29] = 0.0;
   out_4637657594296017115[30] = 1.0;
   out_4637657594296017115[31] = 0.0;
   out_4637657594296017115[32] = 0.0;
   out_4637657594296017115[33] = 0.0;
   out_4637657594296017115[34] = 0.0;
   out_4637657594296017115[35] = 0.0;
   out_4637657594296017115[36] = 0.0;
   out_4637657594296017115[37] = 0.0;
   out_4637657594296017115[38] = 0.0;
   out_4637657594296017115[39] = 0.0;
   out_4637657594296017115[40] = 1.0;
   out_4637657594296017115[41] = 0.0;
   out_4637657594296017115[42] = 0.0;
   out_4637657594296017115[43] = 0.0;
   out_4637657594296017115[44] = 0.0;
   out_4637657594296017115[45] = 0.0;
   out_4637657594296017115[46] = 0.0;
   out_4637657594296017115[47] = 0.0;
   out_4637657594296017115[48] = 0.0;
   out_4637657594296017115[49] = 0.0;
   out_4637657594296017115[50] = 1.0;
   out_4637657594296017115[51] = 0.0;
   out_4637657594296017115[52] = 0.0;
   out_4637657594296017115[53] = 0.0;
   out_4637657594296017115[54] = 0.0;
   out_4637657594296017115[55] = 0.0;
   out_4637657594296017115[56] = 0.0;
   out_4637657594296017115[57] = 0.0;
   out_4637657594296017115[58] = 0.0;
   out_4637657594296017115[59] = 0.0;
   out_4637657594296017115[60] = 1.0;
   out_4637657594296017115[61] = 0.0;
   out_4637657594296017115[62] = 0.0;
   out_4637657594296017115[63] = 0.0;
   out_4637657594296017115[64] = 0.0;
   out_4637657594296017115[65] = 0.0;
   out_4637657594296017115[66] = 0.0;
   out_4637657594296017115[67] = 0.0;
   out_4637657594296017115[68] = 0.0;
   out_4637657594296017115[69] = 0.0;
   out_4637657594296017115[70] = 1.0;
   out_4637657594296017115[71] = 0.0;
   out_4637657594296017115[72] = 0.0;
   out_4637657594296017115[73] = 0.0;
   out_4637657594296017115[74] = 0.0;
   out_4637657594296017115[75] = 0.0;
   out_4637657594296017115[76] = 0.0;
   out_4637657594296017115[77] = 0.0;
   out_4637657594296017115[78] = 0.0;
   out_4637657594296017115[79] = 0.0;
   out_4637657594296017115[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5625388813302586479) {
   out_5625388813302586479[0] = state[0];
   out_5625388813302586479[1] = state[1];
   out_5625388813302586479[2] = state[2];
   out_5625388813302586479[3] = state[3];
   out_5625388813302586479[4] = state[4];
   out_5625388813302586479[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5625388813302586479[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5625388813302586479[7] = state[7];
   out_5625388813302586479[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3506019992486908746) {
   out_3506019992486908746[0] = 1;
   out_3506019992486908746[1] = 0;
   out_3506019992486908746[2] = 0;
   out_3506019992486908746[3] = 0;
   out_3506019992486908746[4] = 0;
   out_3506019992486908746[5] = 0;
   out_3506019992486908746[6] = 0;
   out_3506019992486908746[7] = 0;
   out_3506019992486908746[8] = 0;
   out_3506019992486908746[9] = 0;
   out_3506019992486908746[10] = 1;
   out_3506019992486908746[11] = 0;
   out_3506019992486908746[12] = 0;
   out_3506019992486908746[13] = 0;
   out_3506019992486908746[14] = 0;
   out_3506019992486908746[15] = 0;
   out_3506019992486908746[16] = 0;
   out_3506019992486908746[17] = 0;
   out_3506019992486908746[18] = 0;
   out_3506019992486908746[19] = 0;
   out_3506019992486908746[20] = 1;
   out_3506019992486908746[21] = 0;
   out_3506019992486908746[22] = 0;
   out_3506019992486908746[23] = 0;
   out_3506019992486908746[24] = 0;
   out_3506019992486908746[25] = 0;
   out_3506019992486908746[26] = 0;
   out_3506019992486908746[27] = 0;
   out_3506019992486908746[28] = 0;
   out_3506019992486908746[29] = 0;
   out_3506019992486908746[30] = 1;
   out_3506019992486908746[31] = 0;
   out_3506019992486908746[32] = 0;
   out_3506019992486908746[33] = 0;
   out_3506019992486908746[34] = 0;
   out_3506019992486908746[35] = 0;
   out_3506019992486908746[36] = 0;
   out_3506019992486908746[37] = 0;
   out_3506019992486908746[38] = 0;
   out_3506019992486908746[39] = 0;
   out_3506019992486908746[40] = 1;
   out_3506019992486908746[41] = 0;
   out_3506019992486908746[42] = 0;
   out_3506019992486908746[43] = 0;
   out_3506019992486908746[44] = 0;
   out_3506019992486908746[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3506019992486908746[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3506019992486908746[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3506019992486908746[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3506019992486908746[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3506019992486908746[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3506019992486908746[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3506019992486908746[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3506019992486908746[53] = -9.8000000000000007*dt;
   out_3506019992486908746[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3506019992486908746[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3506019992486908746[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3506019992486908746[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3506019992486908746[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3506019992486908746[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3506019992486908746[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3506019992486908746[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3506019992486908746[62] = 0;
   out_3506019992486908746[63] = 0;
   out_3506019992486908746[64] = 0;
   out_3506019992486908746[65] = 0;
   out_3506019992486908746[66] = 0;
   out_3506019992486908746[67] = 0;
   out_3506019992486908746[68] = 0;
   out_3506019992486908746[69] = 0;
   out_3506019992486908746[70] = 1;
   out_3506019992486908746[71] = 0;
   out_3506019992486908746[72] = 0;
   out_3506019992486908746[73] = 0;
   out_3506019992486908746[74] = 0;
   out_3506019992486908746[75] = 0;
   out_3506019992486908746[76] = 0;
   out_3506019992486908746[77] = 0;
   out_3506019992486908746[78] = 0;
   out_3506019992486908746[79] = 0;
   out_3506019992486908746[80] = 1;
}
void h_25(double *state, double *unused, double *out_1659760556682213941) {
   out_1659760556682213941[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2386459148589029399) {
   out_2386459148589029399[0] = 0;
   out_2386459148589029399[1] = 0;
   out_2386459148589029399[2] = 0;
   out_2386459148589029399[3] = 0;
   out_2386459148589029399[4] = 0;
   out_2386459148589029399[5] = 0;
   out_2386459148589029399[6] = 1;
   out_2386459148589029399[7] = 0;
   out_2386459148589029399[8] = 0;
}
void h_24(double *state, double *unused, double *out_8231109947655627744) {
   out_8231109947655627744[0] = state[4];
   out_8231109947655627744[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6841606037480165826) {
   out_6841606037480165826[0] = 0;
   out_6841606037480165826[1] = 0;
   out_6841606037480165826[2] = 0;
   out_6841606037480165826[3] = 0;
   out_6841606037480165826[4] = 1;
   out_6841606037480165826[5] = 0;
   out_6841606037480165826[6] = 0;
   out_6841606037480165826[7] = 0;
   out_6841606037480165826[8] = 0;
   out_6841606037480165826[9] = 0;
   out_6841606037480165826[10] = 0;
   out_6841606037480165826[11] = 0;
   out_6841606037480165826[12] = 0;
   out_6841606037480165826[13] = 0;
   out_6841606037480165826[14] = 1;
   out_6841606037480165826[15] = 0;
   out_6841606037480165826[16] = 0;
   out_6841606037480165826[17] = 0;
}
void h_30(double *state, double *unused, double *out_4852978595674001882) {
   out_4852978595674001882[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6914155478716637597) {
   out_6914155478716637597[0] = 0;
   out_6914155478716637597[1] = 0;
   out_6914155478716637597[2] = 0;
   out_6914155478716637597[3] = 0;
   out_6914155478716637597[4] = 1;
   out_6914155478716637597[5] = 0;
   out_6914155478716637597[6] = 0;
   out_6914155478716637597[7] = 0;
   out_6914155478716637597[8] = 0;
}
void h_26(double *state, double *unused, double *out_4167564816766474560) {
   out_4167564816766474560[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6127962467463085623) {
   out_6127962467463085623[0] = 0;
   out_6127962467463085623[1] = 0;
   out_6127962467463085623[2] = 0;
   out_6127962467463085623[3] = 0;
   out_6127962467463085623[4] = 0;
   out_6127962467463085623[5] = 0;
   out_6127962467463085623[6] = 0;
   out_6127962467463085623[7] = 1;
   out_6127962467463085623[8] = 0;
}
void h_27(double *state, double *unused, double *out_6057712637963842953) {
   out_6057712637963842953[0] = state[3];
}
void H_27(double *state, double *unused, double *out_9088918790517062508) {
   out_9088918790517062508[0] = 0;
   out_9088918790517062508[1] = 0;
   out_9088918790517062508[2] = 0;
   out_9088918790517062508[3] = 1;
   out_9088918790517062508[4] = 0;
   out_9088918790517062508[5] = 0;
   out_9088918790517062508[6] = 0;
   out_9088918790517062508[7] = 0;
   out_9088918790517062508[8] = 0;
}
void h_29(double *state, double *unused, double *out_5900525969612713808) {
   out_5900525969612713808[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6403924134402245413) {
   out_6403924134402245413[0] = 0;
   out_6403924134402245413[1] = 1;
   out_6403924134402245413[2] = 0;
   out_6403924134402245413[3] = 0;
   out_6403924134402245413[4] = 0;
   out_6403924134402245413[5] = 0;
   out_6403924134402245413[6] = 0;
   out_6403924134402245413[7] = 0;
   out_6403924134402245413[8] = 0;
}
void h_28(double *state, double *unused, double *out_3067473329497799704) {
   out_3067473329497799704[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6960420922237775629) {
   out_6960420922237775629[0] = 1;
   out_6960420922237775629[1] = 0;
   out_6960420922237775629[2] = 0;
   out_6960420922237775629[3] = 0;
   out_6960420922237775629[4] = 0;
   out_6960420922237775629[5] = 0;
   out_6960420922237775629[6] = 0;
   out_6960420922237775629[7] = 0;
   out_6960420922237775629[8] = 0;
}
void h_31(double *state, double *unused, double *out_2506670762931880475) {
   out_2506670762931880475[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6754170569696437099) {
   out_6754170569696437099[0] = 0;
   out_6754170569696437099[1] = 0;
   out_6754170569696437099[2] = 0;
   out_6754170569696437099[3] = 0;
   out_6754170569696437099[4] = 0;
   out_6754170569696437099[5] = 0;
   out_6754170569696437099[6] = 0;
   out_6754170569696437099[7] = 0;
   out_6754170569696437099[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7098692201925828218) {
  err_fun(nom_x, delta_x, out_7098692201925828218);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8974564382236626677) {
  inv_err_fun(nom_x, true_x, out_8974564382236626677);
}
void car_H_mod_fun(double *state, double *out_4637657594296017115) {
  H_mod_fun(state, out_4637657594296017115);
}
void car_f_fun(double *state, double dt, double *out_5625388813302586479) {
  f_fun(state,  dt, out_5625388813302586479);
}
void car_F_fun(double *state, double dt, double *out_3506019992486908746) {
  F_fun(state,  dt, out_3506019992486908746);
}
void car_h_25(double *state, double *unused, double *out_1659760556682213941) {
  h_25(state, unused, out_1659760556682213941);
}
void car_H_25(double *state, double *unused, double *out_2386459148589029399) {
  H_25(state, unused, out_2386459148589029399);
}
void car_h_24(double *state, double *unused, double *out_8231109947655627744) {
  h_24(state, unused, out_8231109947655627744);
}
void car_H_24(double *state, double *unused, double *out_6841606037480165826) {
  H_24(state, unused, out_6841606037480165826);
}
void car_h_30(double *state, double *unused, double *out_4852978595674001882) {
  h_30(state, unused, out_4852978595674001882);
}
void car_H_30(double *state, double *unused, double *out_6914155478716637597) {
  H_30(state, unused, out_6914155478716637597);
}
void car_h_26(double *state, double *unused, double *out_4167564816766474560) {
  h_26(state, unused, out_4167564816766474560);
}
void car_H_26(double *state, double *unused, double *out_6127962467463085623) {
  H_26(state, unused, out_6127962467463085623);
}
void car_h_27(double *state, double *unused, double *out_6057712637963842953) {
  h_27(state, unused, out_6057712637963842953);
}
void car_H_27(double *state, double *unused, double *out_9088918790517062508) {
  H_27(state, unused, out_9088918790517062508);
}
void car_h_29(double *state, double *unused, double *out_5900525969612713808) {
  h_29(state, unused, out_5900525969612713808);
}
void car_H_29(double *state, double *unused, double *out_6403924134402245413) {
  H_29(state, unused, out_6403924134402245413);
}
void car_h_28(double *state, double *unused, double *out_3067473329497799704) {
  h_28(state, unused, out_3067473329497799704);
}
void car_H_28(double *state, double *unused, double *out_6960420922237775629) {
  H_28(state, unused, out_6960420922237775629);
}
void car_h_31(double *state, double *unused, double *out_2506670762931880475) {
  h_31(state, unused, out_2506670762931880475);
}
void car_H_31(double *state, double *unused, double *out_6754170569696437099) {
  H_31(state, unused, out_6754170569696437099);
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
