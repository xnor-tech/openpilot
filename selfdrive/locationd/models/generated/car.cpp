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
void err_fun(double *nom_x, double *delta_x, double *out_8162787069879679621) {
   out_8162787069879679621[0] = delta_x[0] + nom_x[0];
   out_8162787069879679621[1] = delta_x[1] + nom_x[1];
   out_8162787069879679621[2] = delta_x[2] + nom_x[2];
   out_8162787069879679621[3] = delta_x[3] + nom_x[3];
   out_8162787069879679621[4] = delta_x[4] + nom_x[4];
   out_8162787069879679621[5] = delta_x[5] + nom_x[5];
   out_8162787069879679621[6] = delta_x[6] + nom_x[6];
   out_8162787069879679621[7] = delta_x[7] + nom_x[7];
   out_8162787069879679621[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4646430404185096575) {
   out_4646430404185096575[0] = -nom_x[0] + true_x[0];
   out_4646430404185096575[1] = -nom_x[1] + true_x[1];
   out_4646430404185096575[2] = -nom_x[2] + true_x[2];
   out_4646430404185096575[3] = -nom_x[3] + true_x[3];
   out_4646430404185096575[4] = -nom_x[4] + true_x[4];
   out_4646430404185096575[5] = -nom_x[5] + true_x[5];
   out_4646430404185096575[6] = -nom_x[6] + true_x[6];
   out_4646430404185096575[7] = -nom_x[7] + true_x[7];
   out_4646430404185096575[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8634146753423524100) {
   out_8634146753423524100[0] = 1.0;
   out_8634146753423524100[1] = 0.0;
   out_8634146753423524100[2] = 0.0;
   out_8634146753423524100[3] = 0.0;
   out_8634146753423524100[4] = 0.0;
   out_8634146753423524100[5] = 0.0;
   out_8634146753423524100[6] = 0.0;
   out_8634146753423524100[7] = 0.0;
   out_8634146753423524100[8] = 0.0;
   out_8634146753423524100[9] = 0.0;
   out_8634146753423524100[10] = 1.0;
   out_8634146753423524100[11] = 0.0;
   out_8634146753423524100[12] = 0.0;
   out_8634146753423524100[13] = 0.0;
   out_8634146753423524100[14] = 0.0;
   out_8634146753423524100[15] = 0.0;
   out_8634146753423524100[16] = 0.0;
   out_8634146753423524100[17] = 0.0;
   out_8634146753423524100[18] = 0.0;
   out_8634146753423524100[19] = 0.0;
   out_8634146753423524100[20] = 1.0;
   out_8634146753423524100[21] = 0.0;
   out_8634146753423524100[22] = 0.0;
   out_8634146753423524100[23] = 0.0;
   out_8634146753423524100[24] = 0.0;
   out_8634146753423524100[25] = 0.0;
   out_8634146753423524100[26] = 0.0;
   out_8634146753423524100[27] = 0.0;
   out_8634146753423524100[28] = 0.0;
   out_8634146753423524100[29] = 0.0;
   out_8634146753423524100[30] = 1.0;
   out_8634146753423524100[31] = 0.0;
   out_8634146753423524100[32] = 0.0;
   out_8634146753423524100[33] = 0.0;
   out_8634146753423524100[34] = 0.0;
   out_8634146753423524100[35] = 0.0;
   out_8634146753423524100[36] = 0.0;
   out_8634146753423524100[37] = 0.0;
   out_8634146753423524100[38] = 0.0;
   out_8634146753423524100[39] = 0.0;
   out_8634146753423524100[40] = 1.0;
   out_8634146753423524100[41] = 0.0;
   out_8634146753423524100[42] = 0.0;
   out_8634146753423524100[43] = 0.0;
   out_8634146753423524100[44] = 0.0;
   out_8634146753423524100[45] = 0.0;
   out_8634146753423524100[46] = 0.0;
   out_8634146753423524100[47] = 0.0;
   out_8634146753423524100[48] = 0.0;
   out_8634146753423524100[49] = 0.0;
   out_8634146753423524100[50] = 1.0;
   out_8634146753423524100[51] = 0.0;
   out_8634146753423524100[52] = 0.0;
   out_8634146753423524100[53] = 0.0;
   out_8634146753423524100[54] = 0.0;
   out_8634146753423524100[55] = 0.0;
   out_8634146753423524100[56] = 0.0;
   out_8634146753423524100[57] = 0.0;
   out_8634146753423524100[58] = 0.0;
   out_8634146753423524100[59] = 0.0;
   out_8634146753423524100[60] = 1.0;
   out_8634146753423524100[61] = 0.0;
   out_8634146753423524100[62] = 0.0;
   out_8634146753423524100[63] = 0.0;
   out_8634146753423524100[64] = 0.0;
   out_8634146753423524100[65] = 0.0;
   out_8634146753423524100[66] = 0.0;
   out_8634146753423524100[67] = 0.0;
   out_8634146753423524100[68] = 0.0;
   out_8634146753423524100[69] = 0.0;
   out_8634146753423524100[70] = 1.0;
   out_8634146753423524100[71] = 0.0;
   out_8634146753423524100[72] = 0.0;
   out_8634146753423524100[73] = 0.0;
   out_8634146753423524100[74] = 0.0;
   out_8634146753423524100[75] = 0.0;
   out_8634146753423524100[76] = 0.0;
   out_8634146753423524100[77] = 0.0;
   out_8634146753423524100[78] = 0.0;
   out_8634146753423524100[79] = 0.0;
   out_8634146753423524100[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6186994550747992320) {
   out_6186994550747992320[0] = state[0];
   out_6186994550747992320[1] = state[1];
   out_6186994550747992320[2] = state[2];
   out_6186994550747992320[3] = state[3];
   out_6186994550747992320[4] = state[4];
   out_6186994550747992320[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6186994550747992320[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6186994550747992320[7] = state[7];
   out_6186994550747992320[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1846682865561113223) {
   out_1846682865561113223[0] = 1;
   out_1846682865561113223[1] = 0;
   out_1846682865561113223[2] = 0;
   out_1846682865561113223[3] = 0;
   out_1846682865561113223[4] = 0;
   out_1846682865561113223[5] = 0;
   out_1846682865561113223[6] = 0;
   out_1846682865561113223[7] = 0;
   out_1846682865561113223[8] = 0;
   out_1846682865561113223[9] = 0;
   out_1846682865561113223[10] = 1;
   out_1846682865561113223[11] = 0;
   out_1846682865561113223[12] = 0;
   out_1846682865561113223[13] = 0;
   out_1846682865561113223[14] = 0;
   out_1846682865561113223[15] = 0;
   out_1846682865561113223[16] = 0;
   out_1846682865561113223[17] = 0;
   out_1846682865561113223[18] = 0;
   out_1846682865561113223[19] = 0;
   out_1846682865561113223[20] = 1;
   out_1846682865561113223[21] = 0;
   out_1846682865561113223[22] = 0;
   out_1846682865561113223[23] = 0;
   out_1846682865561113223[24] = 0;
   out_1846682865561113223[25] = 0;
   out_1846682865561113223[26] = 0;
   out_1846682865561113223[27] = 0;
   out_1846682865561113223[28] = 0;
   out_1846682865561113223[29] = 0;
   out_1846682865561113223[30] = 1;
   out_1846682865561113223[31] = 0;
   out_1846682865561113223[32] = 0;
   out_1846682865561113223[33] = 0;
   out_1846682865561113223[34] = 0;
   out_1846682865561113223[35] = 0;
   out_1846682865561113223[36] = 0;
   out_1846682865561113223[37] = 0;
   out_1846682865561113223[38] = 0;
   out_1846682865561113223[39] = 0;
   out_1846682865561113223[40] = 1;
   out_1846682865561113223[41] = 0;
   out_1846682865561113223[42] = 0;
   out_1846682865561113223[43] = 0;
   out_1846682865561113223[44] = 0;
   out_1846682865561113223[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1846682865561113223[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1846682865561113223[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1846682865561113223[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1846682865561113223[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1846682865561113223[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1846682865561113223[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1846682865561113223[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1846682865561113223[53] = -9.8000000000000007*dt;
   out_1846682865561113223[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1846682865561113223[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1846682865561113223[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1846682865561113223[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1846682865561113223[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1846682865561113223[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1846682865561113223[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1846682865561113223[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1846682865561113223[62] = 0;
   out_1846682865561113223[63] = 0;
   out_1846682865561113223[64] = 0;
   out_1846682865561113223[65] = 0;
   out_1846682865561113223[66] = 0;
   out_1846682865561113223[67] = 0;
   out_1846682865561113223[68] = 0;
   out_1846682865561113223[69] = 0;
   out_1846682865561113223[70] = 1;
   out_1846682865561113223[71] = 0;
   out_1846682865561113223[72] = 0;
   out_1846682865561113223[73] = 0;
   out_1846682865561113223[74] = 0;
   out_1846682865561113223[75] = 0;
   out_1846682865561113223[76] = 0;
   out_1846682865561113223[77] = 0;
   out_1846682865561113223[78] = 0;
   out_1846682865561113223[79] = 0;
   out_1846682865561113223[80] = 1;
}
void h_25(double *state, double *unused, double *out_4028775186055664403) {
   out_4028775186055664403[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4908905495101362676) {
   out_4908905495101362676[0] = 0;
   out_4908905495101362676[1] = 0;
   out_4908905495101362676[2] = 0;
   out_4908905495101362676[3] = 0;
   out_4908905495101362676[4] = 0;
   out_4908905495101362676[5] = 0;
   out_4908905495101362676[6] = 1;
   out_4908905495101362676[7] = 0;
   out_4908905495101362676[8] = 0;
}
void h_24(double *state, double *unused, double *out_7860148482378442816) {
   out_7860148482378442816[0] = state[4];
   out_7860148482378442816[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2731691071494212703) {
   out_2731691071494212703[0] = 0;
   out_2731691071494212703[1] = 0;
   out_2731691071494212703[2] = 0;
   out_2731691071494212703[3] = 0;
   out_2731691071494212703[4] = 1;
   out_2731691071494212703[5] = 0;
   out_2731691071494212703[6] = 0;
   out_2731691071494212703[7] = 0;
   out_2731691071494212703[8] = 0;
   out_2731691071494212703[9] = 0;
   out_2731691071494212703[10] = 0;
   out_2731691071494212703[11] = 0;
   out_2731691071494212703[12] = 0;
   out_2731691071494212703[13] = 0;
   out_2731691071494212703[14] = 1;
   out_2731691071494212703[15] = 0;
   out_2731691071494212703[16] = 0;
   out_2731691071494212703[17] = 0;
}
void h_30(double *state, double *unused, double *out_4107489310009488734) {
   out_4107489310009488734[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2390572536594114049) {
   out_2390572536594114049[0] = 0;
   out_2390572536594114049[1] = 0;
   out_2390572536594114049[2] = 0;
   out_2390572536594114049[3] = 0;
   out_2390572536594114049[4] = 1;
   out_2390572536594114049[5] = 0;
   out_2390572536594114049[6] = 0;
   out_2390572536594114049[7] = 0;
   out_2390572536594114049[8] = 0;
}
void h_26(double *state, double *unused, double *out_9109162577670334379) {
   out_9109162577670334379[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8650408813975418900) {
   out_8650408813975418900[0] = 0;
   out_8650408813975418900[1] = 0;
   out_8650408813975418900[2] = 0;
   out_8650408813975418900[3] = 0;
   out_8650408813975418900[4] = 0;
   out_8650408813975418900[5] = 0;
   out_8650408813975418900[6] = 0;
   out_8650408813975418900[7] = 1;
   out_8650408813975418900[8] = 0;
}
void h_27(double *state, double *unused, double *out_972898081605181242) {
   out_972898081605181242[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4565335848394538960) {
   out_4565335848394538960[0] = 0;
   out_4565335848394538960[1] = 0;
   out_4565335848394538960[2] = 0;
   out_4565335848394538960[3] = 1;
   out_4565335848394538960[4] = 0;
   out_4565335848394538960[5] = 0;
   out_4565335848394538960[6] = 0;
   out_4565335848394538960[7] = 0;
   out_4565335848394538960[8] = 0;
}
void h_29(double *state, double *unused, double *out_7523762599658558750) {
   out_7523762599658558750[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1880341192279721865) {
   out_1880341192279721865[0] = 0;
   out_1880341192279721865[1] = 1;
   out_1880341192279721865[2] = 0;
   out_1880341192279721865[3] = 0;
   out_1880341192279721865[4] = 0;
   out_1880341192279721865[5] = 0;
   out_1880341192279721865[6] = 0;
   out_1880341192279721865[7] = 0;
   out_1880341192279721865[8] = 0;
}
void h_28(double *state, double *unused, double *out_6117788144720828779) {
   out_6117788144720828779[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6962740209349252439) {
   out_6962740209349252439[0] = 1;
   out_6962740209349252439[1] = 0;
   out_6962740209349252439[2] = 0;
   out_6962740209349252439[3] = 0;
   out_6962740209349252439[4] = 0;
   out_6962740209349252439[5] = 0;
   out_6962740209349252439[6] = 0;
   out_6962740209349252439[7] = 0;
   out_6962740209349252439[8] = 0;
}
void h_31(double *state, double *unused, double *out_2820532584691991347) {
   out_2820532584691991347[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9170127157500781240) {
   out_9170127157500781240[0] = 0;
   out_9170127157500781240[1] = 0;
   out_9170127157500781240[2] = 0;
   out_9170127157500781240[3] = 0;
   out_9170127157500781240[4] = 0;
   out_9170127157500781240[5] = 0;
   out_9170127157500781240[6] = 0;
   out_9170127157500781240[7] = 0;
   out_9170127157500781240[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8162787069879679621) {
  err_fun(nom_x, delta_x, out_8162787069879679621);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4646430404185096575) {
  inv_err_fun(nom_x, true_x, out_4646430404185096575);
}
void car_H_mod_fun(double *state, double *out_8634146753423524100) {
  H_mod_fun(state, out_8634146753423524100);
}
void car_f_fun(double *state, double dt, double *out_6186994550747992320) {
  f_fun(state,  dt, out_6186994550747992320);
}
void car_F_fun(double *state, double dt, double *out_1846682865561113223) {
  F_fun(state,  dt, out_1846682865561113223);
}
void car_h_25(double *state, double *unused, double *out_4028775186055664403) {
  h_25(state, unused, out_4028775186055664403);
}
void car_H_25(double *state, double *unused, double *out_4908905495101362676) {
  H_25(state, unused, out_4908905495101362676);
}
void car_h_24(double *state, double *unused, double *out_7860148482378442816) {
  h_24(state, unused, out_7860148482378442816);
}
void car_H_24(double *state, double *unused, double *out_2731691071494212703) {
  H_24(state, unused, out_2731691071494212703);
}
void car_h_30(double *state, double *unused, double *out_4107489310009488734) {
  h_30(state, unused, out_4107489310009488734);
}
void car_H_30(double *state, double *unused, double *out_2390572536594114049) {
  H_30(state, unused, out_2390572536594114049);
}
void car_h_26(double *state, double *unused, double *out_9109162577670334379) {
  h_26(state, unused, out_9109162577670334379);
}
void car_H_26(double *state, double *unused, double *out_8650408813975418900) {
  H_26(state, unused, out_8650408813975418900);
}
void car_h_27(double *state, double *unused, double *out_972898081605181242) {
  h_27(state, unused, out_972898081605181242);
}
void car_H_27(double *state, double *unused, double *out_4565335848394538960) {
  H_27(state, unused, out_4565335848394538960);
}
void car_h_29(double *state, double *unused, double *out_7523762599658558750) {
  h_29(state, unused, out_7523762599658558750);
}
void car_H_29(double *state, double *unused, double *out_1880341192279721865) {
  H_29(state, unused, out_1880341192279721865);
}
void car_h_28(double *state, double *unused, double *out_6117788144720828779) {
  h_28(state, unused, out_6117788144720828779);
}
void car_H_28(double *state, double *unused, double *out_6962740209349252439) {
  H_28(state, unused, out_6962740209349252439);
}
void car_h_31(double *state, double *unused, double *out_2820532584691991347) {
  h_31(state, unused, out_2820532584691991347);
}
void car_H_31(double *state, double *unused, double *out_9170127157500781240) {
  H_31(state, unused, out_9170127157500781240);
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
