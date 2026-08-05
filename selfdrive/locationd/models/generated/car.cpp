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
void err_fun(double *nom_x, double *delta_x, double *out_8231570846104287187) {
   out_8231570846104287187[0] = delta_x[0] + nom_x[0];
   out_8231570846104287187[1] = delta_x[1] + nom_x[1];
   out_8231570846104287187[2] = delta_x[2] + nom_x[2];
   out_8231570846104287187[3] = delta_x[3] + nom_x[3];
   out_8231570846104287187[4] = delta_x[4] + nom_x[4];
   out_8231570846104287187[5] = delta_x[5] + nom_x[5];
   out_8231570846104287187[6] = delta_x[6] + nom_x[6];
   out_8231570846104287187[7] = delta_x[7] + nom_x[7];
   out_8231570846104287187[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3172369976442532592) {
   out_3172369976442532592[0] = -nom_x[0] + true_x[0];
   out_3172369976442532592[1] = -nom_x[1] + true_x[1];
   out_3172369976442532592[2] = -nom_x[2] + true_x[2];
   out_3172369976442532592[3] = -nom_x[3] + true_x[3];
   out_3172369976442532592[4] = -nom_x[4] + true_x[4];
   out_3172369976442532592[5] = -nom_x[5] + true_x[5];
   out_3172369976442532592[6] = -nom_x[6] + true_x[6];
   out_3172369976442532592[7] = -nom_x[7] + true_x[7];
   out_3172369976442532592[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3076533359310538874) {
   out_3076533359310538874[0] = 1.0;
   out_3076533359310538874[1] = 0.0;
   out_3076533359310538874[2] = 0.0;
   out_3076533359310538874[3] = 0.0;
   out_3076533359310538874[4] = 0.0;
   out_3076533359310538874[5] = 0.0;
   out_3076533359310538874[6] = 0.0;
   out_3076533359310538874[7] = 0.0;
   out_3076533359310538874[8] = 0.0;
   out_3076533359310538874[9] = 0.0;
   out_3076533359310538874[10] = 1.0;
   out_3076533359310538874[11] = 0.0;
   out_3076533359310538874[12] = 0.0;
   out_3076533359310538874[13] = 0.0;
   out_3076533359310538874[14] = 0.0;
   out_3076533359310538874[15] = 0.0;
   out_3076533359310538874[16] = 0.0;
   out_3076533359310538874[17] = 0.0;
   out_3076533359310538874[18] = 0.0;
   out_3076533359310538874[19] = 0.0;
   out_3076533359310538874[20] = 1.0;
   out_3076533359310538874[21] = 0.0;
   out_3076533359310538874[22] = 0.0;
   out_3076533359310538874[23] = 0.0;
   out_3076533359310538874[24] = 0.0;
   out_3076533359310538874[25] = 0.0;
   out_3076533359310538874[26] = 0.0;
   out_3076533359310538874[27] = 0.0;
   out_3076533359310538874[28] = 0.0;
   out_3076533359310538874[29] = 0.0;
   out_3076533359310538874[30] = 1.0;
   out_3076533359310538874[31] = 0.0;
   out_3076533359310538874[32] = 0.0;
   out_3076533359310538874[33] = 0.0;
   out_3076533359310538874[34] = 0.0;
   out_3076533359310538874[35] = 0.0;
   out_3076533359310538874[36] = 0.0;
   out_3076533359310538874[37] = 0.0;
   out_3076533359310538874[38] = 0.0;
   out_3076533359310538874[39] = 0.0;
   out_3076533359310538874[40] = 1.0;
   out_3076533359310538874[41] = 0.0;
   out_3076533359310538874[42] = 0.0;
   out_3076533359310538874[43] = 0.0;
   out_3076533359310538874[44] = 0.0;
   out_3076533359310538874[45] = 0.0;
   out_3076533359310538874[46] = 0.0;
   out_3076533359310538874[47] = 0.0;
   out_3076533359310538874[48] = 0.0;
   out_3076533359310538874[49] = 0.0;
   out_3076533359310538874[50] = 1.0;
   out_3076533359310538874[51] = 0.0;
   out_3076533359310538874[52] = 0.0;
   out_3076533359310538874[53] = 0.0;
   out_3076533359310538874[54] = 0.0;
   out_3076533359310538874[55] = 0.0;
   out_3076533359310538874[56] = 0.0;
   out_3076533359310538874[57] = 0.0;
   out_3076533359310538874[58] = 0.0;
   out_3076533359310538874[59] = 0.0;
   out_3076533359310538874[60] = 1.0;
   out_3076533359310538874[61] = 0.0;
   out_3076533359310538874[62] = 0.0;
   out_3076533359310538874[63] = 0.0;
   out_3076533359310538874[64] = 0.0;
   out_3076533359310538874[65] = 0.0;
   out_3076533359310538874[66] = 0.0;
   out_3076533359310538874[67] = 0.0;
   out_3076533359310538874[68] = 0.0;
   out_3076533359310538874[69] = 0.0;
   out_3076533359310538874[70] = 1.0;
   out_3076533359310538874[71] = 0.0;
   out_3076533359310538874[72] = 0.0;
   out_3076533359310538874[73] = 0.0;
   out_3076533359310538874[74] = 0.0;
   out_3076533359310538874[75] = 0.0;
   out_3076533359310538874[76] = 0.0;
   out_3076533359310538874[77] = 0.0;
   out_3076533359310538874[78] = 0.0;
   out_3076533359310538874[79] = 0.0;
   out_3076533359310538874[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4555482552254326600) {
   out_4555482552254326600[0] = state[0];
   out_4555482552254326600[1] = state[1];
   out_4555482552254326600[2] = state[2];
   out_4555482552254326600[3] = state[3];
   out_4555482552254326600[4] = state[4];
   out_4555482552254326600[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4555482552254326600[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4555482552254326600[7] = state[7];
   out_4555482552254326600[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3328421804098371723) {
   out_3328421804098371723[0] = 1;
   out_3328421804098371723[1] = 0;
   out_3328421804098371723[2] = 0;
   out_3328421804098371723[3] = 0;
   out_3328421804098371723[4] = 0;
   out_3328421804098371723[5] = 0;
   out_3328421804098371723[6] = 0;
   out_3328421804098371723[7] = 0;
   out_3328421804098371723[8] = 0;
   out_3328421804098371723[9] = 0;
   out_3328421804098371723[10] = 1;
   out_3328421804098371723[11] = 0;
   out_3328421804098371723[12] = 0;
   out_3328421804098371723[13] = 0;
   out_3328421804098371723[14] = 0;
   out_3328421804098371723[15] = 0;
   out_3328421804098371723[16] = 0;
   out_3328421804098371723[17] = 0;
   out_3328421804098371723[18] = 0;
   out_3328421804098371723[19] = 0;
   out_3328421804098371723[20] = 1;
   out_3328421804098371723[21] = 0;
   out_3328421804098371723[22] = 0;
   out_3328421804098371723[23] = 0;
   out_3328421804098371723[24] = 0;
   out_3328421804098371723[25] = 0;
   out_3328421804098371723[26] = 0;
   out_3328421804098371723[27] = 0;
   out_3328421804098371723[28] = 0;
   out_3328421804098371723[29] = 0;
   out_3328421804098371723[30] = 1;
   out_3328421804098371723[31] = 0;
   out_3328421804098371723[32] = 0;
   out_3328421804098371723[33] = 0;
   out_3328421804098371723[34] = 0;
   out_3328421804098371723[35] = 0;
   out_3328421804098371723[36] = 0;
   out_3328421804098371723[37] = 0;
   out_3328421804098371723[38] = 0;
   out_3328421804098371723[39] = 0;
   out_3328421804098371723[40] = 1;
   out_3328421804098371723[41] = 0;
   out_3328421804098371723[42] = 0;
   out_3328421804098371723[43] = 0;
   out_3328421804098371723[44] = 0;
   out_3328421804098371723[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3328421804098371723[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3328421804098371723[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3328421804098371723[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3328421804098371723[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3328421804098371723[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3328421804098371723[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3328421804098371723[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3328421804098371723[53] = -9.8100000000000005*dt;
   out_3328421804098371723[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3328421804098371723[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3328421804098371723[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3328421804098371723[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3328421804098371723[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3328421804098371723[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3328421804098371723[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3328421804098371723[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3328421804098371723[62] = 0;
   out_3328421804098371723[63] = 0;
   out_3328421804098371723[64] = 0;
   out_3328421804098371723[65] = 0;
   out_3328421804098371723[66] = 0;
   out_3328421804098371723[67] = 0;
   out_3328421804098371723[68] = 0;
   out_3328421804098371723[69] = 0;
   out_3328421804098371723[70] = 1;
   out_3328421804098371723[71] = 0;
   out_3328421804098371723[72] = 0;
   out_3328421804098371723[73] = 0;
   out_3328421804098371723[74] = 0;
   out_3328421804098371723[75] = 0;
   out_3328421804098371723[76] = 0;
   out_3328421804098371723[77] = 0;
   out_3328421804098371723[78] = 0;
   out_3328421804098371723[79] = 0;
   out_3328421804098371723[80] = 1;
}
void h_25(double *state, double *unused, double *out_8332545656712404133) {
   out_8332545656712404133[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7871364202238407983) {
   out_7871364202238407983[0] = 0;
   out_7871364202238407983[1] = 0;
   out_7871364202238407983[2] = 0;
   out_7871364202238407983[3] = 0;
   out_7871364202238407983[4] = 0;
   out_7871364202238407983[5] = 0;
   out_7871364202238407983[6] = 1;
   out_7871364202238407983[7] = 0;
   out_7871364202238407983[8] = 0;
}
void h_24(double *state, double *unused, double *out_8057594001447114148) {
   out_8057594001447114148[0] = state[4];
   out_8057594001447114148[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8402730272465644067) {
   out_8402730272465644067[0] = 0;
   out_8402730272465644067[1] = 0;
   out_8402730272465644067[2] = 0;
   out_8402730272465644067[3] = 0;
   out_8402730272465644067[4] = 1;
   out_8402730272465644067[5] = 0;
   out_8402730272465644067[6] = 0;
   out_8402730272465644067[7] = 0;
   out_8402730272465644067[8] = 0;
   out_8402730272465644067[9] = 0;
   out_8402730272465644067[10] = 0;
   out_8402730272465644067[11] = 0;
   out_8402730272465644067[12] = 0;
   out_8402730272465644067[13] = 0;
   out_8402730272465644067[14] = 1;
   out_8402730272465644067[15] = 0;
   out_8402730272465644067[16] = 0;
   out_8402730272465644067[17] = 0;
}
void h_30(double *state, double *unused, double *out_7731127499099040579) {
   out_7731127499099040579[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5353031243731159356) {
   out_5353031243731159356[0] = 0;
   out_5353031243731159356[1] = 0;
   out_5353031243731159356[2] = 0;
   out_5353031243731159356[3] = 0;
   out_5353031243731159356[4] = 1;
   out_5353031243731159356[5] = 0;
   out_5353031243731159356[6] = 0;
   out_5353031243731159356[7] = 0;
   out_5353031243731159356[8] = 0;
}
void h_26(double *state, double *unused, double *out_8895494501810979159) {
   out_8895494501810979159[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6833876552597087409) {
   out_6833876552597087409[0] = 0;
   out_6833876552597087409[1] = 0;
   out_6833876552597087409[2] = 0;
   out_6833876552597087409[3] = 0;
   out_6833876552597087409[4] = 0;
   out_6833876552597087409[5] = 0;
   out_6833876552597087409[6] = 0;
   out_6833876552597087409[7] = 1;
   out_6833876552597087409[8] = 0;
}
void h_27(double *state, double *unused, double *out_8935861541388881650) {
   out_8935861541388881650[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7527794555531584267) {
   out_7527794555531584267[0] = 0;
   out_7527794555531584267[1] = 0;
   out_7527794555531584267[2] = 0;
   out_7527794555531584267[3] = 1;
   out_7527794555531584267[4] = 0;
   out_7527794555531584267[5] = 0;
   out_7527794555531584267[6] = 0;
   out_7527794555531584267[7] = 0;
   out_7527794555531584267[8] = 0;
}
void h_29(double *state, double *unused, double *out_9211055603673387539) {
   out_9211055603673387539[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4842799899416767172) {
   out_4842799899416767172[0] = 0;
   out_4842799899416767172[1] = 1;
   out_4842799899416767172[2] = 0;
   out_4842799899416767172[3] = 0;
   out_4842799899416767172[4] = 0;
   out_4842799899416767172[5] = 0;
   out_4842799899416767172[6] = 0;
   out_4842799899416767172[7] = 0;
   out_4842799899416767172[8] = 0;
}
void h_28(double *state, double *unused, double *out_7849134893032417323) {
   out_7849134893032417323[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8521545157223253870) {
   out_8521545157223253870[0] = 1;
   out_8521545157223253870[1] = 0;
   out_8521545157223253870[2] = 0;
   out_8521545157223253870[3] = 0;
   out_8521545157223253870[4] = 0;
   out_8521545157223253870[5] = 0;
   out_8521545157223253870[6] = 0;
   out_8521545157223253870[7] = 0;
   out_8521545157223253870[8] = 0;
}
void h_31(double *state, double *unused, double *out_4695096128458381783) {
   out_4695096128458381783[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6207668450363735933) {
   out_6207668450363735933[0] = 0;
   out_6207668450363735933[1] = 0;
   out_6207668450363735933[2] = 0;
   out_6207668450363735933[3] = 0;
   out_6207668450363735933[4] = 0;
   out_6207668450363735933[5] = 0;
   out_6207668450363735933[6] = 0;
   out_6207668450363735933[7] = 0;
   out_6207668450363735933[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8231570846104287187) {
  err_fun(nom_x, delta_x, out_8231570846104287187);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3172369976442532592) {
  inv_err_fun(nom_x, true_x, out_3172369976442532592);
}
void car_H_mod_fun(double *state, double *out_3076533359310538874) {
  H_mod_fun(state, out_3076533359310538874);
}
void car_f_fun(double *state, double dt, double *out_4555482552254326600) {
  f_fun(state,  dt, out_4555482552254326600);
}
void car_F_fun(double *state, double dt, double *out_3328421804098371723) {
  F_fun(state,  dt, out_3328421804098371723);
}
void car_h_25(double *state, double *unused, double *out_8332545656712404133) {
  h_25(state, unused, out_8332545656712404133);
}
void car_H_25(double *state, double *unused, double *out_7871364202238407983) {
  H_25(state, unused, out_7871364202238407983);
}
void car_h_24(double *state, double *unused, double *out_8057594001447114148) {
  h_24(state, unused, out_8057594001447114148);
}
void car_H_24(double *state, double *unused, double *out_8402730272465644067) {
  H_24(state, unused, out_8402730272465644067);
}
void car_h_30(double *state, double *unused, double *out_7731127499099040579) {
  h_30(state, unused, out_7731127499099040579);
}
void car_H_30(double *state, double *unused, double *out_5353031243731159356) {
  H_30(state, unused, out_5353031243731159356);
}
void car_h_26(double *state, double *unused, double *out_8895494501810979159) {
  h_26(state, unused, out_8895494501810979159);
}
void car_H_26(double *state, double *unused, double *out_6833876552597087409) {
  H_26(state, unused, out_6833876552597087409);
}
void car_h_27(double *state, double *unused, double *out_8935861541388881650) {
  h_27(state, unused, out_8935861541388881650);
}
void car_H_27(double *state, double *unused, double *out_7527794555531584267) {
  H_27(state, unused, out_7527794555531584267);
}
void car_h_29(double *state, double *unused, double *out_9211055603673387539) {
  h_29(state, unused, out_9211055603673387539);
}
void car_H_29(double *state, double *unused, double *out_4842799899416767172) {
  H_29(state, unused, out_4842799899416767172);
}
void car_h_28(double *state, double *unused, double *out_7849134893032417323) {
  h_28(state, unused, out_7849134893032417323);
}
void car_H_28(double *state, double *unused, double *out_8521545157223253870) {
  H_28(state, unused, out_8521545157223253870);
}
void car_h_31(double *state, double *unused, double *out_4695096128458381783) {
  h_31(state, unused, out_4695096128458381783);
}
void car_H_31(double *state, double *unused, double *out_6207668450363735933) {
  H_31(state, unused, out_6207668450363735933);
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
