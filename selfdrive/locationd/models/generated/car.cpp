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
void err_fun(double *nom_x, double *delta_x, double *out_2612944280414334461) {
   out_2612944280414334461[0] = delta_x[0] + nom_x[0];
   out_2612944280414334461[1] = delta_x[1] + nom_x[1];
   out_2612944280414334461[2] = delta_x[2] + nom_x[2];
   out_2612944280414334461[3] = delta_x[3] + nom_x[3];
   out_2612944280414334461[4] = delta_x[4] + nom_x[4];
   out_2612944280414334461[5] = delta_x[5] + nom_x[5];
   out_2612944280414334461[6] = delta_x[6] + nom_x[6];
   out_2612944280414334461[7] = delta_x[7] + nom_x[7];
   out_2612944280414334461[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5198894641976812856) {
   out_5198894641976812856[0] = -nom_x[0] + true_x[0];
   out_5198894641976812856[1] = -nom_x[1] + true_x[1];
   out_5198894641976812856[2] = -nom_x[2] + true_x[2];
   out_5198894641976812856[3] = -nom_x[3] + true_x[3];
   out_5198894641976812856[4] = -nom_x[4] + true_x[4];
   out_5198894641976812856[5] = -nom_x[5] + true_x[5];
   out_5198894641976812856[6] = -nom_x[6] + true_x[6];
   out_5198894641976812856[7] = -nom_x[7] + true_x[7];
   out_5198894641976812856[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4469793830309029803) {
   out_4469793830309029803[0] = 1.0;
   out_4469793830309029803[1] = 0.0;
   out_4469793830309029803[2] = 0.0;
   out_4469793830309029803[3] = 0.0;
   out_4469793830309029803[4] = 0.0;
   out_4469793830309029803[5] = 0.0;
   out_4469793830309029803[6] = 0.0;
   out_4469793830309029803[7] = 0.0;
   out_4469793830309029803[8] = 0.0;
   out_4469793830309029803[9] = 0.0;
   out_4469793830309029803[10] = 1.0;
   out_4469793830309029803[11] = 0.0;
   out_4469793830309029803[12] = 0.0;
   out_4469793830309029803[13] = 0.0;
   out_4469793830309029803[14] = 0.0;
   out_4469793830309029803[15] = 0.0;
   out_4469793830309029803[16] = 0.0;
   out_4469793830309029803[17] = 0.0;
   out_4469793830309029803[18] = 0.0;
   out_4469793830309029803[19] = 0.0;
   out_4469793830309029803[20] = 1.0;
   out_4469793830309029803[21] = 0.0;
   out_4469793830309029803[22] = 0.0;
   out_4469793830309029803[23] = 0.0;
   out_4469793830309029803[24] = 0.0;
   out_4469793830309029803[25] = 0.0;
   out_4469793830309029803[26] = 0.0;
   out_4469793830309029803[27] = 0.0;
   out_4469793830309029803[28] = 0.0;
   out_4469793830309029803[29] = 0.0;
   out_4469793830309029803[30] = 1.0;
   out_4469793830309029803[31] = 0.0;
   out_4469793830309029803[32] = 0.0;
   out_4469793830309029803[33] = 0.0;
   out_4469793830309029803[34] = 0.0;
   out_4469793830309029803[35] = 0.0;
   out_4469793830309029803[36] = 0.0;
   out_4469793830309029803[37] = 0.0;
   out_4469793830309029803[38] = 0.0;
   out_4469793830309029803[39] = 0.0;
   out_4469793830309029803[40] = 1.0;
   out_4469793830309029803[41] = 0.0;
   out_4469793830309029803[42] = 0.0;
   out_4469793830309029803[43] = 0.0;
   out_4469793830309029803[44] = 0.0;
   out_4469793830309029803[45] = 0.0;
   out_4469793830309029803[46] = 0.0;
   out_4469793830309029803[47] = 0.0;
   out_4469793830309029803[48] = 0.0;
   out_4469793830309029803[49] = 0.0;
   out_4469793830309029803[50] = 1.0;
   out_4469793830309029803[51] = 0.0;
   out_4469793830309029803[52] = 0.0;
   out_4469793830309029803[53] = 0.0;
   out_4469793830309029803[54] = 0.0;
   out_4469793830309029803[55] = 0.0;
   out_4469793830309029803[56] = 0.0;
   out_4469793830309029803[57] = 0.0;
   out_4469793830309029803[58] = 0.0;
   out_4469793830309029803[59] = 0.0;
   out_4469793830309029803[60] = 1.0;
   out_4469793830309029803[61] = 0.0;
   out_4469793830309029803[62] = 0.0;
   out_4469793830309029803[63] = 0.0;
   out_4469793830309029803[64] = 0.0;
   out_4469793830309029803[65] = 0.0;
   out_4469793830309029803[66] = 0.0;
   out_4469793830309029803[67] = 0.0;
   out_4469793830309029803[68] = 0.0;
   out_4469793830309029803[69] = 0.0;
   out_4469793830309029803[70] = 1.0;
   out_4469793830309029803[71] = 0.0;
   out_4469793830309029803[72] = 0.0;
   out_4469793830309029803[73] = 0.0;
   out_4469793830309029803[74] = 0.0;
   out_4469793830309029803[75] = 0.0;
   out_4469793830309029803[76] = 0.0;
   out_4469793830309029803[77] = 0.0;
   out_4469793830309029803[78] = 0.0;
   out_4469793830309029803[79] = 0.0;
   out_4469793830309029803[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4731745578040490156) {
   out_4731745578040490156[0] = state[0];
   out_4731745578040490156[1] = state[1];
   out_4731745578040490156[2] = state[2];
   out_4731745578040490156[3] = state[3];
   out_4731745578040490156[4] = state[4];
   out_4731745578040490156[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4731745578040490156[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4731745578040490156[7] = state[7];
   out_4731745578040490156[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8512268445394675783) {
   out_8512268445394675783[0] = 1;
   out_8512268445394675783[1] = 0;
   out_8512268445394675783[2] = 0;
   out_8512268445394675783[3] = 0;
   out_8512268445394675783[4] = 0;
   out_8512268445394675783[5] = 0;
   out_8512268445394675783[6] = 0;
   out_8512268445394675783[7] = 0;
   out_8512268445394675783[8] = 0;
   out_8512268445394675783[9] = 0;
   out_8512268445394675783[10] = 1;
   out_8512268445394675783[11] = 0;
   out_8512268445394675783[12] = 0;
   out_8512268445394675783[13] = 0;
   out_8512268445394675783[14] = 0;
   out_8512268445394675783[15] = 0;
   out_8512268445394675783[16] = 0;
   out_8512268445394675783[17] = 0;
   out_8512268445394675783[18] = 0;
   out_8512268445394675783[19] = 0;
   out_8512268445394675783[20] = 1;
   out_8512268445394675783[21] = 0;
   out_8512268445394675783[22] = 0;
   out_8512268445394675783[23] = 0;
   out_8512268445394675783[24] = 0;
   out_8512268445394675783[25] = 0;
   out_8512268445394675783[26] = 0;
   out_8512268445394675783[27] = 0;
   out_8512268445394675783[28] = 0;
   out_8512268445394675783[29] = 0;
   out_8512268445394675783[30] = 1;
   out_8512268445394675783[31] = 0;
   out_8512268445394675783[32] = 0;
   out_8512268445394675783[33] = 0;
   out_8512268445394675783[34] = 0;
   out_8512268445394675783[35] = 0;
   out_8512268445394675783[36] = 0;
   out_8512268445394675783[37] = 0;
   out_8512268445394675783[38] = 0;
   out_8512268445394675783[39] = 0;
   out_8512268445394675783[40] = 1;
   out_8512268445394675783[41] = 0;
   out_8512268445394675783[42] = 0;
   out_8512268445394675783[43] = 0;
   out_8512268445394675783[44] = 0;
   out_8512268445394675783[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8512268445394675783[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8512268445394675783[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8512268445394675783[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8512268445394675783[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8512268445394675783[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8512268445394675783[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8512268445394675783[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8512268445394675783[53] = -9.8100000000000005*dt;
   out_8512268445394675783[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8512268445394675783[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8512268445394675783[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8512268445394675783[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8512268445394675783[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8512268445394675783[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8512268445394675783[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8512268445394675783[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8512268445394675783[62] = 0;
   out_8512268445394675783[63] = 0;
   out_8512268445394675783[64] = 0;
   out_8512268445394675783[65] = 0;
   out_8512268445394675783[66] = 0;
   out_8512268445394675783[67] = 0;
   out_8512268445394675783[68] = 0;
   out_8512268445394675783[69] = 0;
   out_8512268445394675783[70] = 1;
   out_8512268445394675783[71] = 0;
   out_8512268445394675783[72] = 0;
   out_8512268445394675783[73] = 0;
   out_8512268445394675783[74] = 0;
   out_8512268445394675783[75] = 0;
   out_8512268445394675783[76] = 0;
   out_8512268445394675783[77] = 0;
   out_8512268445394675783[78] = 0;
   out_8512268445394675783[79] = 0;
   out_8512268445394675783[80] = 1;
}
void h_25(double *state, double *unused, double *out_1706109444363025734) {
   out_1706109444363025734[0] = state[6];
}
void H_25(double *state, double *unused, double *out_946813180319538126) {
   out_946813180319538126[0] = 0;
   out_946813180319538126[1] = 0;
   out_946813180319538126[2] = 0;
   out_946813180319538126[3] = 0;
   out_946813180319538126[4] = 0;
   out_946813180319538126[5] = 0;
   out_946813180319538126[6] = 1;
   out_946813180319538126[7] = 0;
   out_946813180319538126[8] = 0;
}
void h_24(double *state, double *unused, double *out_6287632968672598585) {
   out_6287632968672598585[0] = state[4];
   out_6287632968672598585[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2611112418482785010) {
   out_2611112418482785010[0] = 0;
   out_2611112418482785010[1] = 0;
   out_2611112418482785010[2] = 0;
   out_2611112418482785010[3] = 0;
   out_2611112418482785010[4] = 1;
   out_2611112418482785010[5] = 0;
   out_2611112418482785010[6] = 0;
   out_2611112418482785010[7] = 0;
   out_2611112418482785010[8] = 0;
   out_2611112418482785010[9] = 0;
   out_2611112418482785010[10] = 0;
   out_2611112418482785010[11] = 0;
   out_2611112418482785010[12] = 0;
   out_2611112418482785010[13] = 0;
   out_2611112418482785010[14] = 1;
   out_2611112418482785010[15] = 0;
   out_2611112418482785010[16] = 0;
   out_2611112418482785010[17] = 0;
}
void h_30(double *state, double *unused, double *out_9166668208611914059) {
   out_9166668208611914059[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5969877161172078629) {
   out_5969877161172078629[0] = 0;
   out_5969877161172078629[1] = 0;
   out_5969877161172078629[2] = 0;
   out_5969877161172078629[3] = 0;
   out_5969877161172078629[4] = 1;
   out_5969877161172078629[5] = 0;
   out_5969877161172078629[6] = 0;
   out_5969877161172078629[7] = 0;
   out_5969877161172078629[8] = 0;
}
void h_26(double *state, double *unused, double *out_4278402597436791912) {
   out_4278402597436791912[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4688316499193594350) {
   out_4688316499193594350[0] = 0;
   out_4688316499193594350[1] = 0;
   out_4688316499193594350[2] = 0;
   out_4688316499193594350[3] = 0;
   out_4688316499193594350[4] = 0;
   out_4688316499193594350[5] = 0;
   out_4688316499193594350[6] = 0;
   out_4688316499193594350[7] = 1;
   out_4688316499193594350[8] = 0;
}
void h_27(double *state, double *unused, double *out_2649619297264688169) {
   out_2649619297264688169[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3795113849371653718) {
   out_3795113849371653718[0] = 0;
   out_3795113849371653718[1] = 0;
   out_3795113849371653718[2] = 0;
   out_3795113849371653718[3] = 1;
   out_3795113849371653718[4] = 0;
   out_3795113849371653718[5] = 0;
   out_3795113849371653718[6] = 0;
   out_3795113849371653718[7] = 0;
   out_3795113849371653718[8] = 0;
}
void h_29(double *state, double *unused, double *out_4100250689893571336) {
   out_4100250689893571336[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2081751122502102685) {
   out_2081751122502102685[0] = 0;
   out_2081751122502102685[1] = 1;
   out_2081751122502102685[2] = 0;
   out_2081751122502102685[3] = 0;
   out_2081751122502102685[4] = 0;
   out_2081751122502102685[5] = 0;
   out_2081751122502102685[6] = 0;
   out_2081751122502102685[7] = 0;
   out_2081751122502102685[8] = 0;
}
void h_28(double *state, double *unused, double *out_5254024618073385820) {
   out_5254024618073385820[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3000647894567427889) {
   out_3000647894567427889[0] = 1;
   out_3000647894567427889[1] = 0;
   out_3000647894567427889[2] = 0;
   out_3000647894567427889[3] = 0;
   out_3000647894567427889[4] = 0;
   out_3000647894567427889[5] = 0;
   out_3000647894567427889[6] = 0;
   out_3000647894567427889[7] = 0;
   out_3000647894567427889[8] = 0;
}
void h_31(double *state, double *unused, double *out_6933768032355516150) {
   out_6933768032355516150[0] = state[8];
}
void H_31(double *state, double *unused, double *out_916167218442577698) {
   out_916167218442577698[0] = 0;
   out_916167218442577698[1] = 0;
   out_916167218442577698[2] = 0;
   out_916167218442577698[3] = 0;
   out_916167218442577698[4] = 0;
   out_916167218442577698[5] = 0;
   out_916167218442577698[6] = 0;
   out_916167218442577698[7] = 0;
   out_916167218442577698[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2612944280414334461) {
  err_fun(nom_x, delta_x, out_2612944280414334461);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5198894641976812856) {
  inv_err_fun(nom_x, true_x, out_5198894641976812856);
}
void car_H_mod_fun(double *state, double *out_4469793830309029803) {
  H_mod_fun(state, out_4469793830309029803);
}
void car_f_fun(double *state, double dt, double *out_4731745578040490156) {
  f_fun(state,  dt, out_4731745578040490156);
}
void car_F_fun(double *state, double dt, double *out_8512268445394675783) {
  F_fun(state,  dt, out_8512268445394675783);
}
void car_h_25(double *state, double *unused, double *out_1706109444363025734) {
  h_25(state, unused, out_1706109444363025734);
}
void car_H_25(double *state, double *unused, double *out_946813180319538126) {
  H_25(state, unused, out_946813180319538126);
}
void car_h_24(double *state, double *unused, double *out_6287632968672598585) {
  h_24(state, unused, out_6287632968672598585);
}
void car_H_24(double *state, double *unused, double *out_2611112418482785010) {
  H_24(state, unused, out_2611112418482785010);
}
void car_h_30(double *state, double *unused, double *out_9166668208611914059) {
  h_30(state, unused, out_9166668208611914059);
}
void car_H_30(double *state, double *unused, double *out_5969877161172078629) {
  H_30(state, unused, out_5969877161172078629);
}
void car_h_26(double *state, double *unused, double *out_4278402597436791912) {
  h_26(state, unused, out_4278402597436791912);
}
void car_H_26(double *state, double *unused, double *out_4688316499193594350) {
  H_26(state, unused, out_4688316499193594350);
}
void car_h_27(double *state, double *unused, double *out_2649619297264688169) {
  h_27(state, unused, out_2649619297264688169);
}
void car_H_27(double *state, double *unused, double *out_3795113849371653718) {
  H_27(state, unused, out_3795113849371653718);
}
void car_h_29(double *state, double *unused, double *out_4100250689893571336) {
  h_29(state, unused, out_4100250689893571336);
}
void car_H_29(double *state, double *unused, double *out_2081751122502102685) {
  H_29(state, unused, out_2081751122502102685);
}
void car_h_28(double *state, double *unused, double *out_5254024618073385820) {
  h_28(state, unused, out_5254024618073385820);
}
void car_H_28(double *state, double *unused, double *out_3000647894567427889) {
  H_28(state, unused, out_3000647894567427889);
}
void car_h_31(double *state, double *unused, double *out_6933768032355516150) {
  h_31(state, unused, out_6933768032355516150);
}
void car_H_31(double *state, double *unused, double *out_916167218442577698) {
  H_31(state, unused, out_916167218442577698);
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
