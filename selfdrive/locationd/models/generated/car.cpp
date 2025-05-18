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
void err_fun(double *nom_x, double *delta_x, double *out_2728693116701389222) {
   out_2728693116701389222[0] = delta_x[0] + nom_x[0];
   out_2728693116701389222[1] = delta_x[1] + nom_x[1];
   out_2728693116701389222[2] = delta_x[2] + nom_x[2];
   out_2728693116701389222[3] = delta_x[3] + nom_x[3];
   out_2728693116701389222[4] = delta_x[4] + nom_x[4];
   out_2728693116701389222[5] = delta_x[5] + nom_x[5];
   out_2728693116701389222[6] = delta_x[6] + nom_x[6];
   out_2728693116701389222[7] = delta_x[7] + nom_x[7];
   out_2728693116701389222[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3002832702974128596) {
   out_3002832702974128596[0] = -nom_x[0] + true_x[0];
   out_3002832702974128596[1] = -nom_x[1] + true_x[1];
   out_3002832702974128596[2] = -nom_x[2] + true_x[2];
   out_3002832702974128596[3] = -nom_x[3] + true_x[3];
   out_3002832702974128596[4] = -nom_x[4] + true_x[4];
   out_3002832702974128596[5] = -nom_x[5] + true_x[5];
   out_3002832702974128596[6] = -nom_x[6] + true_x[6];
   out_3002832702974128596[7] = -nom_x[7] + true_x[7];
   out_3002832702974128596[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7934990902020908970) {
   out_7934990902020908970[0] = 1.0;
   out_7934990902020908970[1] = 0.0;
   out_7934990902020908970[2] = 0.0;
   out_7934990902020908970[3] = 0.0;
   out_7934990902020908970[4] = 0.0;
   out_7934990902020908970[5] = 0.0;
   out_7934990902020908970[6] = 0.0;
   out_7934990902020908970[7] = 0.0;
   out_7934990902020908970[8] = 0.0;
   out_7934990902020908970[9] = 0.0;
   out_7934990902020908970[10] = 1.0;
   out_7934990902020908970[11] = 0.0;
   out_7934990902020908970[12] = 0.0;
   out_7934990902020908970[13] = 0.0;
   out_7934990902020908970[14] = 0.0;
   out_7934990902020908970[15] = 0.0;
   out_7934990902020908970[16] = 0.0;
   out_7934990902020908970[17] = 0.0;
   out_7934990902020908970[18] = 0.0;
   out_7934990902020908970[19] = 0.0;
   out_7934990902020908970[20] = 1.0;
   out_7934990902020908970[21] = 0.0;
   out_7934990902020908970[22] = 0.0;
   out_7934990902020908970[23] = 0.0;
   out_7934990902020908970[24] = 0.0;
   out_7934990902020908970[25] = 0.0;
   out_7934990902020908970[26] = 0.0;
   out_7934990902020908970[27] = 0.0;
   out_7934990902020908970[28] = 0.0;
   out_7934990902020908970[29] = 0.0;
   out_7934990902020908970[30] = 1.0;
   out_7934990902020908970[31] = 0.0;
   out_7934990902020908970[32] = 0.0;
   out_7934990902020908970[33] = 0.0;
   out_7934990902020908970[34] = 0.0;
   out_7934990902020908970[35] = 0.0;
   out_7934990902020908970[36] = 0.0;
   out_7934990902020908970[37] = 0.0;
   out_7934990902020908970[38] = 0.0;
   out_7934990902020908970[39] = 0.0;
   out_7934990902020908970[40] = 1.0;
   out_7934990902020908970[41] = 0.0;
   out_7934990902020908970[42] = 0.0;
   out_7934990902020908970[43] = 0.0;
   out_7934990902020908970[44] = 0.0;
   out_7934990902020908970[45] = 0.0;
   out_7934990902020908970[46] = 0.0;
   out_7934990902020908970[47] = 0.0;
   out_7934990902020908970[48] = 0.0;
   out_7934990902020908970[49] = 0.0;
   out_7934990902020908970[50] = 1.0;
   out_7934990902020908970[51] = 0.0;
   out_7934990902020908970[52] = 0.0;
   out_7934990902020908970[53] = 0.0;
   out_7934990902020908970[54] = 0.0;
   out_7934990902020908970[55] = 0.0;
   out_7934990902020908970[56] = 0.0;
   out_7934990902020908970[57] = 0.0;
   out_7934990902020908970[58] = 0.0;
   out_7934990902020908970[59] = 0.0;
   out_7934990902020908970[60] = 1.0;
   out_7934990902020908970[61] = 0.0;
   out_7934990902020908970[62] = 0.0;
   out_7934990902020908970[63] = 0.0;
   out_7934990902020908970[64] = 0.0;
   out_7934990902020908970[65] = 0.0;
   out_7934990902020908970[66] = 0.0;
   out_7934990902020908970[67] = 0.0;
   out_7934990902020908970[68] = 0.0;
   out_7934990902020908970[69] = 0.0;
   out_7934990902020908970[70] = 1.0;
   out_7934990902020908970[71] = 0.0;
   out_7934990902020908970[72] = 0.0;
   out_7934990902020908970[73] = 0.0;
   out_7934990902020908970[74] = 0.0;
   out_7934990902020908970[75] = 0.0;
   out_7934990902020908970[76] = 0.0;
   out_7934990902020908970[77] = 0.0;
   out_7934990902020908970[78] = 0.0;
   out_7934990902020908970[79] = 0.0;
   out_7934990902020908970[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2890300663064821146) {
   out_2890300663064821146[0] = state[0];
   out_2890300663064821146[1] = state[1];
   out_2890300663064821146[2] = state[2];
   out_2890300663064821146[3] = state[3];
   out_2890300663064821146[4] = state[4];
   out_2890300663064821146[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2890300663064821146[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2890300663064821146[7] = state[7];
   out_2890300663064821146[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6771328482541706301) {
   out_6771328482541706301[0] = 1;
   out_6771328482541706301[1] = 0;
   out_6771328482541706301[2] = 0;
   out_6771328482541706301[3] = 0;
   out_6771328482541706301[4] = 0;
   out_6771328482541706301[5] = 0;
   out_6771328482541706301[6] = 0;
   out_6771328482541706301[7] = 0;
   out_6771328482541706301[8] = 0;
   out_6771328482541706301[9] = 0;
   out_6771328482541706301[10] = 1;
   out_6771328482541706301[11] = 0;
   out_6771328482541706301[12] = 0;
   out_6771328482541706301[13] = 0;
   out_6771328482541706301[14] = 0;
   out_6771328482541706301[15] = 0;
   out_6771328482541706301[16] = 0;
   out_6771328482541706301[17] = 0;
   out_6771328482541706301[18] = 0;
   out_6771328482541706301[19] = 0;
   out_6771328482541706301[20] = 1;
   out_6771328482541706301[21] = 0;
   out_6771328482541706301[22] = 0;
   out_6771328482541706301[23] = 0;
   out_6771328482541706301[24] = 0;
   out_6771328482541706301[25] = 0;
   out_6771328482541706301[26] = 0;
   out_6771328482541706301[27] = 0;
   out_6771328482541706301[28] = 0;
   out_6771328482541706301[29] = 0;
   out_6771328482541706301[30] = 1;
   out_6771328482541706301[31] = 0;
   out_6771328482541706301[32] = 0;
   out_6771328482541706301[33] = 0;
   out_6771328482541706301[34] = 0;
   out_6771328482541706301[35] = 0;
   out_6771328482541706301[36] = 0;
   out_6771328482541706301[37] = 0;
   out_6771328482541706301[38] = 0;
   out_6771328482541706301[39] = 0;
   out_6771328482541706301[40] = 1;
   out_6771328482541706301[41] = 0;
   out_6771328482541706301[42] = 0;
   out_6771328482541706301[43] = 0;
   out_6771328482541706301[44] = 0;
   out_6771328482541706301[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6771328482541706301[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6771328482541706301[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6771328482541706301[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6771328482541706301[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6771328482541706301[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6771328482541706301[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6771328482541706301[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6771328482541706301[53] = -9.8000000000000007*dt;
   out_6771328482541706301[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6771328482541706301[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6771328482541706301[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6771328482541706301[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6771328482541706301[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6771328482541706301[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6771328482541706301[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6771328482541706301[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6771328482541706301[62] = 0;
   out_6771328482541706301[63] = 0;
   out_6771328482541706301[64] = 0;
   out_6771328482541706301[65] = 0;
   out_6771328482541706301[66] = 0;
   out_6771328482541706301[67] = 0;
   out_6771328482541706301[68] = 0;
   out_6771328482541706301[69] = 0;
   out_6771328482541706301[70] = 1;
   out_6771328482541706301[71] = 0;
   out_6771328482541706301[72] = 0;
   out_6771328482541706301[73] = 0;
   out_6771328482541706301[74] = 0;
   out_6771328482541706301[75] = 0;
   out_6771328482541706301[76] = 0;
   out_6771328482541706301[77] = 0;
   out_6771328482541706301[78] = 0;
   out_6771328482541706301[79] = 0;
   out_6771328482541706301[80] = 1;
}
void h_25(double *state, double *unused, double *out_850067438180914005) {
   out_850067438180914005[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7059682157681905990) {
   out_7059682157681905990[0] = 0;
   out_7059682157681905990[1] = 0;
   out_7059682157681905990[2] = 0;
   out_7059682157681905990[3] = 0;
   out_7059682157681905990[4] = 0;
   out_7059682157681905990[5] = 0;
   out_7059682157681905990[6] = 1;
   out_7059682157681905990[7] = 0;
   out_7059682157681905990[8] = 0;
}
void h_24(double *state, double *unused, double *out_347899436057359381) {
   out_347899436057359381[0] = state[4];
   out_347899436057359381[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3501756558879582854) {
   out_3501756558879582854[0] = 0;
   out_3501756558879582854[1] = 0;
   out_3501756558879582854[2] = 0;
   out_3501756558879582854[3] = 0;
   out_3501756558879582854[4] = 1;
   out_3501756558879582854[5] = 0;
   out_3501756558879582854[6] = 0;
   out_3501756558879582854[7] = 0;
   out_3501756558879582854[8] = 0;
   out_3501756558879582854[9] = 0;
   out_3501756558879582854[10] = 0;
   out_3501756558879582854[11] = 0;
   out_3501756558879582854[12] = 0;
   out_3501756558879582854[13] = 0;
   out_3501756558879582854[14] = 1;
   out_3501756558879582854[15] = 0;
   out_3501756558879582854[16] = 0;
   out_3501756558879582854[17] = 0;
}
void h_30(double *state, double *unused, double *out_6610491326067974320) {
   out_6610491326067974320[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4541349199174657363) {
   out_4541349199174657363[0] = 0;
   out_4541349199174657363[1] = 0;
   out_4541349199174657363[2] = 0;
   out_4541349199174657363[3] = 0;
   out_4541349199174657363[4] = 1;
   out_4541349199174657363[5] = 0;
   out_4541349199174657363[6] = 0;
   out_4541349199174657363[7] = 0;
   out_4541349199174657363[8] = 0;
}
void h_26(double *state, double *unused, double *out_6834579479980731651) {
   out_6834579479980731651[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7645558597153589402) {
   out_7645558597153589402[0] = 0;
   out_7645558597153589402[1] = 0;
   out_7645558597153589402[2] = 0;
   out_7645558597153589402[3] = 0;
   out_7645558597153589402[4] = 0;
   out_7645558597153589402[5] = 0;
   out_7645558597153589402[6] = 0;
   out_7645558597153589402[7] = 1;
   out_7645558597153589402[8] = 0;
}
void h_27(double *state, double *unused, double *out_1754337882627783646) {
   out_1754337882627783646[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2317755127990714146) {
   out_2317755127990714146[0] = 0;
   out_2317755127990714146[1] = 0;
   out_2317755127990714146[2] = 0;
   out_2317755127990714146[3] = 1;
   out_2317755127990714146[4] = 0;
   out_2317755127990714146[5] = 0;
   out_2317755127990714146[6] = 0;
   out_2317755127990714146[7] = 0;
   out_2317755127990714146[8] = 0;
}
void h_29(double *state, double *unused, double *out_1608818058407128675) {
   out_1608818058407128675[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4031117854860265179) {
   out_4031117854860265179[0] = 0;
   out_4031117854860265179[1] = 1;
   out_4031117854860265179[2] = 0;
   out_4031117854860265179[3] = 0;
   out_4031117854860265179[4] = 0;
   out_4031117854860265179[5] = 0;
   out_4031117854860265179[6] = 0;
   out_4031117854860265179[7] = 0;
   out_4031117854860265179[8] = 0;
}
void h_28(double *state, double *unused, double *out_4908230852940097215) {
   out_4908230852940097215[0] = state[0];
}
void H_28(double *state, double *unused, double *out_9113516871929795753) {
   out_9113516871929795753[0] = 1;
   out_9113516871929795753[1] = 0;
   out_9113516871929795753[2] = 0;
   out_9113516871929795753[3] = 0;
   out_9113516871929795753[4] = 0;
   out_9113516871929795753[5] = 0;
   out_9113516871929795753[6] = 0;
   out_9113516871929795753[7] = 0;
   out_9113516871929795753[8] = 0;
}
void h_31(double *state, double *unused, double *out_1007254106532043150) {
   out_1007254106532043150[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7019350494920237926) {
   out_7019350494920237926[0] = 0;
   out_7019350494920237926[1] = 0;
   out_7019350494920237926[2] = 0;
   out_7019350494920237926[3] = 0;
   out_7019350494920237926[4] = 0;
   out_7019350494920237926[5] = 0;
   out_7019350494920237926[6] = 0;
   out_7019350494920237926[7] = 0;
   out_7019350494920237926[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2728693116701389222) {
  err_fun(nom_x, delta_x, out_2728693116701389222);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3002832702974128596) {
  inv_err_fun(nom_x, true_x, out_3002832702974128596);
}
void car_H_mod_fun(double *state, double *out_7934990902020908970) {
  H_mod_fun(state, out_7934990902020908970);
}
void car_f_fun(double *state, double dt, double *out_2890300663064821146) {
  f_fun(state,  dt, out_2890300663064821146);
}
void car_F_fun(double *state, double dt, double *out_6771328482541706301) {
  F_fun(state,  dt, out_6771328482541706301);
}
void car_h_25(double *state, double *unused, double *out_850067438180914005) {
  h_25(state, unused, out_850067438180914005);
}
void car_H_25(double *state, double *unused, double *out_7059682157681905990) {
  H_25(state, unused, out_7059682157681905990);
}
void car_h_24(double *state, double *unused, double *out_347899436057359381) {
  h_24(state, unused, out_347899436057359381);
}
void car_H_24(double *state, double *unused, double *out_3501756558879582854) {
  H_24(state, unused, out_3501756558879582854);
}
void car_h_30(double *state, double *unused, double *out_6610491326067974320) {
  h_30(state, unused, out_6610491326067974320);
}
void car_H_30(double *state, double *unused, double *out_4541349199174657363) {
  H_30(state, unused, out_4541349199174657363);
}
void car_h_26(double *state, double *unused, double *out_6834579479980731651) {
  h_26(state, unused, out_6834579479980731651);
}
void car_H_26(double *state, double *unused, double *out_7645558597153589402) {
  H_26(state, unused, out_7645558597153589402);
}
void car_h_27(double *state, double *unused, double *out_1754337882627783646) {
  h_27(state, unused, out_1754337882627783646);
}
void car_H_27(double *state, double *unused, double *out_2317755127990714146) {
  H_27(state, unused, out_2317755127990714146);
}
void car_h_29(double *state, double *unused, double *out_1608818058407128675) {
  h_29(state, unused, out_1608818058407128675);
}
void car_H_29(double *state, double *unused, double *out_4031117854860265179) {
  H_29(state, unused, out_4031117854860265179);
}
void car_h_28(double *state, double *unused, double *out_4908230852940097215) {
  h_28(state, unused, out_4908230852940097215);
}
void car_H_28(double *state, double *unused, double *out_9113516871929795753) {
  H_28(state, unused, out_9113516871929795753);
}
void car_h_31(double *state, double *unused, double *out_1007254106532043150) {
  h_31(state, unused, out_1007254106532043150);
}
void car_H_31(double *state, double *unused, double *out_7019350494920237926) {
  H_31(state, unused, out_7019350494920237926);
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
