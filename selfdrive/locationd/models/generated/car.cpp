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
void err_fun(double *nom_x, double *delta_x, double *out_8605324444285015125) {
   out_8605324444285015125[0] = delta_x[0] + nom_x[0];
   out_8605324444285015125[1] = delta_x[1] + nom_x[1];
   out_8605324444285015125[2] = delta_x[2] + nom_x[2];
   out_8605324444285015125[3] = delta_x[3] + nom_x[3];
   out_8605324444285015125[4] = delta_x[4] + nom_x[4];
   out_8605324444285015125[5] = delta_x[5] + nom_x[5];
   out_8605324444285015125[6] = delta_x[6] + nom_x[6];
   out_8605324444285015125[7] = delta_x[7] + nom_x[7];
   out_8605324444285015125[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5007064384226221423) {
   out_5007064384226221423[0] = -nom_x[0] + true_x[0];
   out_5007064384226221423[1] = -nom_x[1] + true_x[1];
   out_5007064384226221423[2] = -nom_x[2] + true_x[2];
   out_5007064384226221423[3] = -nom_x[3] + true_x[3];
   out_5007064384226221423[4] = -nom_x[4] + true_x[4];
   out_5007064384226221423[5] = -nom_x[5] + true_x[5];
   out_5007064384226221423[6] = -nom_x[6] + true_x[6];
   out_5007064384226221423[7] = -nom_x[7] + true_x[7];
   out_5007064384226221423[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4661944159559162272) {
   out_4661944159559162272[0] = 1.0;
   out_4661944159559162272[1] = 0.0;
   out_4661944159559162272[2] = 0.0;
   out_4661944159559162272[3] = 0.0;
   out_4661944159559162272[4] = 0.0;
   out_4661944159559162272[5] = 0.0;
   out_4661944159559162272[6] = 0.0;
   out_4661944159559162272[7] = 0.0;
   out_4661944159559162272[8] = 0.0;
   out_4661944159559162272[9] = 0.0;
   out_4661944159559162272[10] = 1.0;
   out_4661944159559162272[11] = 0.0;
   out_4661944159559162272[12] = 0.0;
   out_4661944159559162272[13] = 0.0;
   out_4661944159559162272[14] = 0.0;
   out_4661944159559162272[15] = 0.0;
   out_4661944159559162272[16] = 0.0;
   out_4661944159559162272[17] = 0.0;
   out_4661944159559162272[18] = 0.0;
   out_4661944159559162272[19] = 0.0;
   out_4661944159559162272[20] = 1.0;
   out_4661944159559162272[21] = 0.0;
   out_4661944159559162272[22] = 0.0;
   out_4661944159559162272[23] = 0.0;
   out_4661944159559162272[24] = 0.0;
   out_4661944159559162272[25] = 0.0;
   out_4661944159559162272[26] = 0.0;
   out_4661944159559162272[27] = 0.0;
   out_4661944159559162272[28] = 0.0;
   out_4661944159559162272[29] = 0.0;
   out_4661944159559162272[30] = 1.0;
   out_4661944159559162272[31] = 0.0;
   out_4661944159559162272[32] = 0.0;
   out_4661944159559162272[33] = 0.0;
   out_4661944159559162272[34] = 0.0;
   out_4661944159559162272[35] = 0.0;
   out_4661944159559162272[36] = 0.0;
   out_4661944159559162272[37] = 0.0;
   out_4661944159559162272[38] = 0.0;
   out_4661944159559162272[39] = 0.0;
   out_4661944159559162272[40] = 1.0;
   out_4661944159559162272[41] = 0.0;
   out_4661944159559162272[42] = 0.0;
   out_4661944159559162272[43] = 0.0;
   out_4661944159559162272[44] = 0.0;
   out_4661944159559162272[45] = 0.0;
   out_4661944159559162272[46] = 0.0;
   out_4661944159559162272[47] = 0.0;
   out_4661944159559162272[48] = 0.0;
   out_4661944159559162272[49] = 0.0;
   out_4661944159559162272[50] = 1.0;
   out_4661944159559162272[51] = 0.0;
   out_4661944159559162272[52] = 0.0;
   out_4661944159559162272[53] = 0.0;
   out_4661944159559162272[54] = 0.0;
   out_4661944159559162272[55] = 0.0;
   out_4661944159559162272[56] = 0.0;
   out_4661944159559162272[57] = 0.0;
   out_4661944159559162272[58] = 0.0;
   out_4661944159559162272[59] = 0.0;
   out_4661944159559162272[60] = 1.0;
   out_4661944159559162272[61] = 0.0;
   out_4661944159559162272[62] = 0.0;
   out_4661944159559162272[63] = 0.0;
   out_4661944159559162272[64] = 0.0;
   out_4661944159559162272[65] = 0.0;
   out_4661944159559162272[66] = 0.0;
   out_4661944159559162272[67] = 0.0;
   out_4661944159559162272[68] = 0.0;
   out_4661944159559162272[69] = 0.0;
   out_4661944159559162272[70] = 1.0;
   out_4661944159559162272[71] = 0.0;
   out_4661944159559162272[72] = 0.0;
   out_4661944159559162272[73] = 0.0;
   out_4661944159559162272[74] = 0.0;
   out_4661944159559162272[75] = 0.0;
   out_4661944159559162272[76] = 0.0;
   out_4661944159559162272[77] = 0.0;
   out_4661944159559162272[78] = 0.0;
   out_4661944159559162272[79] = 0.0;
   out_4661944159559162272[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8472733918882151860) {
   out_8472733918882151860[0] = state[0];
   out_8472733918882151860[1] = state[1];
   out_8472733918882151860[2] = state[2];
   out_8472733918882151860[3] = state[3];
   out_8472733918882151860[4] = state[4];
   out_8472733918882151860[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8472733918882151860[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8472733918882151860[7] = state[7];
   out_8472733918882151860[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9120236176575515967) {
   out_9120236176575515967[0] = 1;
   out_9120236176575515967[1] = 0;
   out_9120236176575515967[2] = 0;
   out_9120236176575515967[3] = 0;
   out_9120236176575515967[4] = 0;
   out_9120236176575515967[5] = 0;
   out_9120236176575515967[6] = 0;
   out_9120236176575515967[7] = 0;
   out_9120236176575515967[8] = 0;
   out_9120236176575515967[9] = 0;
   out_9120236176575515967[10] = 1;
   out_9120236176575515967[11] = 0;
   out_9120236176575515967[12] = 0;
   out_9120236176575515967[13] = 0;
   out_9120236176575515967[14] = 0;
   out_9120236176575515967[15] = 0;
   out_9120236176575515967[16] = 0;
   out_9120236176575515967[17] = 0;
   out_9120236176575515967[18] = 0;
   out_9120236176575515967[19] = 0;
   out_9120236176575515967[20] = 1;
   out_9120236176575515967[21] = 0;
   out_9120236176575515967[22] = 0;
   out_9120236176575515967[23] = 0;
   out_9120236176575515967[24] = 0;
   out_9120236176575515967[25] = 0;
   out_9120236176575515967[26] = 0;
   out_9120236176575515967[27] = 0;
   out_9120236176575515967[28] = 0;
   out_9120236176575515967[29] = 0;
   out_9120236176575515967[30] = 1;
   out_9120236176575515967[31] = 0;
   out_9120236176575515967[32] = 0;
   out_9120236176575515967[33] = 0;
   out_9120236176575515967[34] = 0;
   out_9120236176575515967[35] = 0;
   out_9120236176575515967[36] = 0;
   out_9120236176575515967[37] = 0;
   out_9120236176575515967[38] = 0;
   out_9120236176575515967[39] = 0;
   out_9120236176575515967[40] = 1;
   out_9120236176575515967[41] = 0;
   out_9120236176575515967[42] = 0;
   out_9120236176575515967[43] = 0;
   out_9120236176575515967[44] = 0;
   out_9120236176575515967[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9120236176575515967[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9120236176575515967[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9120236176575515967[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9120236176575515967[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9120236176575515967[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9120236176575515967[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9120236176575515967[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9120236176575515967[53] = -9.8100000000000005*dt;
   out_9120236176575515967[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9120236176575515967[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9120236176575515967[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9120236176575515967[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9120236176575515967[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9120236176575515967[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9120236176575515967[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9120236176575515967[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9120236176575515967[62] = 0;
   out_9120236176575515967[63] = 0;
   out_9120236176575515967[64] = 0;
   out_9120236176575515967[65] = 0;
   out_9120236176575515967[66] = 0;
   out_9120236176575515967[67] = 0;
   out_9120236176575515967[68] = 0;
   out_9120236176575515967[69] = 0;
   out_9120236176575515967[70] = 1;
   out_9120236176575515967[71] = 0;
   out_9120236176575515967[72] = 0;
   out_9120236176575515967[73] = 0;
   out_9120236176575515967[74] = 0;
   out_9120236176575515967[75] = 0;
   out_9120236176575515967[76] = 0;
   out_9120236176575515967[77] = 0;
   out_9120236176575515967[78] = 0;
   out_9120236176575515967[79] = 0;
   out_9120236176575515967[80] = 1;
}
void h_25(double *state, double *unused, double *out_7967351267121302732) {
   out_7967351267121302732[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4156609717358865560) {
   out_4156609717358865560[0] = 0;
   out_4156609717358865560[1] = 0;
   out_4156609717358865560[2] = 0;
   out_4156609717358865560[3] = 0;
   out_4156609717358865560[4] = 0;
   out_4156609717358865560[5] = 0;
   out_4156609717358865560[6] = 1;
   out_4156609717358865560[7] = 0;
   out_4156609717358865560[8] = 0;
}
void h_24(double *state, double *unused, double *out_8185183297069036004) {
   out_8185183297069036004[0] = state[4];
   out_8185183297069036004[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7200926927363213752) {
   out_7200926927363213752[0] = 0;
   out_7200926927363213752[1] = 0;
   out_7200926927363213752[2] = 0;
   out_7200926927363213752[3] = 0;
   out_7200926927363213752[4] = 1;
   out_7200926927363213752[5] = 0;
   out_7200926927363213752[6] = 0;
   out_7200926927363213752[7] = 0;
   out_7200926927363213752[8] = 0;
   out_7200926927363213752[9] = 0;
   out_7200926927363213752[10] = 0;
   out_7200926927363213752[11] = 0;
   out_7200926927363213752[12] = 0;
   out_7200926927363213752[13] = 0;
   out_7200926927363213752[14] = 1;
   out_7200926927363213752[15] = 0;
   out_7200926927363213752[16] = 0;
   out_7200926927363213752[17] = 0;
}
void h_30(double *state, double *unused, double *out_8242545329405808621) {
   out_8242545329405808621[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4285948664502105630) {
   out_4285948664502105630[0] = 0;
   out_4285948664502105630[1] = 0;
   out_4285948664502105630[2] = 0;
   out_4285948664502105630[3] = 0;
   out_4285948664502105630[4] = 1;
   out_4285948664502105630[5] = 0;
   out_4285948664502105630[6] = 0;
   out_4285948664502105630[7] = 0;
   out_4285948664502105630[8] = 0;
}
void h_26(double *state, double *unused, double *out_3805157226889893849) {
   out_3805157226889893849[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7898113036232921784) {
   out_7898113036232921784[0] = 0;
   out_7898113036232921784[1] = 0;
   out_7898113036232921784[2] = 0;
   out_7898113036232921784[3] = 0;
   out_7898113036232921784[4] = 0;
   out_7898113036232921784[5] = 0;
   out_7898113036232921784[6] = 0;
   out_7898113036232921784[7] = 1;
   out_7898113036232921784[8] = 0;
}
void h_27(double *state, double *unused, double *out_7297447274010470293) {
   out_7297447274010470293[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6460711976302530541) {
   out_6460711976302530541[0] = 0;
   out_6460711976302530541[1] = 0;
   out_6460711976302530541[2] = 0;
   out_6460711976302530541[3] = 1;
   out_6460711976302530541[4] = 0;
   out_6460711976302530541[5] = 0;
   out_6460711976302530541[6] = 0;
   out_6460711976302530541[7] = 0;
   out_6460711976302530541[8] = 0;
}
void h_29(double *state, double *unused, double *out_5205759277961475936) {
   out_5205759277961475936[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3775717320187713446) {
   out_3775717320187713446[0] = 0;
   out_3775717320187713446[1] = 1;
   out_3775717320187713446[2] = 0;
   out_3775717320187713446[3] = 0;
   out_3775717320187713446[4] = 0;
   out_3775717320187713446[5] = 0;
   out_3775717320187713446[6] = 0;
   out_3775717320187713446[7] = 0;
   out_3775717320187713446[8] = 0;
}
void h_28(double *state, double *unused, double *out_6291555153765214756) {
   out_6291555153765214756[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8858116337257244020) {
   out_8858116337257244020[0] = 1;
   out_8858116337257244020[1] = 0;
   out_8858116337257244020[2] = 0;
   out_8858116337257244020[3] = 0;
   out_8858116337257244020[4] = 0;
   out_8858116337257244020[5] = 0;
   out_8858116337257244020[6] = 0;
   out_8858116337257244020[7] = 0;
   out_8858116337257244020[8] = 0;
}
void h_31(double *state, double *unused, double *out_1341675521904397702) {
   out_1341675521904397702[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4125963755481905132) {
   out_4125963755481905132[0] = 0;
   out_4125963755481905132[1] = 0;
   out_4125963755481905132[2] = 0;
   out_4125963755481905132[3] = 0;
   out_4125963755481905132[4] = 0;
   out_4125963755481905132[5] = 0;
   out_4125963755481905132[6] = 0;
   out_4125963755481905132[7] = 0;
   out_4125963755481905132[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8605324444285015125) {
  err_fun(nom_x, delta_x, out_8605324444285015125);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5007064384226221423) {
  inv_err_fun(nom_x, true_x, out_5007064384226221423);
}
void car_H_mod_fun(double *state, double *out_4661944159559162272) {
  H_mod_fun(state, out_4661944159559162272);
}
void car_f_fun(double *state, double dt, double *out_8472733918882151860) {
  f_fun(state,  dt, out_8472733918882151860);
}
void car_F_fun(double *state, double dt, double *out_9120236176575515967) {
  F_fun(state,  dt, out_9120236176575515967);
}
void car_h_25(double *state, double *unused, double *out_7967351267121302732) {
  h_25(state, unused, out_7967351267121302732);
}
void car_H_25(double *state, double *unused, double *out_4156609717358865560) {
  H_25(state, unused, out_4156609717358865560);
}
void car_h_24(double *state, double *unused, double *out_8185183297069036004) {
  h_24(state, unused, out_8185183297069036004);
}
void car_H_24(double *state, double *unused, double *out_7200926927363213752) {
  H_24(state, unused, out_7200926927363213752);
}
void car_h_30(double *state, double *unused, double *out_8242545329405808621) {
  h_30(state, unused, out_8242545329405808621);
}
void car_H_30(double *state, double *unused, double *out_4285948664502105630) {
  H_30(state, unused, out_4285948664502105630);
}
void car_h_26(double *state, double *unused, double *out_3805157226889893849) {
  h_26(state, unused, out_3805157226889893849);
}
void car_H_26(double *state, double *unused, double *out_7898113036232921784) {
  H_26(state, unused, out_7898113036232921784);
}
void car_h_27(double *state, double *unused, double *out_7297447274010470293) {
  h_27(state, unused, out_7297447274010470293);
}
void car_H_27(double *state, double *unused, double *out_6460711976302530541) {
  H_27(state, unused, out_6460711976302530541);
}
void car_h_29(double *state, double *unused, double *out_5205759277961475936) {
  h_29(state, unused, out_5205759277961475936);
}
void car_H_29(double *state, double *unused, double *out_3775717320187713446) {
  H_29(state, unused, out_3775717320187713446);
}
void car_h_28(double *state, double *unused, double *out_6291555153765214756) {
  h_28(state, unused, out_6291555153765214756);
}
void car_H_28(double *state, double *unused, double *out_8858116337257244020) {
  H_28(state, unused, out_8858116337257244020);
}
void car_h_31(double *state, double *unused, double *out_1341675521904397702) {
  h_31(state, unused, out_1341675521904397702);
}
void car_H_31(double *state, double *unused, double *out_4125963755481905132) {
  H_31(state, unused, out_4125963755481905132);
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
