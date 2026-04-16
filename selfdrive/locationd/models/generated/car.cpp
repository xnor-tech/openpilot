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
void err_fun(double *nom_x, double *delta_x, double *out_2045986475089525769) {
   out_2045986475089525769[0] = delta_x[0] + nom_x[0];
   out_2045986475089525769[1] = delta_x[1] + nom_x[1];
   out_2045986475089525769[2] = delta_x[2] + nom_x[2];
   out_2045986475089525769[3] = delta_x[3] + nom_x[3];
   out_2045986475089525769[4] = delta_x[4] + nom_x[4];
   out_2045986475089525769[5] = delta_x[5] + nom_x[5];
   out_2045986475089525769[6] = delta_x[6] + nom_x[6];
   out_2045986475089525769[7] = delta_x[7] + nom_x[7];
   out_2045986475089525769[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4725156432634517583) {
   out_4725156432634517583[0] = -nom_x[0] + true_x[0];
   out_4725156432634517583[1] = -nom_x[1] + true_x[1];
   out_4725156432634517583[2] = -nom_x[2] + true_x[2];
   out_4725156432634517583[3] = -nom_x[3] + true_x[3];
   out_4725156432634517583[4] = -nom_x[4] + true_x[4];
   out_4725156432634517583[5] = -nom_x[5] + true_x[5];
   out_4725156432634517583[6] = -nom_x[6] + true_x[6];
   out_4725156432634517583[7] = -nom_x[7] + true_x[7];
   out_4725156432634517583[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3827119867446887139) {
   out_3827119867446887139[0] = 1.0;
   out_3827119867446887139[1] = 0.0;
   out_3827119867446887139[2] = 0.0;
   out_3827119867446887139[3] = 0.0;
   out_3827119867446887139[4] = 0.0;
   out_3827119867446887139[5] = 0.0;
   out_3827119867446887139[6] = 0.0;
   out_3827119867446887139[7] = 0.0;
   out_3827119867446887139[8] = 0.0;
   out_3827119867446887139[9] = 0.0;
   out_3827119867446887139[10] = 1.0;
   out_3827119867446887139[11] = 0.0;
   out_3827119867446887139[12] = 0.0;
   out_3827119867446887139[13] = 0.0;
   out_3827119867446887139[14] = 0.0;
   out_3827119867446887139[15] = 0.0;
   out_3827119867446887139[16] = 0.0;
   out_3827119867446887139[17] = 0.0;
   out_3827119867446887139[18] = 0.0;
   out_3827119867446887139[19] = 0.0;
   out_3827119867446887139[20] = 1.0;
   out_3827119867446887139[21] = 0.0;
   out_3827119867446887139[22] = 0.0;
   out_3827119867446887139[23] = 0.0;
   out_3827119867446887139[24] = 0.0;
   out_3827119867446887139[25] = 0.0;
   out_3827119867446887139[26] = 0.0;
   out_3827119867446887139[27] = 0.0;
   out_3827119867446887139[28] = 0.0;
   out_3827119867446887139[29] = 0.0;
   out_3827119867446887139[30] = 1.0;
   out_3827119867446887139[31] = 0.0;
   out_3827119867446887139[32] = 0.0;
   out_3827119867446887139[33] = 0.0;
   out_3827119867446887139[34] = 0.0;
   out_3827119867446887139[35] = 0.0;
   out_3827119867446887139[36] = 0.0;
   out_3827119867446887139[37] = 0.0;
   out_3827119867446887139[38] = 0.0;
   out_3827119867446887139[39] = 0.0;
   out_3827119867446887139[40] = 1.0;
   out_3827119867446887139[41] = 0.0;
   out_3827119867446887139[42] = 0.0;
   out_3827119867446887139[43] = 0.0;
   out_3827119867446887139[44] = 0.0;
   out_3827119867446887139[45] = 0.0;
   out_3827119867446887139[46] = 0.0;
   out_3827119867446887139[47] = 0.0;
   out_3827119867446887139[48] = 0.0;
   out_3827119867446887139[49] = 0.0;
   out_3827119867446887139[50] = 1.0;
   out_3827119867446887139[51] = 0.0;
   out_3827119867446887139[52] = 0.0;
   out_3827119867446887139[53] = 0.0;
   out_3827119867446887139[54] = 0.0;
   out_3827119867446887139[55] = 0.0;
   out_3827119867446887139[56] = 0.0;
   out_3827119867446887139[57] = 0.0;
   out_3827119867446887139[58] = 0.0;
   out_3827119867446887139[59] = 0.0;
   out_3827119867446887139[60] = 1.0;
   out_3827119867446887139[61] = 0.0;
   out_3827119867446887139[62] = 0.0;
   out_3827119867446887139[63] = 0.0;
   out_3827119867446887139[64] = 0.0;
   out_3827119867446887139[65] = 0.0;
   out_3827119867446887139[66] = 0.0;
   out_3827119867446887139[67] = 0.0;
   out_3827119867446887139[68] = 0.0;
   out_3827119867446887139[69] = 0.0;
   out_3827119867446887139[70] = 1.0;
   out_3827119867446887139[71] = 0.0;
   out_3827119867446887139[72] = 0.0;
   out_3827119867446887139[73] = 0.0;
   out_3827119867446887139[74] = 0.0;
   out_3827119867446887139[75] = 0.0;
   out_3827119867446887139[76] = 0.0;
   out_3827119867446887139[77] = 0.0;
   out_3827119867446887139[78] = 0.0;
   out_3827119867446887139[79] = 0.0;
   out_3827119867446887139[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_864076176166956704) {
   out_864076176166956704[0] = state[0];
   out_864076176166956704[1] = state[1];
   out_864076176166956704[2] = state[2];
   out_864076176166956704[3] = state[3];
   out_864076176166956704[4] = state[4];
   out_864076176166956704[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_864076176166956704[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_864076176166956704[7] = state[7];
   out_864076176166956704[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6878955300494497585) {
   out_6878955300494497585[0] = 1;
   out_6878955300494497585[1] = 0;
   out_6878955300494497585[2] = 0;
   out_6878955300494497585[3] = 0;
   out_6878955300494497585[4] = 0;
   out_6878955300494497585[5] = 0;
   out_6878955300494497585[6] = 0;
   out_6878955300494497585[7] = 0;
   out_6878955300494497585[8] = 0;
   out_6878955300494497585[9] = 0;
   out_6878955300494497585[10] = 1;
   out_6878955300494497585[11] = 0;
   out_6878955300494497585[12] = 0;
   out_6878955300494497585[13] = 0;
   out_6878955300494497585[14] = 0;
   out_6878955300494497585[15] = 0;
   out_6878955300494497585[16] = 0;
   out_6878955300494497585[17] = 0;
   out_6878955300494497585[18] = 0;
   out_6878955300494497585[19] = 0;
   out_6878955300494497585[20] = 1;
   out_6878955300494497585[21] = 0;
   out_6878955300494497585[22] = 0;
   out_6878955300494497585[23] = 0;
   out_6878955300494497585[24] = 0;
   out_6878955300494497585[25] = 0;
   out_6878955300494497585[26] = 0;
   out_6878955300494497585[27] = 0;
   out_6878955300494497585[28] = 0;
   out_6878955300494497585[29] = 0;
   out_6878955300494497585[30] = 1;
   out_6878955300494497585[31] = 0;
   out_6878955300494497585[32] = 0;
   out_6878955300494497585[33] = 0;
   out_6878955300494497585[34] = 0;
   out_6878955300494497585[35] = 0;
   out_6878955300494497585[36] = 0;
   out_6878955300494497585[37] = 0;
   out_6878955300494497585[38] = 0;
   out_6878955300494497585[39] = 0;
   out_6878955300494497585[40] = 1;
   out_6878955300494497585[41] = 0;
   out_6878955300494497585[42] = 0;
   out_6878955300494497585[43] = 0;
   out_6878955300494497585[44] = 0;
   out_6878955300494497585[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6878955300494497585[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6878955300494497585[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6878955300494497585[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6878955300494497585[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6878955300494497585[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6878955300494497585[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6878955300494497585[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6878955300494497585[53] = -9.8100000000000005*dt;
   out_6878955300494497585[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6878955300494497585[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6878955300494497585[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6878955300494497585[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6878955300494497585[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6878955300494497585[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6878955300494497585[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6878955300494497585[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6878955300494497585[62] = 0;
   out_6878955300494497585[63] = 0;
   out_6878955300494497585[64] = 0;
   out_6878955300494497585[65] = 0;
   out_6878955300494497585[66] = 0;
   out_6878955300494497585[67] = 0;
   out_6878955300494497585[68] = 0;
   out_6878955300494497585[69] = 0;
   out_6878955300494497585[70] = 1;
   out_6878955300494497585[71] = 0;
   out_6878955300494497585[72] = 0;
   out_6878955300494497585[73] = 0;
   out_6878955300494497585[74] = 0;
   out_6878955300494497585[75] = 0;
   out_6878955300494497585[76] = 0;
   out_6878955300494497585[77] = 0;
   out_6878955300494497585[78] = 0;
   out_6878955300494497585[79] = 0;
   out_6878955300494497585[80] = 1;
}
void h_25(double *state, double *unused, double *out_983273569991112921) {
   out_983273569991112921[0] = state[6];
}
void H_25(double *state, double *unused, double *out_967710975480981970) {
   out_967710975480981970[0] = 0;
   out_967710975480981970[1] = 0;
   out_967710975480981970[2] = 0;
   out_967710975480981970[3] = 0;
   out_967710975480981970[4] = 0;
   out_967710975480981970[5] = 0;
   out_967710975480981970[6] = 1;
   out_967710975480981970[7] = 0;
   out_967710975480981970[8] = 0;
}
void h_24(double *state, double *unused, double *out_5323618623202364012) {
   out_5323618623202364012[0] = state[4];
   out_5323618623202364012[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3140360574486481536) {
   out_3140360574486481536[0] = 0;
   out_3140360574486481536[1] = 0;
   out_3140360574486481536[2] = 0;
   out_3140360574486481536[3] = 0;
   out_3140360574486481536[4] = 1;
   out_3140360574486481536[5] = 0;
   out_3140360574486481536[6] = 0;
   out_3140360574486481536[7] = 0;
   out_3140360574486481536[8] = 0;
   out_3140360574486481536[9] = 0;
   out_3140360574486481536[10] = 0;
   out_3140360574486481536[11] = 0;
   out_3140360574486481536[12] = 0;
   out_3140360574486481536[13] = 0;
   out_3140360574486481536[14] = 1;
   out_3140360574486481536[15] = 0;
   out_3140360574486481536[16] = 0;
   out_3140360574486481536[17] = 0;
}
void h_30(double *state, double *unused, double *out_3552231351687777287) {
   out_3552231351687777287[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1550621983026266657) {
   out_1550621983026266657[0] = 0;
   out_1550621983026266657[1] = 0;
   out_1550621983026266657[2] = 0;
   out_1550621983026266657[3] = 0;
   out_1550621983026266657[4] = 1;
   out_1550621983026266657[5] = 0;
   out_1550621983026266657[6] = 0;
   out_1550621983026266657[7] = 0;
   out_1550621983026266657[8] = 0;
}
void h_26(double *state, double *unused, double *out_2948915467011299770) {
   out_2948915467011299770[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2336814994279818631) {
   out_2336814994279818631[0] = 0;
   out_2336814994279818631[1] = 0;
   out_2336814994279818631[2] = 0;
   out_2336814994279818631[3] = 0;
   out_2336814994279818631[4] = 0;
   out_2336814994279818631[5] = 0;
   out_2336814994279818631[6] = 0;
   out_2336814994279818631[7] = 1;
   out_2336814994279818631[8] = 0;
}
void h_27(double *state, double *unused, double *out_2333527436501608963) {
   out_2333527436501608963[0] = state[3];
}
void H_27(double *state, double *unused, double *out_624141328774158254) {
   out_624141328774158254[0] = 0;
   out_624141328774158254[1] = 0;
   out_624141328774158254[2] = 0;
   out_624141328774158254[3] = 1;
   out_624141328774158254[4] = 0;
   out_624141328774158254[5] = 0;
   out_624141328774158254[6] = 0;
   out_624141328774158254[7] = 0;
   out_624141328774158254[8] = 0;
}
void h_29(double *state, double *unused, double *out_5873969309581297887) {
   out_5873969309581297887[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2060853327340658841) {
   out_2060853327340658841[0] = 0;
   out_2060853327340658841[1] = 1;
   out_2060853327340658841[2] = 0;
   out_2060853327340658841[3] = 0;
   out_2060853327340658841[4] = 0;
   out_2060853327340658841[5] = 0;
   out_2060853327340658841[6] = 0;
   out_2060853327340658841[7] = 0;
   out_2060853327340658841[8] = 0;
}
void h_28(double *state, double *unused, double *out_270877884307088688) {
   out_270877884307088688[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3021545689728871733) {
   out_3021545689728871733[0] = 1;
   out_3021545689728871733[1] = 0;
   out_3021545689728871733[2] = 0;
   out_3021545689728871733[3] = 0;
   out_3021545689728871733[4] = 0;
   out_3021545689728871733[5] = 0;
   out_3021545689728871733[6] = 0;
   out_3021545689728871733[7] = 0;
   out_3021545689728871733[8] = 0;
}
void h_31(double *state, double *unused, double *out_2878409906437388731) {
   out_2878409906437388731[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5335422396588389670) {
   out_5335422396588389670[0] = 0;
   out_5335422396588389670[1] = 0;
   out_5335422396588389670[2] = 0;
   out_5335422396588389670[3] = 0;
   out_5335422396588389670[4] = 0;
   out_5335422396588389670[5] = 0;
   out_5335422396588389670[6] = 0;
   out_5335422396588389670[7] = 0;
   out_5335422396588389670[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2045986475089525769) {
  err_fun(nom_x, delta_x, out_2045986475089525769);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4725156432634517583) {
  inv_err_fun(nom_x, true_x, out_4725156432634517583);
}
void car_H_mod_fun(double *state, double *out_3827119867446887139) {
  H_mod_fun(state, out_3827119867446887139);
}
void car_f_fun(double *state, double dt, double *out_864076176166956704) {
  f_fun(state,  dt, out_864076176166956704);
}
void car_F_fun(double *state, double dt, double *out_6878955300494497585) {
  F_fun(state,  dt, out_6878955300494497585);
}
void car_h_25(double *state, double *unused, double *out_983273569991112921) {
  h_25(state, unused, out_983273569991112921);
}
void car_H_25(double *state, double *unused, double *out_967710975480981970) {
  H_25(state, unused, out_967710975480981970);
}
void car_h_24(double *state, double *unused, double *out_5323618623202364012) {
  h_24(state, unused, out_5323618623202364012);
}
void car_H_24(double *state, double *unused, double *out_3140360574486481536) {
  H_24(state, unused, out_3140360574486481536);
}
void car_h_30(double *state, double *unused, double *out_3552231351687777287) {
  h_30(state, unused, out_3552231351687777287);
}
void car_H_30(double *state, double *unused, double *out_1550621983026266657) {
  H_30(state, unused, out_1550621983026266657);
}
void car_h_26(double *state, double *unused, double *out_2948915467011299770) {
  h_26(state, unused, out_2948915467011299770);
}
void car_H_26(double *state, double *unused, double *out_2336814994279818631) {
  H_26(state, unused, out_2336814994279818631);
}
void car_h_27(double *state, double *unused, double *out_2333527436501608963) {
  h_27(state, unused, out_2333527436501608963);
}
void car_H_27(double *state, double *unused, double *out_624141328774158254) {
  H_27(state, unused, out_624141328774158254);
}
void car_h_29(double *state, double *unused, double *out_5873969309581297887) {
  h_29(state, unused, out_5873969309581297887);
}
void car_H_29(double *state, double *unused, double *out_2060853327340658841) {
  H_29(state, unused, out_2060853327340658841);
}
void car_h_28(double *state, double *unused, double *out_270877884307088688) {
  h_28(state, unused, out_270877884307088688);
}
void car_H_28(double *state, double *unused, double *out_3021545689728871733) {
  H_28(state, unused, out_3021545689728871733);
}
void car_h_31(double *state, double *unused, double *out_2878409906437388731) {
  h_31(state, unused, out_2878409906437388731);
}
void car_H_31(double *state, double *unused, double *out_5335422396588389670) {
  H_31(state, unused, out_5335422396588389670);
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
