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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2724251513109582597) {
   out_2724251513109582597[0] = delta_x[0] + nom_x[0];
   out_2724251513109582597[1] = delta_x[1] + nom_x[1];
   out_2724251513109582597[2] = delta_x[2] + nom_x[2];
   out_2724251513109582597[3] = delta_x[3] + nom_x[3];
   out_2724251513109582597[4] = delta_x[4] + nom_x[4];
   out_2724251513109582597[5] = delta_x[5] + nom_x[5];
   out_2724251513109582597[6] = delta_x[6] + nom_x[6];
   out_2724251513109582597[7] = delta_x[7] + nom_x[7];
   out_2724251513109582597[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8376452329691755560) {
   out_8376452329691755560[0] = -nom_x[0] + true_x[0];
   out_8376452329691755560[1] = -nom_x[1] + true_x[1];
   out_8376452329691755560[2] = -nom_x[2] + true_x[2];
   out_8376452329691755560[3] = -nom_x[3] + true_x[3];
   out_8376452329691755560[4] = -nom_x[4] + true_x[4];
   out_8376452329691755560[5] = -nom_x[5] + true_x[5];
   out_8376452329691755560[6] = -nom_x[6] + true_x[6];
   out_8376452329691755560[7] = -nom_x[7] + true_x[7];
   out_8376452329691755560[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5184655659434066940) {
   out_5184655659434066940[0] = 1.0;
   out_5184655659434066940[1] = 0;
   out_5184655659434066940[2] = 0;
   out_5184655659434066940[3] = 0;
   out_5184655659434066940[4] = 0;
   out_5184655659434066940[5] = 0;
   out_5184655659434066940[6] = 0;
   out_5184655659434066940[7] = 0;
   out_5184655659434066940[8] = 0;
   out_5184655659434066940[9] = 0;
   out_5184655659434066940[10] = 1.0;
   out_5184655659434066940[11] = 0;
   out_5184655659434066940[12] = 0;
   out_5184655659434066940[13] = 0;
   out_5184655659434066940[14] = 0;
   out_5184655659434066940[15] = 0;
   out_5184655659434066940[16] = 0;
   out_5184655659434066940[17] = 0;
   out_5184655659434066940[18] = 0;
   out_5184655659434066940[19] = 0;
   out_5184655659434066940[20] = 1.0;
   out_5184655659434066940[21] = 0;
   out_5184655659434066940[22] = 0;
   out_5184655659434066940[23] = 0;
   out_5184655659434066940[24] = 0;
   out_5184655659434066940[25] = 0;
   out_5184655659434066940[26] = 0;
   out_5184655659434066940[27] = 0;
   out_5184655659434066940[28] = 0;
   out_5184655659434066940[29] = 0;
   out_5184655659434066940[30] = 1.0;
   out_5184655659434066940[31] = 0;
   out_5184655659434066940[32] = 0;
   out_5184655659434066940[33] = 0;
   out_5184655659434066940[34] = 0;
   out_5184655659434066940[35] = 0;
   out_5184655659434066940[36] = 0;
   out_5184655659434066940[37] = 0;
   out_5184655659434066940[38] = 0;
   out_5184655659434066940[39] = 0;
   out_5184655659434066940[40] = 1.0;
   out_5184655659434066940[41] = 0;
   out_5184655659434066940[42] = 0;
   out_5184655659434066940[43] = 0;
   out_5184655659434066940[44] = 0;
   out_5184655659434066940[45] = 0;
   out_5184655659434066940[46] = 0;
   out_5184655659434066940[47] = 0;
   out_5184655659434066940[48] = 0;
   out_5184655659434066940[49] = 0;
   out_5184655659434066940[50] = 1.0;
   out_5184655659434066940[51] = 0;
   out_5184655659434066940[52] = 0;
   out_5184655659434066940[53] = 0;
   out_5184655659434066940[54] = 0;
   out_5184655659434066940[55] = 0;
   out_5184655659434066940[56] = 0;
   out_5184655659434066940[57] = 0;
   out_5184655659434066940[58] = 0;
   out_5184655659434066940[59] = 0;
   out_5184655659434066940[60] = 1.0;
   out_5184655659434066940[61] = 0;
   out_5184655659434066940[62] = 0;
   out_5184655659434066940[63] = 0;
   out_5184655659434066940[64] = 0;
   out_5184655659434066940[65] = 0;
   out_5184655659434066940[66] = 0;
   out_5184655659434066940[67] = 0;
   out_5184655659434066940[68] = 0;
   out_5184655659434066940[69] = 0;
   out_5184655659434066940[70] = 1.0;
   out_5184655659434066940[71] = 0;
   out_5184655659434066940[72] = 0;
   out_5184655659434066940[73] = 0;
   out_5184655659434066940[74] = 0;
   out_5184655659434066940[75] = 0;
   out_5184655659434066940[76] = 0;
   out_5184655659434066940[77] = 0;
   out_5184655659434066940[78] = 0;
   out_5184655659434066940[79] = 0;
   out_5184655659434066940[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2131619432792421671) {
   out_2131619432792421671[0] = state[0];
   out_2131619432792421671[1] = state[1];
   out_2131619432792421671[2] = state[2];
   out_2131619432792421671[3] = state[3];
   out_2131619432792421671[4] = state[4];
   out_2131619432792421671[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2131619432792421671[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2131619432792421671[7] = state[7];
   out_2131619432792421671[8] = state[8];
}
void F_fun(double *state, double dt, double *out_600621081321688166) {
   out_600621081321688166[0] = 1;
   out_600621081321688166[1] = 0;
   out_600621081321688166[2] = 0;
   out_600621081321688166[3] = 0;
   out_600621081321688166[4] = 0;
   out_600621081321688166[5] = 0;
   out_600621081321688166[6] = 0;
   out_600621081321688166[7] = 0;
   out_600621081321688166[8] = 0;
   out_600621081321688166[9] = 0;
   out_600621081321688166[10] = 1;
   out_600621081321688166[11] = 0;
   out_600621081321688166[12] = 0;
   out_600621081321688166[13] = 0;
   out_600621081321688166[14] = 0;
   out_600621081321688166[15] = 0;
   out_600621081321688166[16] = 0;
   out_600621081321688166[17] = 0;
   out_600621081321688166[18] = 0;
   out_600621081321688166[19] = 0;
   out_600621081321688166[20] = 1;
   out_600621081321688166[21] = 0;
   out_600621081321688166[22] = 0;
   out_600621081321688166[23] = 0;
   out_600621081321688166[24] = 0;
   out_600621081321688166[25] = 0;
   out_600621081321688166[26] = 0;
   out_600621081321688166[27] = 0;
   out_600621081321688166[28] = 0;
   out_600621081321688166[29] = 0;
   out_600621081321688166[30] = 1;
   out_600621081321688166[31] = 0;
   out_600621081321688166[32] = 0;
   out_600621081321688166[33] = 0;
   out_600621081321688166[34] = 0;
   out_600621081321688166[35] = 0;
   out_600621081321688166[36] = 0;
   out_600621081321688166[37] = 0;
   out_600621081321688166[38] = 0;
   out_600621081321688166[39] = 0;
   out_600621081321688166[40] = 1;
   out_600621081321688166[41] = 0;
   out_600621081321688166[42] = 0;
   out_600621081321688166[43] = 0;
   out_600621081321688166[44] = 0;
   out_600621081321688166[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_600621081321688166[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_600621081321688166[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_600621081321688166[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_600621081321688166[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_600621081321688166[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_600621081321688166[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_600621081321688166[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_600621081321688166[53] = -9.8000000000000007*dt;
   out_600621081321688166[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_600621081321688166[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_600621081321688166[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_600621081321688166[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_600621081321688166[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_600621081321688166[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_600621081321688166[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_600621081321688166[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_600621081321688166[62] = 0;
   out_600621081321688166[63] = 0;
   out_600621081321688166[64] = 0;
   out_600621081321688166[65] = 0;
   out_600621081321688166[66] = 0;
   out_600621081321688166[67] = 0;
   out_600621081321688166[68] = 0;
   out_600621081321688166[69] = 0;
   out_600621081321688166[70] = 1;
   out_600621081321688166[71] = 0;
   out_600621081321688166[72] = 0;
   out_600621081321688166[73] = 0;
   out_600621081321688166[74] = 0;
   out_600621081321688166[75] = 0;
   out_600621081321688166[76] = 0;
   out_600621081321688166[77] = 0;
   out_600621081321688166[78] = 0;
   out_600621081321688166[79] = 0;
   out_600621081321688166[80] = 1;
}
void h_25(double *state, double *unused, double *out_7067729207585171189) {
   out_7067729207585171189[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2870604109442777267) {
   out_2870604109442777267[0] = 0;
   out_2870604109442777267[1] = 0;
   out_2870604109442777267[2] = 0;
   out_2870604109442777267[3] = 0;
   out_2870604109442777267[4] = 0;
   out_2870604109442777267[5] = 0;
   out_2870604109442777267[6] = 1;
   out_2870604109442777267[7] = 0;
   out_2870604109442777267[8] = 0;
}
void h_24(double *state, double *unused, double *out_3796831930133441469) {
   out_3796831930133441469[0] = state[4];
   out_3796831930133441469[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6428529708245100403) {
   out_6428529708245100403[0] = 0;
   out_6428529708245100403[1] = 0;
   out_6428529708245100403[2] = 0;
   out_6428529708245100403[3] = 0;
   out_6428529708245100403[4] = 1;
   out_6428529708245100403[5] = 0;
   out_6428529708245100403[6] = 0;
   out_6428529708245100403[7] = 0;
   out_6428529708245100403[8] = 0;
   out_6428529708245100403[9] = 0;
   out_6428529708245100403[10] = 0;
   out_6428529708245100403[11] = 0;
   out_6428529708245100403[12] = 0;
   out_6428529708245100403[13] = 0;
   out_6428529708245100403[14] = 1;
   out_6428529708245100403[15] = 0;
   out_6428529708245100403[16] = 0;
   out_6428529708245100403[17] = 0;
}
void h_30(double *state, double *unused, double *out_479901281108876026) {
   out_479901281108876026[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1657092220684830931) {
   out_1657092220684830931[0] = 0;
   out_1657092220684830931[1] = 0;
   out_1657092220684830931[2] = 0;
   out_1657092220684830931[3] = 0;
   out_1657092220684830931[4] = 1;
   out_1657092220684830931[5] = 0;
   out_1657092220684830931[6] = 0;
   out_1657092220684830931[7] = 0;
   out_1657092220684830931[8] = 0;
}
void h_26(double *state, double *unused, double *out_6522861422684189753) {
   out_6522861422684189753[0] = state[7];
}
void H_26(double *state, double *unused, double *out_870899209431278957) {
   out_870899209431278957[0] = 0;
   out_870899209431278957[1] = 0;
   out_870899209431278957[2] = 0;
   out_870899209431278957[3] = 0;
   out_870899209431278957[4] = 0;
   out_870899209431278957[5] = 0;
   out_870899209431278957[6] = 0;
   out_870899209431278957[7] = 1;
   out_870899209431278957[8] = 0;
}
void h_27(double *state, double *unused, double *out_1424999336504214354) {
   out_1424999336504214354[0] = state[3];
}
void H_27(double *state, double *unused, double *out_566501850499112286) {
   out_566501850499112286[0] = 0;
   out_566501850499112286[1] = 0;
   out_566501850499112286[2] = 0;
   out_566501850499112286[3] = 1;
   out_566501850499112286[4] = 0;
   out_566501850499112286[5] = 0;
   out_566501850499112286[6] = 0;
   out_566501850499112286[7] = 0;
   out_566501850499112286[8] = 0;
}
void h_29(double *state, double *unused, double *out_5766549776351297442) {
   out_5766549776351297442[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1146860876370438747) {
   out_1146860876370438747[0] = 0;
   out_1146860876370438747[1] = 1;
   out_1146860876370438747[2] = 0;
   out_1146860876370438747[3] = 0;
   out_1146860876370438747[4] = 0;
   out_1146860876370438747[5] = 0;
   out_1146860876370438747[6] = 0;
   out_1146860876370438747[7] = 0;
   out_1146860876370438747[8] = 0;
}
void h_28(double *state, double *unused, double *out_7407923175810356107) {
   out_7407923175810356107[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6229259893439969321) {
   out_6229259893439969321[0] = 1;
   out_6229259893439969321[1] = 0;
   out_6229259893439969321[2] = 0;
   out_6229259893439969321[3] = 0;
   out_6229259893439969321[4] = 0;
   out_6229259893439969321[5] = 0;
   out_6229259893439969321[6] = 0;
   out_6229259893439969321[7] = 0;
   out_6229259893439969321[8] = 0;
}
void h_31(double *state, double *unused, double *out_8382621941290469267) {
   out_8382621941290469267[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1497107311664630433) {
   out_1497107311664630433[0] = 0;
   out_1497107311664630433[1] = 0;
   out_1497107311664630433[2] = 0;
   out_1497107311664630433[3] = 0;
   out_1497107311664630433[4] = 0;
   out_1497107311664630433[5] = 0;
   out_1497107311664630433[6] = 0;
   out_1497107311664630433[7] = 0;
   out_1497107311664630433[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2724251513109582597) {
  err_fun(nom_x, delta_x, out_2724251513109582597);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8376452329691755560) {
  inv_err_fun(nom_x, true_x, out_8376452329691755560);
}
void car_H_mod_fun(double *state, double *out_5184655659434066940) {
  H_mod_fun(state, out_5184655659434066940);
}
void car_f_fun(double *state, double dt, double *out_2131619432792421671) {
  f_fun(state,  dt, out_2131619432792421671);
}
void car_F_fun(double *state, double dt, double *out_600621081321688166) {
  F_fun(state,  dt, out_600621081321688166);
}
void car_h_25(double *state, double *unused, double *out_7067729207585171189) {
  h_25(state, unused, out_7067729207585171189);
}
void car_H_25(double *state, double *unused, double *out_2870604109442777267) {
  H_25(state, unused, out_2870604109442777267);
}
void car_h_24(double *state, double *unused, double *out_3796831930133441469) {
  h_24(state, unused, out_3796831930133441469);
}
void car_H_24(double *state, double *unused, double *out_6428529708245100403) {
  H_24(state, unused, out_6428529708245100403);
}
void car_h_30(double *state, double *unused, double *out_479901281108876026) {
  h_30(state, unused, out_479901281108876026);
}
void car_H_30(double *state, double *unused, double *out_1657092220684830931) {
  H_30(state, unused, out_1657092220684830931);
}
void car_h_26(double *state, double *unused, double *out_6522861422684189753) {
  h_26(state, unused, out_6522861422684189753);
}
void car_H_26(double *state, double *unused, double *out_870899209431278957) {
  H_26(state, unused, out_870899209431278957);
}
void car_h_27(double *state, double *unused, double *out_1424999336504214354) {
  h_27(state, unused, out_1424999336504214354);
}
void car_H_27(double *state, double *unused, double *out_566501850499112286) {
  H_27(state, unused, out_566501850499112286);
}
void car_h_29(double *state, double *unused, double *out_5766549776351297442) {
  h_29(state, unused, out_5766549776351297442);
}
void car_H_29(double *state, double *unused, double *out_1146860876370438747) {
  H_29(state, unused, out_1146860876370438747);
}
void car_h_28(double *state, double *unused, double *out_7407923175810356107) {
  h_28(state, unused, out_7407923175810356107);
}
void car_H_28(double *state, double *unused, double *out_6229259893439969321) {
  H_28(state, unused, out_6229259893439969321);
}
void car_h_31(double *state, double *unused, double *out_8382621941290469267) {
  h_31(state, unused, out_8382621941290469267);
}
void car_H_31(double *state, double *unused, double *out_1497107311664630433) {
  H_31(state, unused, out_1497107311664630433);
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
