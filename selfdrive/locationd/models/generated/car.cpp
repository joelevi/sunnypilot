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
void err_fun(double *nom_x, double *delta_x, double *out_161151116830974420) {
   out_161151116830974420[0] = delta_x[0] + nom_x[0];
   out_161151116830974420[1] = delta_x[1] + nom_x[1];
   out_161151116830974420[2] = delta_x[2] + nom_x[2];
   out_161151116830974420[3] = delta_x[3] + nom_x[3];
   out_161151116830974420[4] = delta_x[4] + nom_x[4];
   out_161151116830974420[5] = delta_x[5] + nom_x[5];
   out_161151116830974420[6] = delta_x[6] + nom_x[6];
   out_161151116830974420[7] = delta_x[7] + nom_x[7];
   out_161151116830974420[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_868195081031098241) {
   out_868195081031098241[0] = -nom_x[0] + true_x[0];
   out_868195081031098241[1] = -nom_x[1] + true_x[1];
   out_868195081031098241[2] = -nom_x[2] + true_x[2];
   out_868195081031098241[3] = -nom_x[3] + true_x[3];
   out_868195081031098241[4] = -nom_x[4] + true_x[4];
   out_868195081031098241[5] = -nom_x[5] + true_x[5];
   out_868195081031098241[6] = -nom_x[6] + true_x[6];
   out_868195081031098241[7] = -nom_x[7] + true_x[7];
   out_868195081031098241[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_9075143253330918196) {
   out_9075143253330918196[0] = 1.0;
   out_9075143253330918196[1] = 0.0;
   out_9075143253330918196[2] = 0.0;
   out_9075143253330918196[3] = 0.0;
   out_9075143253330918196[4] = 0.0;
   out_9075143253330918196[5] = 0.0;
   out_9075143253330918196[6] = 0.0;
   out_9075143253330918196[7] = 0.0;
   out_9075143253330918196[8] = 0.0;
   out_9075143253330918196[9] = 0.0;
   out_9075143253330918196[10] = 1.0;
   out_9075143253330918196[11] = 0.0;
   out_9075143253330918196[12] = 0.0;
   out_9075143253330918196[13] = 0.0;
   out_9075143253330918196[14] = 0.0;
   out_9075143253330918196[15] = 0.0;
   out_9075143253330918196[16] = 0.0;
   out_9075143253330918196[17] = 0.0;
   out_9075143253330918196[18] = 0.0;
   out_9075143253330918196[19] = 0.0;
   out_9075143253330918196[20] = 1.0;
   out_9075143253330918196[21] = 0.0;
   out_9075143253330918196[22] = 0.0;
   out_9075143253330918196[23] = 0.0;
   out_9075143253330918196[24] = 0.0;
   out_9075143253330918196[25] = 0.0;
   out_9075143253330918196[26] = 0.0;
   out_9075143253330918196[27] = 0.0;
   out_9075143253330918196[28] = 0.0;
   out_9075143253330918196[29] = 0.0;
   out_9075143253330918196[30] = 1.0;
   out_9075143253330918196[31] = 0.0;
   out_9075143253330918196[32] = 0.0;
   out_9075143253330918196[33] = 0.0;
   out_9075143253330918196[34] = 0.0;
   out_9075143253330918196[35] = 0.0;
   out_9075143253330918196[36] = 0.0;
   out_9075143253330918196[37] = 0.0;
   out_9075143253330918196[38] = 0.0;
   out_9075143253330918196[39] = 0.0;
   out_9075143253330918196[40] = 1.0;
   out_9075143253330918196[41] = 0.0;
   out_9075143253330918196[42] = 0.0;
   out_9075143253330918196[43] = 0.0;
   out_9075143253330918196[44] = 0.0;
   out_9075143253330918196[45] = 0.0;
   out_9075143253330918196[46] = 0.0;
   out_9075143253330918196[47] = 0.0;
   out_9075143253330918196[48] = 0.0;
   out_9075143253330918196[49] = 0.0;
   out_9075143253330918196[50] = 1.0;
   out_9075143253330918196[51] = 0.0;
   out_9075143253330918196[52] = 0.0;
   out_9075143253330918196[53] = 0.0;
   out_9075143253330918196[54] = 0.0;
   out_9075143253330918196[55] = 0.0;
   out_9075143253330918196[56] = 0.0;
   out_9075143253330918196[57] = 0.0;
   out_9075143253330918196[58] = 0.0;
   out_9075143253330918196[59] = 0.0;
   out_9075143253330918196[60] = 1.0;
   out_9075143253330918196[61] = 0.0;
   out_9075143253330918196[62] = 0.0;
   out_9075143253330918196[63] = 0.0;
   out_9075143253330918196[64] = 0.0;
   out_9075143253330918196[65] = 0.0;
   out_9075143253330918196[66] = 0.0;
   out_9075143253330918196[67] = 0.0;
   out_9075143253330918196[68] = 0.0;
   out_9075143253330918196[69] = 0.0;
   out_9075143253330918196[70] = 1.0;
   out_9075143253330918196[71] = 0.0;
   out_9075143253330918196[72] = 0.0;
   out_9075143253330918196[73] = 0.0;
   out_9075143253330918196[74] = 0.0;
   out_9075143253330918196[75] = 0.0;
   out_9075143253330918196[76] = 0.0;
   out_9075143253330918196[77] = 0.0;
   out_9075143253330918196[78] = 0.0;
   out_9075143253330918196[79] = 0.0;
   out_9075143253330918196[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4562342941996871353) {
   out_4562342941996871353[0] = state[0];
   out_4562342941996871353[1] = state[1];
   out_4562342941996871353[2] = state[2];
   out_4562342941996871353[3] = state[3];
   out_4562342941996871353[4] = state[4];
   out_4562342941996871353[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4562342941996871353[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4562342941996871353[7] = state[7];
   out_4562342941996871353[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5331884570962474814) {
   out_5331884570962474814[0] = 1;
   out_5331884570962474814[1] = 0;
   out_5331884570962474814[2] = 0;
   out_5331884570962474814[3] = 0;
   out_5331884570962474814[4] = 0;
   out_5331884570962474814[5] = 0;
   out_5331884570962474814[6] = 0;
   out_5331884570962474814[7] = 0;
   out_5331884570962474814[8] = 0;
   out_5331884570962474814[9] = 0;
   out_5331884570962474814[10] = 1;
   out_5331884570962474814[11] = 0;
   out_5331884570962474814[12] = 0;
   out_5331884570962474814[13] = 0;
   out_5331884570962474814[14] = 0;
   out_5331884570962474814[15] = 0;
   out_5331884570962474814[16] = 0;
   out_5331884570962474814[17] = 0;
   out_5331884570962474814[18] = 0;
   out_5331884570962474814[19] = 0;
   out_5331884570962474814[20] = 1;
   out_5331884570962474814[21] = 0;
   out_5331884570962474814[22] = 0;
   out_5331884570962474814[23] = 0;
   out_5331884570962474814[24] = 0;
   out_5331884570962474814[25] = 0;
   out_5331884570962474814[26] = 0;
   out_5331884570962474814[27] = 0;
   out_5331884570962474814[28] = 0;
   out_5331884570962474814[29] = 0;
   out_5331884570962474814[30] = 1;
   out_5331884570962474814[31] = 0;
   out_5331884570962474814[32] = 0;
   out_5331884570962474814[33] = 0;
   out_5331884570962474814[34] = 0;
   out_5331884570962474814[35] = 0;
   out_5331884570962474814[36] = 0;
   out_5331884570962474814[37] = 0;
   out_5331884570962474814[38] = 0;
   out_5331884570962474814[39] = 0;
   out_5331884570962474814[40] = 1;
   out_5331884570962474814[41] = 0;
   out_5331884570962474814[42] = 0;
   out_5331884570962474814[43] = 0;
   out_5331884570962474814[44] = 0;
   out_5331884570962474814[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5331884570962474814[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5331884570962474814[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5331884570962474814[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5331884570962474814[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5331884570962474814[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5331884570962474814[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5331884570962474814[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5331884570962474814[53] = -9.8100000000000005*dt;
   out_5331884570962474814[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5331884570962474814[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5331884570962474814[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5331884570962474814[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5331884570962474814[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5331884570962474814[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5331884570962474814[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5331884570962474814[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5331884570962474814[62] = 0;
   out_5331884570962474814[63] = 0;
   out_5331884570962474814[64] = 0;
   out_5331884570962474814[65] = 0;
   out_5331884570962474814[66] = 0;
   out_5331884570962474814[67] = 0;
   out_5331884570962474814[68] = 0;
   out_5331884570962474814[69] = 0;
   out_5331884570962474814[70] = 1;
   out_5331884570962474814[71] = 0;
   out_5331884570962474814[72] = 0;
   out_5331884570962474814[73] = 0;
   out_5331884570962474814[74] = 0;
   out_5331884570962474814[75] = 0;
   out_5331884570962474814[76] = 0;
   out_5331884570962474814[77] = 0;
   out_5331884570962474814[78] = 0;
   out_5331884570962474814[79] = 0;
   out_5331884570962474814[80] = 1;
}
void h_25(double *state, double *unused, double *out_972800349212928633) {
   out_972800349212928633[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1450262787404773615) {
   out_1450262787404773615[0] = 0;
   out_1450262787404773615[1] = 0;
   out_1450262787404773615[2] = 0;
   out_1450262787404773615[3] = 0;
   out_1450262787404773615[4] = 0;
   out_1450262787404773615[5] = 0;
   out_1450262787404773615[6] = 1;
   out_1450262787404773615[7] = 0;
   out_1450262787404773615[8] = 0;
}
void h_24(double *state, double *unused, double *out_3721810467178957937) {
   out_3721810467178957937[0] = state[4];
   out_3721810467178957937[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2107662811397549521) {
   out_2107662811397549521[0] = 0;
   out_2107662811397549521[1] = 0;
   out_2107662811397549521[2] = 0;
   out_2107662811397549521[3] = 0;
   out_2107662811397549521[4] = 1;
   out_2107662811397549521[5] = 0;
   out_2107662811397549521[6] = 0;
   out_2107662811397549521[7] = 0;
   out_2107662811397549521[8] = 0;
   out_2107662811397549521[9] = 0;
   out_2107662811397549521[10] = 0;
   out_2107662811397549521[11] = 0;
   out_2107662811397549521[12] = 0;
   out_2107662811397549521[13] = 0;
   out_2107662811397549521[14] = 1;
   out_2107662811397549521[15] = 0;
   out_2107662811397549521[16] = 0;
   out_2107662811397549521[17] = 0;
}
void h_30(double *state, double *unused, double *out_2664649179041093717) {
   out_2664649179041093717[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5977959117532381813) {
   out_5977959117532381813[0] = 0;
   out_5977959117532381813[1] = 0;
   out_5977959117532381813[2] = 0;
   out_5977959117532381813[3] = 0;
   out_5977959117532381813[4] = 1;
   out_5977959117532381813[5] = 0;
   out_5977959117532381813[6] = 0;
   out_5977959117532381813[7] = 0;
   out_5977959117532381813[8] = 0;
}
void h_26(double *state, double *unused, double *out_5701435230485426402) {
   out_5701435230485426402[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5191766106278829839) {
   out_5191766106278829839[0] = 0;
   out_5191766106278829839[1] = 0;
   out_5191766106278829839[2] = 0;
   out_5191766106278829839[3] = 0;
   out_5191766106278829839[4] = 0;
   out_5191766106278829839[5] = 0;
   out_5191766106278829839[6] = 0;
   out_5191766106278829839[7] = 1;
   out_5191766106278829839[8] = 0;
}
void h_27(double *state, double *unused, double *out_9068286904852396330) {
   out_9068286904852396330[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3754365046348438596) {
   out_3754365046348438596[0] = 0;
   out_3754365046348438596[1] = 0;
   out_3754365046348438596[2] = 0;
   out_3754365046348438596[3] = 1;
   out_3754365046348438596[4] = 0;
   out_3754365046348438596[5] = 0;
   out_3754365046348438596[6] = 0;
   out_3754365046348438596[7] = 0;
   out_3754365046348438596[8] = 0;
}
void h_29(double *state, double *unused, double *out_284477806406875368) {
   out_284477806406875368[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5467727773217989629) {
   out_5467727773217989629[0] = 0;
   out_5467727773217989629[1] = 1;
   out_5467727773217989629[2] = 0;
   out_5467727773217989629[3] = 0;
   out_5467727773217989629[4] = 0;
   out_5467727773217989629[5] = 0;
   out_5467727773217989629[6] = 0;
   out_5467727773217989629[7] = 0;
   out_5467727773217989629[8] = 0;
}
void h_28(double *state, double *unused, double *out_4953096253558923753) {
   out_4953096253558923753[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7896617283422031413) {
   out_7896617283422031413[0] = 1;
   out_7896617283422031413[1] = 0;
   out_7896617283422031413[2] = 0;
   out_7896617283422031413[3] = 0;
   out_7896617283422031413[4] = 0;
   out_7896617283422031413[5] = 0;
   out_7896617283422031413[6] = 0;
   out_7896617283422031413[7] = 0;
   out_7896617283422031413[8] = 0;
}
void h_31(double *state, double *unused, double *out_8854072999966977818) {
   out_8854072999966977818[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5817974208512181315) {
   out_5817974208512181315[0] = 0;
   out_5817974208512181315[1] = 0;
   out_5817974208512181315[2] = 0;
   out_5817974208512181315[3] = 0;
   out_5817974208512181315[4] = 0;
   out_5817974208512181315[5] = 0;
   out_5817974208512181315[6] = 0;
   out_5817974208512181315[7] = 0;
   out_5817974208512181315[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_161151116830974420) {
  err_fun(nom_x, delta_x, out_161151116830974420);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_868195081031098241) {
  inv_err_fun(nom_x, true_x, out_868195081031098241);
}
void car_H_mod_fun(double *state, double *out_9075143253330918196) {
  H_mod_fun(state, out_9075143253330918196);
}
void car_f_fun(double *state, double dt, double *out_4562342941996871353) {
  f_fun(state,  dt, out_4562342941996871353);
}
void car_F_fun(double *state, double dt, double *out_5331884570962474814) {
  F_fun(state,  dt, out_5331884570962474814);
}
void car_h_25(double *state, double *unused, double *out_972800349212928633) {
  h_25(state, unused, out_972800349212928633);
}
void car_H_25(double *state, double *unused, double *out_1450262787404773615) {
  H_25(state, unused, out_1450262787404773615);
}
void car_h_24(double *state, double *unused, double *out_3721810467178957937) {
  h_24(state, unused, out_3721810467178957937);
}
void car_H_24(double *state, double *unused, double *out_2107662811397549521) {
  H_24(state, unused, out_2107662811397549521);
}
void car_h_30(double *state, double *unused, double *out_2664649179041093717) {
  h_30(state, unused, out_2664649179041093717);
}
void car_H_30(double *state, double *unused, double *out_5977959117532381813) {
  H_30(state, unused, out_5977959117532381813);
}
void car_h_26(double *state, double *unused, double *out_5701435230485426402) {
  h_26(state, unused, out_5701435230485426402);
}
void car_H_26(double *state, double *unused, double *out_5191766106278829839) {
  H_26(state, unused, out_5191766106278829839);
}
void car_h_27(double *state, double *unused, double *out_9068286904852396330) {
  h_27(state, unused, out_9068286904852396330);
}
void car_H_27(double *state, double *unused, double *out_3754365046348438596) {
  H_27(state, unused, out_3754365046348438596);
}
void car_h_29(double *state, double *unused, double *out_284477806406875368) {
  h_29(state, unused, out_284477806406875368);
}
void car_H_29(double *state, double *unused, double *out_5467727773217989629) {
  H_29(state, unused, out_5467727773217989629);
}
void car_h_28(double *state, double *unused, double *out_4953096253558923753) {
  h_28(state, unused, out_4953096253558923753);
}
void car_H_28(double *state, double *unused, double *out_7896617283422031413) {
  H_28(state, unused, out_7896617283422031413);
}
void car_h_31(double *state, double *unused, double *out_8854072999966977818) {
  h_31(state, unused, out_8854072999966977818);
}
void car_H_31(double *state, double *unused, double *out_5817974208512181315) {
  H_31(state, unused, out_5817974208512181315);
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
