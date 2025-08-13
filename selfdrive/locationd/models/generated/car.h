#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_161151116830974420);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_868195081031098241);
void car_H_mod_fun(double *state, double *out_9075143253330918196);
void car_f_fun(double *state, double dt, double *out_4562342941996871353);
void car_F_fun(double *state, double dt, double *out_5331884570962474814);
void car_h_25(double *state, double *unused, double *out_972800349212928633);
void car_H_25(double *state, double *unused, double *out_1450262787404773615);
void car_h_24(double *state, double *unused, double *out_3721810467178957937);
void car_H_24(double *state, double *unused, double *out_2107662811397549521);
void car_h_30(double *state, double *unused, double *out_2664649179041093717);
void car_H_30(double *state, double *unused, double *out_5977959117532381813);
void car_h_26(double *state, double *unused, double *out_5701435230485426402);
void car_H_26(double *state, double *unused, double *out_5191766106278829839);
void car_h_27(double *state, double *unused, double *out_9068286904852396330);
void car_H_27(double *state, double *unused, double *out_3754365046348438596);
void car_h_29(double *state, double *unused, double *out_284477806406875368);
void car_H_29(double *state, double *unused, double *out_5467727773217989629);
void car_h_28(double *state, double *unused, double *out_4953096253558923753);
void car_H_28(double *state, double *unused, double *out_7896617283422031413);
void car_h_31(double *state, double *unused, double *out_8854072999966977818);
void car_H_31(double *state, double *unused, double *out_5817974208512181315);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}