#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4508435368596058991);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1037000126167514293);
void pose_H_mod_fun(double *state, double *out_4389262536507926261);
void pose_f_fun(double *state, double dt, double *out_1664741234285638622);
void pose_F_fun(double *state, double dt, double *out_4647166909216110620);
void pose_h_4(double *state, double *unused, double *out_8166043361258674617);
void pose_H_4(double *state, double *unused, double *out_8904963152162544620);
void pose_h_10(double *state, double *unused, double *out_3269605573430160868);
void pose_H_10(double *state, double *unused, double *out_9110126995673016572);
void pose_h_13(double *state, double *unused, double *out_5562461953863127465);
void pose_H_13(double *state, double *unused, double *out_5692689326830211819);
void pose_h_14(double *state, double *unused, double *out_6778377519299831387);
void pose_H_14(double *state, double *unused, double *out_4941722295823060091);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}