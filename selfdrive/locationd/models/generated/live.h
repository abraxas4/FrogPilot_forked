#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_4746088412224613480);
void live_err_fun(double *nom_x, double *delta_x, double *out_9001890752258480632);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7751484030935475721);
void live_H_mod_fun(double *state, double *out_6372284872494722072);
void live_f_fun(double *state, double dt, double *out_7737125286734520429);
void live_F_fun(double *state, double dt, double *out_6924291271128232703);
void live_h_4(double *state, double *unused, double *out_7163581597376063005);
void live_H_4(double *state, double *unused, double *out_1282798183191685382);
void live_h_9(double *state, double *unused, double *out_1398902136694561631);
void live_H_9(double *state, double *unused, double *out_4171659735471764724);
void live_h_10(double *state, double *unused, double *out_1154166129467252208);
void live_H_10(double *state, double *unused, double *out_5904459669970046663);
void live_h_12(double *state, double *unused, double *out_5174139242233862891);
void live_H_12(double *state, double *unused, double *out_8949926496874135874);
void live_h_35(double *state, double *unused, double *out_8755816883392181434);
void live_H_35(double *state, double *unused, double *out_4649460240564292758);
void live_h_32(double *state, double *unused, double *out_6465677489100694951);
void live_H_32(double *state, double *unused, double *out_949950172302789475);
void live_h_13(double *state, double *unused, double *out_8354810049473215286);
void live_H_13(double *state, double *unused, double *out_347859678244556079);
void live_h_14(double *state, double *unused, double *out_1398902136694561631);
void live_H_14(double *state, double *unused, double *out_4171659735471764724);
void live_h_33(double *state, double *unused, double *out_3730663583502984644);
void live_H_33(double *state, double *unused, double *out_7800017245203150362);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}