#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4508435368596058991) {
   out_4508435368596058991[0] = delta_x[0] + nom_x[0];
   out_4508435368596058991[1] = delta_x[1] + nom_x[1];
   out_4508435368596058991[2] = delta_x[2] + nom_x[2];
   out_4508435368596058991[3] = delta_x[3] + nom_x[3];
   out_4508435368596058991[4] = delta_x[4] + nom_x[4];
   out_4508435368596058991[5] = delta_x[5] + nom_x[5];
   out_4508435368596058991[6] = delta_x[6] + nom_x[6];
   out_4508435368596058991[7] = delta_x[7] + nom_x[7];
   out_4508435368596058991[8] = delta_x[8] + nom_x[8];
   out_4508435368596058991[9] = delta_x[9] + nom_x[9];
   out_4508435368596058991[10] = delta_x[10] + nom_x[10];
   out_4508435368596058991[11] = delta_x[11] + nom_x[11];
   out_4508435368596058991[12] = delta_x[12] + nom_x[12];
   out_4508435368596058991[13] = delta_x[13] + nom_x[13];
   out_4508435368596058991[14] = delta_x[14] + nom_x[14];
   out_4508435368596058991[15] = delta_x[15] + nom_x[15];
   out_4508435368596058991[16] = delta_x[16] + nom_x[16];
   out_4508435368596058991[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1037000126167514293) {
   out_1037000126167514293[0] = -nom_x[0] + true_x[0];
   out_1037000126167514293[1] = -nom_x[1] + true_x[1];
   out_1037000126167514293[2] = -nom_x[2] + true_x[2];
   out_1037000126167514293[3] = -nom_x[3] + true_x[3];
   out_1037000126167514293[4] = -nom_x[4] + true_x[4];
   out_1037000126167514293[5] = -nom_x[5] + true_x[5];
   out_1037000126167514293[6] = -nom_x[6] + true_x[6];
   out_1037000126167514293[7] = -nom_x[7] + true_x[7];
   out_1037000126167514293[8] = -nom_x[8] + true_x[8];
   out_1037000126167514293[9] = -nom_x[9] + true_x[9];
   out_1037000126167514293[10] = -nom_x[10] + true_x[10];
   out_1037000126167514293[11] = -nom_x[11] + true_x[11];
   out_1037000126167514293[12] = -nom_x[12] + true_x[12];
   out_1037000126167514293[13] = -nom_x[13] + true_x[13];
   out_1037000126167514293[14] = -nom_x[14] + true_x[14];
   out_1037000126167514293[15] = -nom_x[15] + true_x[15];
   out_1037000126167514293[16] = -nom_x[16] + true_x[16];
   out_1037000126167514293[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4389262536507926261) {
   out_4389262536507926261[0] = 1.0;
   out_4389262536507926261[1] = 0.0;
   out_4389262536507926261[2] = 0.0;
   out_4389262536507926261[3] = 0.0;
   out_4389262536507926261[4] = 0.0;
   out_4389262536507926261[5] = 0.0;
   out_4389262536507926261[6] = 0.0;
   out_4389262536507926261[7] = 0.0;
   out_4389262536507926261[8] = 0.0;
   out_4389262536507926261[9] = 0.0;
   out_4389262536507926261[10] = 0.0;
   out_4389262536507926261[11] = 0.0;
   out_4389262536507926261[12] = 0.0;
   out_4389262536507926261[13] = 0.0;
   out_4389262536507926261[14] = 0.0;
   out_4389262536507926261[15] = 0.0;
   out_4389262536507926261[16] = 0.0;
   out_4389262536507926261[17] = 0.0;
   out_4389262536507926261[18] = 0.0;
   out_4389262536507926261[19] = 1.0;
   out_4389262536507926261[20] = 0.0;
   out_4389262536507926261[21] = 0.0;
   out_4389262536507926261[22] = 0.0;
   out_4389262536507926261[23] = 0.0;
   out_4389262536507926261[24] = 0.0;
   out_4389262536507926261[25] = 0.0;
   out_4389262536507926261[26] = 0.0;
   out_4389262536507926261[27] = 0.0;
   out_4389262536507926261[28] = 0.0;
   out_4389262536507926261[29] = 0.0;
   out_4389262536507926261[30] = 0.0;
   out_4389262536507926261[31] = 0.0;
   out_4389262536507926261[32] = 0.0;
   out_4389262536507926261[33] = 0.0;
   out_4389262536507926261[34] = 0.0;
   out_4389262536507926261[35] = 0.0;
   out_4389262536507926261[36] = 0.0;
   out_4389262536507926261[37] = 0.0;
   out_4389262536507926261[38] = 1.0;
   out_4389262536507926261[39] = 0.0;
   out_4389262536507926261[40] = 0.0;
   out_4389262536507926261[41] = 0.0;
   out_4389262536507926261[42] = 0.0;
   out_4389262536507926261[43] = 0.0;
   out_4389262536507926261[44] = 0.0;
   out_4389262536507926261[45] = 0.0;
   out_4389262536507926261[46] = 0.0;
   out_4389262536507926261[47] = 0.0;
   out_4389262536507926261[48] = 0.0;
   out_4389262536507926261[49] = 0.0;
   out_4389262536507926261[50] = 0.0;
   out_4389262536507926261[51] = 0.0;
   out_4389262536507926261[52] = 0.0;
   out_4389262536507926261[53] = 0.0;
   out_4389262536507926261[54] = 0.0;
   out_4389262536507926261[55] = 0.0;
   out_4389262536507926261[56] = 0.0;
   out_4389262536507926261[57] = 1.0;
   out_4389262536507926261[58] = 0.0;
   out_4389262536507926261[59] = 0.0;
   out_4389262536507926261[60] = 0.0;
   out_4389262536507926261[61] = 0.0;
   out_4389262536507926261[62] = 0.0;
   out_4389262536507926261[63] = 0.0;
   out_4389262536507926261[64] = 0.0;
   out_4389262536507926261[65] = 0.0;
   out_4389262536507926261[66] = 0.0;
   out_4389262536507926261[67] = 0.0;
   out_4389262536507926261[68] = 0.0;
   out_4389262536507926261[69] = 0.0;
   out_4389262536507926261[70] = 0.0;
   out_4389262536507926261[71] = 0.0;
   out_4389262536507926261[72] = 0.0;
   out_4389262536507926261[73] = 0.0;
   out_4389262536507926261[74] = 0.0;
   out_4389262536507926261[75] = 0.0;
   out_4389262536507926261[76] = 1.0;
   out_4389262536507926261[77] = 0.0;
   out_4389262536507926261[78] = 0.0;
   out_4389262536507926261[79] = 0.0;
   out_4389262536507926261[80] = 0.0;
   out_4389262536507926261[81] = 0.0;
   out_4389262536507926261[82] = 0.0;
   out_4389262536507926261[83] = 0.0;
   out_4389262536507926261[84] = 0.0;
   out_4389262536507926261[85] = 0.0;
   out_4389262536507926261[86] = 0.0;
   out_4389262536507926261[87] = 0.0;
   out_4389262536507926261[88] = 0.0;
   out_4389262536507926261[89] = 0.0;
   out_4389262536507926261[90] = 0.0;
   out_4389262536507926261[91] = 0.0;
   out_4389262536507926261[92] = 0.0;
   out_4389262536507926261[93] = 0.0;
   out_4389262536507926261[94] = 0.0;
   out_4389262536507926261[95] = 1.0;
   out_4389262536507926261[96] = 0.0;
   out_4389262536507926261[97] = 0.0;
   out_4389262536507926261[98] = 0.0;
   out_4389262536507926261[99] = 0.0;
   out_4389262536507926261[100] = 0.0;
   out_4389262536507926261[101] = 0.0;
   out_4389262536507926261[102] = 0.0;
   out_4389262536507926261[103] = 0.0;
   out_4389262536507926261[104] = 0.0;
   out_4389262536507926261[105] = 0.0;
   out_4389262536507926261[106] = 0.0;
   out_4389262536507926261[107] = 0.0;
   out_4389262536507926261[108] = 0.0;
   out_4389262536507926261[109] = 0.0;
   out_4389262536507926261[110] = 0.0;
   out_4389262536507926261[111] = 0.0;
   out_4389262536507926261[112] = 0.0;
   out_4389262536507926261[113] = 0.0;
   out_4389262536507926261[114] = 1.0;
   out_4389262536507926261[115] = 0.0;
   out_4389262536507926261[116] = 0.0;
   out_4389262536507926261[117] = 0.0;
   out_4389262536507926261[118] = 0.0;
   out_4389262536507926261[119] = 0.0;
   out_4389262536507926261[120] = 0.0;
   out_4389262536507926261[121] = 0.0;
   out_4389262536507926261[122] = 0.0;
   out_4389262536507926261[123] = 0.0;
   out_4389262536507926261[124] = 0.0;
   out_4389262536507926261[125] = 0.0;
   out_4389262536507926261[126] = 0.0;
   out_4389262536507926261[127] = 0.0;
   out_4389262536507926261[128] = 0.0;
   out_4389262536507926261[129] = 0.0;
   out_4389262536507926261[130] = 0.0;
   out_4389262536507926261[131] = 0.0;
   out_4389262536507926261[132] = 0.0;
   out_4389262536507926261[133] = 1.0;
   out_4389262536507926261[134] = 0.0;
   out_4389262536507926261[135] = 0.0;
   out_4389262536507926261[136] = 0.0;
   out_4389262536507926261[137] = 0.0;
   out_4389262536507926261[138] = 0.0;
   out_4389262536507926261[139] = 0.0;
   out_4389262536507926261[140] = 0.0;
   out_4389262536507926261[141] = 0.0;
   out_4389262536507926261[142] = 0.0;
   out_4389262536507926261[143] = 0.0;
   out_4389262536507926261[144] = 0.0;
   out_4389262536507926261[145] = 0.0;
   out_4389262536507926261[146] = 0.0;
   out_4389262536507926261[147] = 0.0;
   out_4389262536507926261[148] = 0.0;
   out_4389262536507926261[149] = 0.0;
   out_4389262536507926261[150] = 0.0;
   out_4389262536507926261[151] = 0.0;
   out_4389262536507926261[152] = 1.0;
   out_4389262536507926261[153] = 0.0;
   out_4389262536507926261[154] = 0.0;
   out_4389262536507926261[155] = 0.0;
   out_4389262536507926261[156] = 0.0;
   out_4389262536507926261[157] = 0.0;
   out_4389262536507926261[158] = 0.0;
   out_4389262536507926261[159] = 0.0;
   out_4389262536507926261[160] = 0.0;
   out_4389262536507926261[161] = 0.0;
   out_4389262536507926261[162] = 0.0;
   out_4389262536507926261[163] = 0.0;
   out_4389262536507926261[164] = 0.0;
   out_4389262536507926261[165] = 0.0;
   out_4389262536507926261[166] = 0.0;
   out_4389262536507926261[167] = 0.0;
   out_4389262536507926261[168] = 0.0;
   out_4389262536507926261[169] = 0.0;
   out_4389262536507926261[170] = 0.0;
   out_4389262536507926261[171] = 1.0;
   out_4389262536507926261[172] = 0.0;
   out_4389262536507926261[173] = 0.0;
   out_4389262536507926261[174] = 0.0;
   out_4389262536507926261[175] = 0.0;
   out_4389262536507926261[176] = 0.0;
   out_4389262536507926261[177] = 0.0;
   out_4389262536507926261[178] = 0.0;
   out_4389262536507926261[179] = 0.0;
   out_4389262536507926261[180] = 0.0;
   out_4389262536507926261[181] = 0.0;
   out_4389262536507926261[182] = 0.0;
   out_4389262536507926261[183] = 0.0;
   out_4389262536507926261[184] = 0.0;
   out_4389262536507926261[185] = 0.0;
   out_4389262536507926261[186] = 0.0;
   out_4389262536507926261[187] = 0.0;
   out_4389262536507926261[188] = 0.0;
   out_4389262536507926261[189] = 0.0;
   out_4389262536507926261[190] = 1.0;
   out_4389262536507926261[191] = 0.0;
   out_4389262536507926261[192] = 0.0;
   out_4389262536507926261[193] = 0.0;
   out_4389262536507926261[194] = 0.0;
   out_4389262536507926261[195] = 0.0;
   out_4389262536507926261[196] = 0.0;
   out_4389262536507926261[197] = 0.0;
   out_4389262536507926261[198] = 0.0;
   out_4389262536507926261[199] = 0.0;
   out_4389262536507926261[200] = 0.0;
   out_4389262536507926261[201] = 0.0;
   out_4389262536507926261[202] = 0.0;
   out_4389262536507926261[203] = 0.0;
   out_4389262536507926261[204] = 0.0;
   out_4389262536507926261[205] = 0.0;
   out_4389262536507926261[206] = 0.0;
   out_4389262536507926261[207] = 0.0;
   out_4389262536507926261[208] = 0.0;
   out_4389262536507926261[209] = 1.0;
   out_4389262536507926261[210] = 0.0;
   out_4389262536507926261[211] = 0.0;
   out_4389262536507926261[212] = 0.0;
   out_4389262536507926261[213] = 0.0;
   out_4389262536507926261[214] = 0.0;
   out_4389262536507926261[215] = 0.0;
   out_4389262536507926261[216] = 0.0;
   out_4389262536507926261[217] = 0.0;
   out_4389262536507926261[218] = 0.0;
   out_4389262536507926261[219] = 0.0;
   out_4389262536507926261[220] = 0.0;
   out_4389262536507926261[221] = 0.0;
   out_4389262536507926261[222] = 0.0;
   out_4389262536507926261[223] = 0.0;
   out_4389262536507926261[224] = 0.0;
   out_4389262536507926261[225] = 0.0;
   out_4389262536507926261[226] = 0.0;
   out_4389262536507926261[227] = 0.0;
   out_4389262536507926261[228] = 1.0;
   out_4389262536507926261[229] = 0.0;
   out_4389262536507926261[230] = 0.0;
   out_4389262536507926261[231] = 0.0;
   out_4389262536507926261[232] = 0.0;
   out_4389262536507926261[233] = 0.0;
   out_4389262536507926261[234] = 0.0;
   out_4389262536507926261[235] = 0.0;
   out_4389262536507926261[236] = 0.0;
   out_4389262536507926261[237] = 0.0;
   out_4389262536507926261[238] = 0.0;
   out_4389262536507926261[239] = 0.0;
   out_4389262536507926261[240] = 0.0;
   out_4389262536507926261[241] = 0.0;
   out_4389262536507926261[242] = 0.0;
   out_4389262536507926261[243] = 0.0;
   out_4389262536507926261[244] = 0.0;
   out_4389262536507926261[245] = 0.0;
   out_4389262536507926261[246] = 0.0;
   out_4389262536507926261[247] = 1.0;
   out_4389262536507926261[248] = 0.0;
   out_4389262536507926261[249] = 0.0;
   out_4389262536507926261[250] = 0.0;
   out_4389262536507926261[251] = 0.0;
   out_4389262536507926261[252] = 0.0;
   out_4389262536507926261[253] = 0.0;
   out_4389262536507926261[254] = 0.0;
   out_4389262536507926261[255] = 0.0;
   out_4389262536507926261[256] = 0.0;
   out_4389262536507926261[257] = 0.0;
   out_4389262536507926261[258] = 0.0;
   out_4389262536507926261[259] = 0.0;
   out_4389262536507926261[260] = 0.0;
   out_4389262536507926261[261] = 0.0;
   out_4389262536507926261[262] = 0.0;
   out_4389262536507926261[263] = 0.0;
   out_4389262536507926261[264] = 0.0;
   out_4389262536507926261[265] = 0.0;
   out_4389262536507926261[266] = 1.0;
   out_4389262536507926261[267] = 0.0;
   out_4389262536507926261[268] = 0.0;
   out_4389262536507926261[269] = 0.0;
   out_4389262536507926261[270] = 0.0;
   out_4389262536507926261[271] = 0.0;
   out_4389262536507926261[272] = 0.0;
   out_4389262536507926261[273] = 0.0;
   out_4389262536507926261[274] = 0.0;
   out_4389262536507926261[275] = 0.0;
   out_4389262536507926261[276] = 0.0;
   out_4389262536507926261[277] = 0.0;
   out_4389262536507926261[278] = 0.0;
   out_4389262536507926261[279] = 0.0;
   out_4389262536507926261[280] = 0.0;
   out_4389262536507926261[281] = 0.0;
   out_4389262536507926261[282] = 0.0;
   out_4389262536507926261[283] = 0.0;
   out_4389262536507926261[284] = 0.0;
   out_4389262536507926261[285] = 1.0;
   out_4389262536507926261[286] = 0.0;
   out_4389262536507926261[287] = 0.0;
   out_4389262536507926261[288] = 0.0;
   out_4389262536507926261[289] = 0.0;
   out_4389262536507926261[290] = 0.0;
   out_4389262536507926261[291] = 0.0;
   out_4389262536507926261[292] = 0.0;
   out_4389262536507926261[293] = 0.0;
   out_4389262536507926261[294] = 0.0;
   out_4389262536507926261[295] = 0.0;
   out_4389262536507926261[296] = 0.0;
   out_4389262536507926261[297] = 0.0;
   out_4389262536507926261[298] = 0.0;
   out_4389262536507926261[299] = 0.0;
   out_4389262536507926261[300] = 0.0;
   out_4389262536507926261[301] = 0.0;
   out_4389262536507926261[302] = 0.0;
   out_4389262536507926261[303] = 0.0;
   out_4389262536507926261[304] = 1.0;
   out_4389262536507926261[305] = 0.0;
   out_4389262536507926261[306] = 0.0;
   out_4389262536507926261[307] = 0.0;
   out_4389262536507926261[308] = 0.0;
   out_4389262536507926261[309] = 0.0;
   out_4389262536507926261[310] = 0.0;
   out_4389262536507926261[311] = 0.0;
   out_4389262536507926261[312] = 0.0;
   out_4389262536507926261[313] = 0.0;
   out_4389262536507926261[314] = 0.0;
   out_4389262536507926261[315] = 0.0;
   out_4389262536507926261[316] = 0.0;
   out_4389262536507926261[317] = 0.0;
   out_4389262536507926261[318] = 0.0;
   out_4389262536507926261[319] = 0.0;
   out_4389262536507926261[320] = 0.0;
   out_4389262536507926261[321] = 0.0;
   out_4389262536507926261[322] = 0.0;
   out_4389262536507926261[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1664741234285638622) {
   out_1664741234285638622[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1664741234285638622[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1664741234285638622[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1664741234285638622[3] = dt*state[12] + state[3];
   out_1664741234285638622[4] = dt*state[13] + state[4];
   out_1664741234285638622[5] = dt*state[14] + state[5];
   out_1664741234285638622[6] = state[6];
   out_1664741234285638622[7] = state[7];
   out_1664741234285638622[8] = state[8];
   out_1664741234285638622[9] = state[9];
   out_1664741234285638622[10] = state[10];
   out_1664741234285638622[11] = state[11];
   out_1664741234285638622[12] = state[12];
   out_1664741234285638622[13] = state[13];
   out_1664741234285638622[14] = state[14];
   out_1664741234285638622[15] = state[15];
   out_1664741234285638622[16] = state[16];
   out_1664741234285638622[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4647166909216110620) {
   out_4647166909216110620[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4647166909216110620[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4647166909216110620[2] = 0;
   out_4647166909216110620[3] = 0;
   out_4647166909216110620[4] = 0;
   out_4647166909216110620[5] = 0;
   out_4647166909216110620[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4647166909216110620[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4647166909216110620[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4647166909216110620[9] = 0;
   out_4647166909216110620[10] = 0;
   out_4647166909216110620[11] = 0;
   out_4647166909216110620[12] = 0;
   out_4647166909216110620[13] = 0;
   out_4647166909216110620[14] = 0;
   out_4647166909216110620[15] = 0;
   out_4647166909216110620[16] = 0;
   out_4647166909216110620[17] = 0;
   out_4647166909216110620[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4647166909216110620[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4647166909216110620[20] = 0;
   out_4647166909216110620[21] = 0;
   out_4647166909216110620[22] = 0;
   out_4647166909216110620[23] = 0;
   out_4647166909216110620[24] = 0;
   out_4647166909216110620[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4647166909216110620[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4647166909216110620[27] = 0;
   out_4647166909216110620[28] = 0;
   out_4647166909216110620[29] = 0;
   out_4647166909216110620[30] = 0;
   out_4647166909216110620[31] = 0;
   out_4647166909216110620[32] = 0;
   out_4647166909216110620[33] = 0;
   out_4647166909216110620[34] = 0;
   out_4647166909216110620[35] = 0;
   out_4647166909216110620[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4647166909216110620[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4647166909216110620[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4647166909216110620[39] = 0;
   out_4647166909216110620[40] = 0;
   out_4647166909216110620[41] = 0;
   out_4647166909216110620[42] = 0;
   out_4647166909216110620[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4647166909216110620[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4647166909216110620[45] = 0;
   out_4647166909216110620[46] = 0;
   out_4647166909216110620[47] = 0;
   out_4647166909216110620[48] = 0;
   out_4647166909216110620[49] = 0;
   out_4647166909216110620[50] = 0;
   out_4647166909216110620[51] = 0;
   out_4647166909216110620[52] = 0;
   out_4647166909216110620[53] = 0;
   out_4647166909216110620[54] = 0;
   out_4647166909216110620[55] = 0;
   out_4647166909216110620[56] = 0;
   out_4647166909216110620[57] = 1;
   out_4647166909216110620[58] = 0;
   out_4647166909216110620[59] = 0;
   out_4647166909216110620[60] = 0;
   out_4647166909216110620[61] = 0;
   out_4647166909216110620[62] = 0;
   out_4647166909216110620[63] = 0;
   out_4647166909216110620[64] = 0;
   out_4647166909216110620[65] = 0;
   out_4647166909216110620[66] = dt;
   out_4647166909216110620[67] = 0;
   out_4647166909216110620[68] = 0;
   out_4647166909216110620[69] = 0;
   out_4647166909216110620[70] = 0;
   out_4647166909216110620[71] = 0;
   out_4647166909216110620[72] = 0;
   out_4647166909216110620[73] = 0;
   out_4647166909216110620[74] = 0;
   out_4647166909216110620[75] = 0;
   out_4647166909216110620[76] = 1;
   out_4647166909216110620[77] = 0;
   out_4647166909216110620[78] = 0;
   out_4647166909216110620[79] = 0;
   out_4647166909216110620[80] = 0;
   out_4647166909216110620[81] = 0;
   out_4647166909216110620[82] = 0;
   out_4647166909216110620[83] = 0;
   out_4647166909216110620[84] = 0;
   out_4647166909216110620[85] = dt;
   out_4647166909216110620[86] = 0;
   out_4647166909216110620[87] = 0;
   out_4647166909216110620[88] = 0;
   out_4647166909216110620[89] = 0;
   out_4647166909216110620[90] = 0;
   out_4647166909216110620[91] = 0;
   out_4647166909216110620[92] = 0;
   out_4647166909216110620[93] = 0;
   out_4647166909216110620[94] = 0;
   out_4647166909216110620[95] = 1;
   out_4647166909216110620[96] = 0;
   out_4647166909216110620[97] = 0;
   out_4647166909216110620[98] = 0;
   out_4647166909216110620[99] = 0;
   out_4647166909216110620[100] = 0;
   out_4647166909216110620[101] = 0;
   out_4647166909216110620[102] = 0;
   out_4647166909216110620[103] = 0;
   out_4647166909216110620[104] = dt;
   out_4647166909216110620[105] = 0;
   out_4647166909216110620[106] = 0;
   out_4647166909216110620[107] = 0;
   out_4647166909216110620[108] = 0;
   out_4647166909216110620[109] = 0;
   out_4647166909216110620[110] = 0;
   out_4647166909216110620[111] = 0;
   out_4647166909216110620[112] = 0;
   out_4647166909216110620[113] = 0;
   out_4647166909216110620[114] = 1;
   out_4647166909216110620[115] = 0;
   out_4647166909216110620[116] = 0;
   out_4647166909216110620[117] = 0;
   out_4647166909216110620[118] = 0;
   out_4647166909216110620[119] = 0;
   out_4647166909216110620[120] = 0;
   out_4647166909216110620[121] = 0;
   out_4647166909216110620[122] = 0;
   out_4647166909216110620[123] = 0;
   out_4647166909216110620[124] = 0;
   out_4647166909216110620[125] = 0;
   out_4647166909216110620[126] = 0;
   out_4647166909216110620[127] = 0;
   out_4647166909216110620[128] = 0;
   out_4647166909216110620[129] = 0;
   out_4647166909216110620[130] = 0;
   out_4647166909216110620[131] = 0;
   out_4647166909216110620[132] = 0;
   out_4647166909216110620[133] = 1;
   out_4647166909216110620[134] = 0;
   out_4647166909216110620[135] = 0;
   out_4647166909216110620[136] = 0;
   out_4647166909216110620[137] = 0;
   out_4647166909216110620[138] = 0;
   out_4647166909216110620[139] = 0;
   out_4647166909216110620[140] = 0;
   out_4647166909216110620[141] = 0;
   out_4647166909216110620[142] = 0;
   out_4647166909216110620[143] = 0;
   out_4647166909216110620[144] = 0;
   out_4647166909216110620[145] = 0;
   out_4647166909216110620[146] = 0;
   out_4647166909216110620[147] = 0;
   out_4647166909216110620[148] = 0;
   out_4647166909216110620[149] = 0;
   out_4647166909216110620[150] = 0;
   out_4647166909216110620[151] = 0;
   out_4647166909216110620[152] = 1;
   out_4647166909216110620[153] = 0;
   out_4647166909216110620[154] = 0;
   out_4647166909216110620[155] = 0;
   out_4647166909216110620[156] = 0;
   out_4647166909216110620[157] = 0;
   out_4647166909216110620[158] = 0;
   out_4647166909216110620[159] = 0;
   out_4647166909216110620[160] = 0;
   out_4647166909216110620[161] = 0;
   out_4647166909216110620[162] = 0;
   out_4647166909216110620[163] = 0;
   out_4647166909216110620[164] = 0;
   out_4647166909216110620[165] = 0;
   out_4647166909216110620[166] = 0;
   out_4647166909216110620[167] = 0;
   out_4647166909216110620[168] = 0;
   out_4647166909216110620[169] = 0;
   out_4647166909216110620[170] = 0;
   out_4647166909216110620[171] = 1;
   out_4647166909216110620[172] = 0;
   out_4647166909216110620[173] = 0;
   out_4647166909216110620[174] = 0;
   out_4647166909216110620[175] = 0;
   out_4647166909216110620[176] = 0;
   out_4647166909216110620[177] = 0;
   out_4647166909216110620[178] = 0;
   out_4647166909216110620[179] = 0;
   out_4647166909216110620[180] = 0;
   out_4647166909216110620[181] = 0;
   out_4647166909216110620[182] = 0;
   out_4647166909216110620[183] = 0;
   out_4647166909216110620[184] = 0;
   out_4647166909216110620[185] = 0;
   out_4647166909216110620[186] = 0;
   out_4647166909216110620[187] = 0;
   out_4647166909216110620[188] = 0;
   out_4647166909216110620[189] = 0;
   out_4647166909216110620[190] = 1;
   out_4647166909216110620[191] = 0;
   out_4647166909216110620[192] = 0;
   out_4647166909216110620[193] = 0;
   out_4647166909216110620[194] = 0;
   out_4647166909216110620[195] = 0;
   out_4647166909216110620[196] = 0;
   out_4647166909216110620[197] = 0;
   out_4647166909216110620[198] = 0;
   out_4647166909216110620[199] = 0;
   out_4647166909216110620[200] = 0;
   out_4647166909216110620[201] = 0;
   out_4647166909216110620[202] = 0;
   out_4647166909216110620[203] = 0;
   out_4647166909216110620[204] = 0;
   out_4647166909216110620[205] = 0;
   out_4647166909216110620[206] = 0;
   out_4647166909216110620[207] = 0;
   out_4647166909216110620[208] = 0;
   out_4647166909216110620[209] = 1;
   out_4647166909216110620[210] = 0;
   out_4647166909216110620[211] = 0;
   out_4647166909216110620[212] = 0;
   out_4647166909216110620[213] = 0;
   out_4647166909216110620[214] = 0;
   out_4647166909216110620[215] = 0;
   out_4647166909216110620[216] = 0;
   out_4647166909216110620[217] = 0;
   out_4647166909216110620[218] = 0;
   out_4647166909216110620[219] = 0;
   out_4647166909216110620[220] = 0;
   out_4647166909216110620[221] = 0;
   out_4647166909216110620[222] = 0;
   out_4647166909216110620[223] = 0;
   out_4647166909216110620[224] = 0;
   out_4647166909216110620[225] = 0;
   out_4647166909216110620[226] = 0;
   out_4647166909216110620[227] = 0;
   out_4647166909216110620[228] = 1;
   out_4647166909216110620[229] = 0;
   out_4647166909216110620[230] = 0;
   out_4647166909216110620[231] = 0;
   out_4647166909216110620[232] = 0;
   out_4647166909216110620[233] = 0;
   out_4647166909216110620[234] = 0;
   out_4647166909216110620[235] = 0;
   out_4647166909216110620[236] = 0;
   out_4647166909216110620[237] = 0;
   out_4647166909216110620[238] = 0;
   out_4647166909216110620[239] = 0;
   out_4647166909216110620[240] = 0;
   out_4647166909216110620[241] = 0;
   out_4647166909216110620[242] = 0;
   out_4647166909216110620[243] = 0;
   out_4647166909216110620[244] = 0;
   out_4647166909216110620[245] = 0;
   out_4647166909216110620[246] = 0;
   out_4647166909216110620[247] = 1;
   out_4647166909216110620[248] = 0;
   out_4647166909216110620[249] = 0;
   out_4647166909216110620[250] = 0;
   out_4647166909216110620[251] = 0;
   out_4647166909216110620[252] = 0;
   out_4647166909216110620[253] = 0;
   out_4647166909216110620[254] = 0;
   out_4647166909216110620[255] = 0;
   out_4647166909216110620[256] = 0;
   out_4647166909216110620[257] = 0;
   out_4647166909216110620[258] = 0;
   out_4647166909216110620[259] = 0;
   out_4647166909216110620[260] = 0;
   out_4647166909216110620[261] = 0;
   out_4647166909216110620[262] = 0;
   out_4647166909216110620[263] = 0;
   out_4647166909216110620[264] = 0;
   out_4647166909216110620[265] = 0;
   out_4647166909216110620[266] = 1;
   out_4647166909216110620[267] = 0;
   out_4647166909216110620[268] = 0;
   out_4647166909216110620[269] = 0;
   out_4647166909216110620[270] = 0;
   out_4647166909216110620[271] = 0;
   out_4647166909216110620[272] = 0;
   out_4647166909216110620[273] = 0;
   out_4647166909216110620[274] = 0;
   out_4647166909216110620[275] = 0;
   out_4647166909216110620[276] = 0;
   out_4647166909216110620[277] = 0;
   out_4647166909216110620[278] = 0;
   out_4647166909216110620[279] = 0;
   out_4647166909216110620[280] = 0;
   out_4647166909216110620[281] = 0;
   out_4647166909216110620[282] = 0;
   out_4647166909216110620[283] = 0;
   out_4647166909216110620[284] = 0;
   out_4647166909216110620[285] = 1;
   out_4647166909216110620[286] = 0;
   out_4647166909216110620[287] = 0;
   out_4647166909216110620[288] = 0;
   out_4647166909216110620[289] = 0;
   out_4647166909216110620[290] = 0;
   out_4647166909216110620[291] = 0;
   out_4647166909216110620[292] = 0;
   out_4647166909216110620[293] = 0;
   out_4647166909216110620[294] = 0;
   out_4647166909216110620[295] = 0;
   out_4647166909216110620[296] = 0;
   out_4647166909216110620[297] = 0;
   out_4647166909216110620[298] = 0;
   out_4647166909216110620[299] = 0;
   out_4647166909216110620[300] = 0;
   out_4647166909216110620[301] = 0;
   out_4647166909216110620[302] = 0;
   out_4647166909216110620[303] = 0;
   out_4647166909216110620[304] = 1;
   out_4647166909216110620[305] = 0;
   out_4647166909216110620[306] = 0;
   out_4647166909216110620[307] = 0;
   out_4647166909216110620[308] = 0;
   out_4647166909216110620[309] = 0;
   out_4647166909216110620[310] = 0;
   out_4647166909216110620[311] = 0;
   out_4647166909216110620[312] = 0;
   out_4647166909216110620[313] = 0;
   out_4647166909216110620[314] = 0;
   out_4647166909216110620[315] = 0;
   out_4647166909216110620[316] = 0;
   out_4647166909216110620[317] = 0;
   out_4647166909216110620[318] = 0;
   out_4647166909216110620[319] = 0;
   out_4647166909216110620[320] = 0;
   out_4647166909216110620[321] = 0;
   out_4647166909216110620[322] = 0;
   out_4647166909216110620[323] = 1;
}
void h_4(double *state, double *unused, double *out_8166043361258674617) {
   out_8166043361258674617[0] = state[6] + state[9];
   out_8166043361258674617[1] = state[7] + state[10];
   out_8166043361258674617[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8904963152162544620) {
   out_8904963152162544620[0] = 0;
   out_8904963152162544620[1] = 0;
   out_8904963152162544620[2] = 0;
   out_8904963152162544620[3] = 0;
   out_8904963152162544620[4] = 0;
   out_8904963152162544620[5] = 0;
   out_8904963152162544620[6] = 1;
   out_8904963152162544620[7] = 0;
   out_8904963152162544620[8] = 0;
   out_8904963152162544620[9] = 1;
   out_8904963152162544620[10] = 0;
   out_8904963152162544620[11] = 0;
   out_8904963152162544620[12] = 0;
   out_8904963152162544620[13] = 0;
   out_8904963152162544620[14] = 0;
   out_8904963152162544620[15] = 0;
   out_8904963152162544620[16] = 0;
   out_8904963152162544620[17] = 0;
   out_8904963152162544620[18] = 0;
   out_8904963152162544620[19] = 0;
   out_8904963152162544620[20] = 0;
   out_8904963152162544620[21] = 0;
   out_8904963152162544620[22] = 0;
   out_8904963152162544620[23] = 0;
   out_8904963152162544620[24] = 0;
   out_8904963152162544620[25] = 1;
   out_8904963152162544620[26] = 0;
   out_8904963152162544620[27] = 0;
   out_8904963152162544620[28] = 1;
   out_8904963152162544620[29] = 0;
   out_8904963152162544620[30] = 0;
   out_8904963152162544620[31] = 0;
   out_8904963152162544620[32] = 0;
   out_8904963152162544620[33] = 0;
   out_8904963152162544620[34] = 0;
   out_8904963152162544620[35] = 0;
   out_8904963152162544620[36] = 0;
   out_8904963152162544620[37] = 0;
   out_8904963152162544620[38] = 0;
   out_8904963152162544620[39] = 0;
   out_8904963152162544620[40] = 0;
   out_8904963152162544620[41] = 0;
   out_8904963152162544620[42] = 0;
   out_8904963152162544620[43] = 0;
   out_8904963152162544620[44] = 1;
   out_8904963152162544620[45] = 0;
   out_8904963152162544620[46] = 0;
   out_8904963152162544620[47] = 1;
   out_8904963152162544620[48] = 0;
   out_8904963152162544620[49] = 0;
   out_8904963152162544620[50] = 0;
   out_8904963152162544620[51] = 0;
   out_8904963152162544620[52] = 0;
   out_8904963152162544620[53] = 0;
}
void h_10(double *state, double *unused, double *out_3269605573430160868) {
   out_3269605573430160868[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3269605573430160868[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3269605573430160868[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_9110126995673016572) {
   out_9110126995673016572[0] = 0;
   out_9110126995673016572[1] = 9.8100000000000005*cos(state[1]);
   out_9110126995673016572[2] = 0;
   out_9110126995673016572[3] = 0;
   out_9110126995673016572[4] = -state[8];
   out_9110126995673016572[5] = state[7];
   out_9110126995673016572[6] = 0;
   out_9110126995673016572[7] = state[5];
   out_9110126995673016572[8] = -state[4];
   out_9110126995673016572[9] = 0;
   out_9110126995673016572[10] = 0;
   out_9110126995673016572[11] = 0;
   out_9110126995673016572[12] = 1;
   out_9110126995673016572[13] = 0;
   out_9110126995673016572[14] = 0;
   out_9110126995673016572[15] = 1;
   out_9110126995673016572[16] = 0;
   out_9110126995673016572[17] = 0;
   out_9110126995673016572[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_9110126995673016572[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_9110126995673016572[20] = 0;
   out_9110126995673016572[21] = state[8];
   out_9110126995673016572[22] = 0;
   out_9110126995673016572[23] = -state[6];
   out_9110126995673016572[24] = -state[5];
   out_9110126995673016572[25] = 0;
   out_9110126995673016572[26] = state[3];
   out_9110126995673016572[27] = 0;
   out_9110126995673016572[28] = 0;
   out_9110126995673016572[29] = 0;
   out_9110126995673016572[30] = 0;
   out_9110126995673016572[31] = 1;
   out_9110126995673016572[32] = 0;
   out_9110126995673016572[33] = 0;
   out_9110126995673016572[34] = 1;
   out_9110126995673016572[35] = 0;
   out_9110126995673016572[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_9110126995673016572[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_9110126995673016572[38] = 0;
   out_9110126995673016572[39] = -state[7];
   out_9110126995673016572[40] = state[6];
   out_9110126995673016572[41] = 0;
   out_9110126995673016572[42] = state[4];
   out_9110126995673016572[43] = -state[3];
   out_9110126995673016572[44] = 0;
   out_9110126995673016572[45] = 0;
   out_9110126995673016572[46] = 0;
   out_9110126995673016572[47] = 0;
   out_9110126995673016572[48] = 0;
   out_9110126995673016572[49] = 0;
   out_9110126995673016572[50] = 1;
   out_9110126995673016572[51] = 0;
   out_9110126995673016572[52] = 0;
   out_9110126995673016572[53] = 1;
}
void h_13(double *state, double *unused, double *out_5562461953863127465) {
   out_5562461953863127465[0] = state[3];
   out_5562461953863127465[1] = state[4];
   out_5562461953863127465[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5692689326830211819) {
   out_5692689326830211819[0] = 0;
   out_5692689326830211819[1] = 0;
   out_5692689326830211819[2] = 0;
   out_5692689326830211819[3] = 1;
   out_5692689326830211819[4] = 0;
   out_5692689326830211819[5] = 0;
   out_5692689326830211819[6] = 0;
   out_5692689326830211819[7] = 0;
   out_5692689326830211819[8] = 0;
   out_5692689326830211819[9] = 0;
   out_5692689326830211819[10] = 0;
   out_5692689326830211819[11] = 0;
   out_5692689326830211819[12] = 0;
   out_5692689326830211819[13] = 0;
   out_5692689326830211819[14] = 0;
   out_5692689326830211819[15] = 0;
   out_5692689326830211819[16] = 0;
   out_5692689326830211819[17] = 0;
   out_5692689326830211819[18] = 0;
   out_5692689326830211819[19] = 0;
   out_5692689326830211819[20] = 0;
   out_5692689326830211819[21] = 0;
   out_5692689326830211819[22] = 1;
   out_5692689326830211819[23] = 0;
   out_5692689326830211819[24] = 0;
   out_5692689326830211819[25] = 0;
   out_5692689326830211819[26] = 0;
   out_5692689326830211819[27] = 0;
   out_5692689326830211819[28] = 0;
   out_5692689326830211819[29] = 0;
   out_5692689326830211819[30] = 0;
   out_5692689326830211819[31] = 0;
   out_5692689326830211819[32] = 0;
   out_5692689326830211819[33] = 0;
   out_5692689326830211819[34] = 0;
   out_5692689326830211819[35] = 0;
   out_5692689326830211819[36] = 0;
   out_5692689326830211819[37] = 0;
   out_5692689326830211819[38] = 0;
   out_5692689326830211819[39] = 0;
   out_5692689326830211819[40] = 0;
   out_5692689326830211819[41] = 1;
   out_5692689326830211819[42] = 0;
   out_5692689326830211819[43] = 0;
   out_5692689326830211819[44] = 0;
   out_5692689326830211819[45] = 0;
   out_5692689326830211819[46] = 0;
   out_5692689326830211819[47] = 0;
   out_5692689326830211819[48] = 0;
   out_5692689326830211819[49] = 0;
   out_5692689326830211819[50] = 0;
   out_5692689326830211819[51] = 0;
   out_5692689326830211819[52] = 0;
   out_5692689326830211819[53] = 0;
}
void h_14(double *state, double *unused, double *out_6778377519299831387) {
   out_6778377519299831387[0] = state[6];
   out_6778377519299831387[1] = state[7];
   out_6778377519299831387[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4941722295823060091) {
   out_4941722295823060091[0] = 0;
   out_4941722295823060091[1] = 0;
   out_4941722295823060091[2] = 0;
   out_4941722295823060091[3] = 0;
   out_4941722295823060091[4] = 0;
   out_4941722295823060091[5] = 0;
   out_4941722295823060091[6] = 1;
   out_4941722295823060091[7] = 0;
   out_4941722295823060091[8] = 0;
   out_4941722295823060091[9] = 0;
   out_4941722295823060091[10] = 0;
   out_4941722295823060091[11] = 0;
   out_4941722295823060091[12] = 0;
   out_4941722295823060091[13] = 0;
   out_4941722295823060091[14] = 0;
   out_4941722295823060091[15] = 0;
   out_4941722295823060091[16] = 0;
   out_4941722295823060091[17] = 0;
   out_4941722295823060091[18] = 0;
   out_4941722295823060091[19] = 0;
   out_4941722295823060091[20] = 0;
   out_4941722295823060091[21] = 0;
   out_4941722295823060091[22] = 0;
   out_4941722295823060091[23] = 0;
   out_4941722295823060091[24] = 0;
   out_4941722295823060091[25] = 1;
   out_4941722295823060091[26] = 0;
   out_4941722295823060091[27] = 0;
   out_4941722295823060091[28] = 0;
   out_4941722295823060091[29] = 0;
   out_4941722295823060091[30] = 0;
   out_4941722295823060091[31] = 0;
   out_4941722295823060091[32] = 0;
   out_4941722295823060091[33] = 0;
   out_4941722295823060091[34] = 0;
   out_4941722295823060091[35] = 0;
   out_4941722295823060091[36] = 0;
   out_4941722295823060091[37] = 0;
   out_4941722295823060091[38] = 0;
   out_4941722295823060091[39] = 0;
   out_4941722295823060091[40] = 0;
   out_4941722295823060091[41] = 0;
   out_4941722295823060091[42] = 0;
   out_4941722295823060091[43] = 0;
   out_4941722295823060091[44] = 1;
   out_4941722295823060091[45] = 0;
   out_4941722295823060091[46] = 0;
   out_4941722295823060091[47] = 0;
   out_4941722295823060091[48] = 0;
   out_4941722295823060091[49] = 0;
   out_4941722295823060091[50] = 0;
   out_4941722295823060091[51] = 0;
   out_4941722295823060091[52] = 0;
   out_4941722295823060091[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_4508435368596058991) {
  err_fun(nom_x, delta_x, out_4508435368596058991);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1037000126167514293) {
  inv_err_fun(nom_x, true_x, out_1037000126167514293);
}
void pose_H_mod_fun(double *state, double *out_4389262536507926261) {
  H_mod_fun(state, out_4389262536507926261);
}
void pose_f_fun(double *state, double dt, double *out_1664741234285638622) {
  f_fun(state,  dt, out_1664741234285638622);
}
void pose_F_fun(double *state, double dt, double *out_4647166909216110620) {
  F_fun(state,  dt, out_4647166909216110620);
}
void pose_h_4(double *state, double *unused, double *out_8166043361258674617) {
  h_4(state, unused, out_8166043361258674617);
}
void pose_H_4(double *state, double *unused, double *out_8904963152162544620) {
  H_4(state, unused, out_8904963152162544620);
}
void pose_h_10(double *state, double *unused, double *out_3269605573430160868) {
  h_10(state, unused, out_3269605573430160868);
}
void pose_H_10(double *state, double *unused, double *out_9110126995673016572) {
  H_10(state, unused, out_9110126995673016572);
}
void pose_h_13(double *state, double *unused, double *out_5562461953863127465) {
  h_13(state, unused, out_5562461953863127465);
}
void pose_H_13(double *state, double *unused, double *out_5692689326830211819) {
  H_13(state, unused, out_5692689326830211819);
}
void pose_h_14(double *state, double *unused, double *out_6778377519299831387) {
  h_14(state, unused, out_6778377519299831387);
}
void pose_H_14(double *state, double *unused, double *out_4941722295823060091) {
  H_14(state, unused, out_4941722295823060091);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
