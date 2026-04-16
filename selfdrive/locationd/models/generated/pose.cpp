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
void err_fun(double *nom_x, double *delta_x, double *out_4779041233668355035) {
   out_4779041233668355035[0] = delta_x[0] + nom_x[0];
   out_4779041233668355035[1] = delta_x[1] + nom_x[1];
   out_4779041233668355035[2] = delta_x[2] + nom_x[2];
   out_4779041233668355035[3] = delta_x[3] + nom_x[3];
   out_4779041233668355035[4] = delta_x[4] + nom_x[4];
   out_4779041233668355035[5] = delta_x[5] + nom_x[5];
   out_4779041233668355035[6] = delta_x[6] + nom_x[6];
   out_4779041233668355035[7] = delta_x[7] + nom_x[7];
   out_4779041233668355035[8] = delta_x[8] + nom_x[8];
   out_4779041233668355035[9] = delta_x[9] + nom_x[9];
   out_4779041233668355035[10] = delta_x[10] + nom_x[10];
   out_4779041233668355035[11] = delta_x[11] + nom_x[11];
   out_4779041233668355035[12] = delta_x[12] + nom_x[12];
   out_4779041233668355035[13] = delta_x[13] + nom_x[13];
   out_4779041233668355035[14] = delta_x[14] + nom_x[14];
   out_4779041233668355035[15] = delta_x[15] + nom_x[15];
   out_4779041233668355035[16] = delta_x[16] + nom_x[16];
   out_4779041233668355035[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6150076879653088119) {
   out_6150076879653088119[0] = -nom_x[0] + true_x[0];
   out_6150076879653088119[1] = -nom_x[1] + true_x[1];
   out_6150076879653088119[2] = -nom_x[2] + true_x[2];
   out_6150076879653088119[3] = -nom_x[3] + true_x[3];
   out_6150076879653088119[4] = -nom_x[4] + true_x[4];
   out_6150076879653088119[5] = -nom_x[5] + true_x[5];
   out_6150076879653088119[6] = -nom_x[6] + true_x[6];
   out_6150076879653088119[7] = -nom_x[7] + true_x[7];
   out_6150076879653088119[8] = -nom_x[8] + true_x[8];
   out_6150076879653088119[9] = -nom_x[9] + true_x[9];
   out_6150076879653088119[10] = -nom_x[10] + true_x[10];
   out_6150076879653088119[11] = -nom_x[11] + true_x[11];
   out_6150076879653088119[12] = -nom_x[12] + true_x[12];
   out_6150076879653088119[13] = -nom_x[13] + true_x[13];
   out_6150076879653088119[14] = -nom_x[14] + true_x[14];
   out_6150076879653088119[15] = -nom_x[15] + true_x[15];
   out_6150076879653088119[16] = -nom_x[16] + true_x[16];
   out_6150076879653088119[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6193522935070858792) {
   out_6193522935070858792[0] = 1.0;
   out_6193522935070858792[1] = 0.0;
   out_6193522935070858792[2] = 0.0;
   out_6193522935070858792[3] = 0.0;
   out_6193522935070858792[4] = 0.0;
   out_6193522935070858792[5] = 0.0;
   out_6193522935070858792[6] = 0.0;
   out_6193522935070858792[7] = 0.0;
   out_6193522935070858792[8] = 0.0;
   out_6193522935070858792[9] = 0.0;
   out_6193522935070858792[10] = 0.0;
   out_6193522935070858792[11] = 0.0;
   out_6193522935070858792[12] = 0.0;
   out_6193522935070858792[13] = 0.0;
   out_6193522935070858792[14] = 0.0;
   out_6193522935070858792[15] = 0.0;
   out_6193522935070858792[16] = 0.0;
   out_6193522935070858792[17] = 0.0;
   out_6193522935070858792[18] = 0.0;
   out_6193522935070858792[19] = 1.0;
   out_6193522935070858792[20] = 0.0;
   out_6193522935070858792[21] = 0.0;
   out_6193522935070858792[22] = 0.0;
   out_6193522935070858792[23] = 0.0;
   out_6193522935070858792[24] = 0.0;
   out_6193522935070858792[25] = 0.0;
   out_6193522935070858792[26] = 0.0;
   out_6193522935070858792[27] = 0.0;
   out_6193522935070858792[28] = 0.0;
   out_6193522935070858792[29] = 0.0;
   out_6193522935070858792[30] = 0.0;
   out_6193522935070858792[31] = 0.0;
   out_6193522935070858792[32] = 0.0;
   out_6193522935070858792[33] = 0.0;
   out_6193522935070858792[34] = 0.0;
   out_6193522935070858792[35] = 0.0;
   out_6193522935070858792[36] = 0.0;
   out_6193522935070858792[37] = 0.0;
   out_6193522935070858792[38] = 1.0;
   out_6193522935070858792[39] = 0.0;
   out_6193522935070858792[40] = 0.0;
   out_6193522935070858792[41] = 0.0;
   out_6193522935070858792[42] = 0.0;
   out_6193522935070858792[43] = 0.0;
   out_6193522935070858792[44] = 0.0;
   out_6193522935070858792[45] = 0.0;
   out_6193522935070858792[46] = 0.0;
   out_6193522935070858792[47] = 0.0;
   out_6193522935070858792[48] = 0.0;
   out_6193522935070858792[49] = 0.0;
   out_6193522935070858792[50] = 0.0;
   out_6193522935070858792[51] = 0.0;
   out_6193522935070858792[52] = 0.0;
   out_6193522935070858792[53] = 0.0;
   out_6193522935070858792[54] = 0.0;
   out_6193522935070858792[55] = 0.0;
   out_6193522935070858792[56] = 0.0;
   out_6193522935070858792[57] = 1.0;
   out_6193522935070858792[58] = 0.0;
   out_6193522935070858792[59] = 0.0;
   out_6193522935070858792[60] = 0.0;
   out_6193522935070858792[61] = 0.0;
   out_6193522935070858792[62] = 0.0;
   out_6193522935070858792[63] = 0.0;
   out_6193522935070858792[64] = 0.0;
   out_6193522935070858792[65] = 0.0;
   out_6193522935070858792[66] = 0.0;
   out_6193522935070858792[67] = 0.0;
   out_6193522935070858792[68] = 0.0;
   out_6193522935070858792[69] = 0.0;
   out_6193522935070858792[70] = 0.0;
   out_6193522935070858792[71] = 0.0;
   out_6193522935070858792[72] = 0.0;
   out_6193522935070858792[73] = 0.0;
   out_6193522935070858792[74] = 0.0;
   out_6193522935070858792[75] = 0.0;
   out_6193522935070858792[76] = 1.0;
   out_6193522935070858792[77] = 0.0;
   out_6193522935070858792[78] = 0.0;
   out_6193522935070858792[79] = 0.0;
   out_6193522935070858792[80] = 0.0;
   out_6193522935070858792[81] = 0.0;
   out_6193522935070858792[82] = 0.0;
   out_6193522935070858792[83] = 0.0;
   out_6193522935070858792[84] = 0.0;
   out_6193522935070858792[85] = 0.0;
   out_6193522935070858792[86] = 0.0;
   out_6193522935070858792[87] = 0.0;
   out_6193522935070858792[88] = 0.0;
   out_6193522935070858792[89] = 0.0;
   out_6193522935070858792[90] = 0.0;
   out_6193522935070858792[91] = 0.0;
   out_6193522935070858792[92] = 0.0;
   out_6193522935070858792[93] = 0.0;
   out_6193522935070858792[94] = 0.0;
   out_6193522935070858792[95] = 1.0;
   out_6193522935070858792[96] = 0.0;
   out_6193522935070858792[97] = 0.0;
   out_6193522935070858792[98] = 0.0;
   out_6193522935070858792[99] = 0.0;
   out_6193522935070858792[100] = 0.0;
   out_6193522935070858792[101] = 0.0;
   out_6193522935070858792[102] = 0.0;
   out_6193522935070858792[103] = 0.0;
   out_6193522935070858792[104] = 0.0;
   out_6193522935070858792[105] = 0.0;
   out_6193522935070858792[106] = 0.0;
   out_6193522935070858792[107] = 0.0;
   out_6193522935070858792[108] = 0.0;
   out_6193522935070858792[109] = 0.0;
   out_6193522935070858792[110] = 0.0;
   out_6193522935070858792[111] = 0.0;
   out_6193522935070858792[112] = 0.0;
   out_6193522935070858792[113] = 0.0;
   out_6193522935070858792[114] = 1.0;
   out_6193522935070858792[115] = 0.0;
   out_6193522935070858792[116] = 0.0;
   out_6193522935070858792[117] = 0.0;
   out_6193522935070858792[118] = 0.0;
   out_6193522935070858792[119] = 0.0;
   out_6193522935070858792[120] = 0.0;
   out_6193522935070858792[121] = 0.0;
   out_6193522935070858792[122] = 0.0;
   out_6193522935070858792[123] = 0.0;
   out_6193522935070858792[124] = 0.0;
   out_6193522935070858792[125] = 0.0;
   out_6193522935070858792[126] = 0.0;
   out_6193522935070858792[127] = 0.0;
   out_6193522935070858792[128] = 0.0;
   out_6193522935070858792[129] = 0.0;
   out_6193522935070858792[130] = 0.0;
   out_6193522935070858792[131] = 0.0;
   out_6193522935070858792[132] = 0.0;
   out_6193522935070858792[133] = 1.0;
   out_6193522935070858792[134] = 0.0;
   out_6193522935070858792[135] = 0.0;
   out_6193522935070858792[136] = 0.0;
   out_6193522935070858792[137] = 0.0;
   out_6193522935070858792[138] = 0.0;
   out_6193522935070858792[139] = 0.0;
   out_6193522935070858792[140] = 0.0;
   out_6193522935070858792[141] = 0.0;
   out_6193522935070858792[142] = 0.0;
   out_6193522935070858792[143] = 0.0;
   out_6193522935070858792[144] = 0.0;
   out_6193522935070858792[145] = 0.0;
   out_6193522935070858792[146] = 0.0;
   out_6193522935070858792[147] = 0.0;
   out_6193522935070858792[148] = 0.0;
   out_6193522935070858792[149] = 0.0;
   out_6193522935070858792[150] = 0.0;
   out_6193522935070858792[151] = 0.0;
   out_6193522935070858792[152] = 1.0;
   out_6193522935070858792[153] = 0.0;
   out_6193522935070858792[154] = 0.0;
   out_6193522935070858792[155] = 0.0;
   out_6193522935070858792[156] = 0.0;
   out_6193522935070858792[157] = 0.0;
   out_6193522935070858792[158] = 0.0;
   out_6193522935070858792[159] = 0.0;
   out_6193522935070858792[160] = 0.0;
   out_6193522935070858792[161] = 0.0;
   out_6193522935070858792[162] = 0.0;
   out_6193522935070858792[163] = 0.0;
   out_6193522935070858792[164] = 0.0;
   out_6193522935070858792[165] = 0.0;
   out_6193522935070858792[166] = 0.0;
   out_6193522935070858792[167] = 0.0;
   out_6193522935070858792[168] = 0.0;
   out_6193522935070858792[169] = 0.0;
   out_6193522935070858792[170] = 0.0;
   out_6193522935070858792[171] = 1.0;
   out_6193522935070858792[172] = 0.0;
   out_6193522935070858792[173] = 0.0;
   out_6193522935070858792[174] = 0.0;
   out_6193522935070858792[175] = 0.0;
   out_6193522935070858792[176] = 0.0;
   out_6193522935070858792[177] = 0.0;
   out_6193522935070858792[178] = 0.0;
   out_6193522935070858792[179] = 0.0;
   out_6193522935070858792[180] = 0.0;
   out_6193522935070858792[181] = 0.0;
   out_6193522935070858792[182] = 0.0;
   out_6193522935070858792[183] = 0.0;
   out_6193522935070858792[184] = 0.0;
   out_6193522935070858792[185] = 0.0;
   out_6193522935070858792[186] = 0.0;
   out_6193522935070858792[187] = 0.0;
   out_6193522935070858792[188] = 0.0;
   out_6193522935070858792[189] = 0.0;
   out_6193522935070858792[190] = 1.0;
   out_6193522935070858792[191] = 0.0;
   out_6193522935070858792[192] = 0.0;
   out_6193522935070858792[193] = 0.0;
   out_6193522935070858792[194] = 0.0;
   out_6193522935070858792[195] = 0.0;
   out_6193522935070858792[196] = 0.0;
   out_6193522935070858792[197] = 0.0;
   out_6193522935070858792[198] = 0.0;
   out_6193522935070858792[199] = 0.0;
   out_6193522935070858792[200] = 0.0;
   out_6193522935070858792[201] = 0.0;
   out_6193522935070858792[202] = 0.0;
   out_6193522935070858792[203] = 0.0;
   out_6193522935070858792[204] = 0.0;
   out_6193522935070858792[205] = 0.0;
   out_6193522935070858792[206] = 0.0;
   out_6193522935070858792[207] = 0.0;
   out_6193522935070858792[208] = 0.0;
   out_6193522935070858792[209] = 1.0;
   out_6193522935070858792[210] = 0.0;
   out_6193522935070858792[211] = 0.0;
   out_6193522935070858792[212] = 0.0;
   out_6193522935070858792[213] = 0.0;
   out_6193522935070858792[214] = 0.0;
   out_6193522935070858792[215] = 0.0;
   out_6193522935070858792[216] = 0.0;
   out_6193522935070858792[217] = 0.0;
   out_6193522935070858792[218] = 0.0;
   out_6193522935070858792[219] = 0.0;
   out_6193522935070858792[220] = 0.0;
   out_6193522935070858792[221] = 0.0;
   out_6193522935070858792[222] = 0.0;
   out_6193522935070858792[223] = 0.0;
   out_6193522935070858792[224] = 0.0;
   out_6193522935070858792[225] = 0.0;
   out_6193522935070858792[226] = 0.0;
   out_6193522935070858792[227] = 0.0;
   out_6193522935070858792[228] = 1.0;
   out_6193522935070858792[229] = 0.0;
   out_6193522935070858792[230] = 0.0;
   out_6193522935070858792[231] = 0.0;
   out_6193522935070858792[232] = 0.0;
   out_6193522935070858792[233] = 0.0;
   out_6193522935070858792[234] = 0.0;
   out_6193522935070858792[235] = 0.0;
   out_6193522935070858792[236] = 0.0;
   out_6193522935070858792[237] = 0.0;
   out_6193522935070858792[238] = 0.0;
   out_6193522935070858792[239] = 0.0;
   out_6193522935070858792[240] = 0.0;
   out_6193522935070858792[241] = 0.0;
   out_6193522935070858792[242] = 0.0;
   out_6193522935070858792[243] = 0.0;
   out_6193522935070858792[244] = 0.0;
   out_6193522935070858792[245] = 0.0;
   out_6193522935070858792[246] = 0.0;
   out_6193522935070858792[247] = 1.0;
   out_6193522935070858792[248] = 0.0;
   out_6193522935070858792[249] = 0.0;
   out_6193522935070858792[250] = 0.0;
   out_6193522935070858792[251] = 0.0;
   out_6193522935070858792[252] = 0.0;
   out_6193522935070858792[253] = 0.0;
   out_6193522935070858792[254] = 0.0;
   out_6193522935070858792[255] = 0.0;
   out_6193522935070858792[256] = 0.0;
   out_6193522935070858792[257] = 0.0;
   out_6193522935070858792[258] = 0.0;
   out_6193522935070858792[259] = 0.0;
   out_6193522935070858792[260] = 0.0;
   out_6193522935070858792[261] = 0.0;
   out_6193522935070858792[262] = 0.0;
   out_6193522935070858792[263] = 0.0;
   out_6193522935070858792[264] = 0.0;
   out_6193522935070858792[265] = 0.0;
   out_6193522935070858792[266] = 1.0;
   out_6193522935070858792[267] = 0.0;
   out_6193522935070858792[268] = 0.0;
   out_6193522935070858792[269] = 0.0;
   out_6193522935070858792[270] = 0.0;
   out_6193522935070858792[271] = 0.0;
   out_6193522935070858792[272] = 0.0;
   out_6193522935070858792[273] = 0.0;
   out_6193522935070858792[274] = 0.0;
   out_6193522935070858792[275] = 0.0;
   out_6193522935070858792[276] = 0.0;
   out_6193522935070858792[277] = 0.0;
   out_6193522935070858792[278] = 0.0;
   out_6193522935070858792[279] = 0.0;
   out_6193522935070858792[280] = 0.0;
   out_6193522935070858792[281] = 0.0;
   out_6193522935070858792[282] = 0.0;
   out_6193522935070858792[283] = 0.0;
   out_6193522935070858792[284] = 0.0;
   out_6193522935070858792[285] = 1.0;
   out_6193522935070858792[286] = 0.0;
   out_6193522935070858792[287] = 0.0;
   out_6193522935070858792[288] = 0.0;
   out_6193522935070858792[289] = 0.0;
   out_6193522935070858792[290] = 0.0;
   out_6193522935070858792[291] = 0.0;
   out_6193522935070858792[292] = 0.0;
   out_6193522935070858792[293] = 0.0;
   out_6193522935070858792[294] = 0.0;
   out_6193522935070858792[295] = 0.0;
   out_6193522935070858792[296] = 0.0;
   out_6193522935070858792[297] = 0.0;
   out_6193522935070858792[298] = 0.0;
   out_6193522935070858792[299] = 0.0;
   out_6193522935070858792[300] = 0.0;
   out_6193522935070858792[301] = 0.0;
   out_6193522935070858792[302] = 0.0;
   out_6193522935070858792[303] = 0.0;
   out_6193522935070858792[304] = 1.0;
   out_6193522935070858792[305] = 0.0;
   out_6193522935070858792[306] = 0.0;
   out_6193522935070858792[307] = 0.0;
   out_6193522935070858792[308] = 0.0;
   out_6193522935070858792[309] = 0.0;
   out_6193522935070858792[310] = 0.0;
   out_6193522935070858792[311] = 0.0;
   out_6193522935070858792[312] = 0.0;
   out_6193522935070858792[313] = 0.0;
   out_6193522935070858792[314] = 0.0;
   out_6193522935070858792[315] = 0.0;
   out_6193522935070858792[316] = 0.0;
   out_6193522935070858792[317] = 0.0;
   out_6193522935070858792[318] = 0.0;
   out_6193522935070858792[319] = 0.0;
   out_6193522935070858792[320] = 0.0;
   out_6193522935070858792[321] = 0.0;
   out_6193522935070858792[322] = 0.0;
   out_6193522935070858792[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3399450163802279159) {
   out_3399450163802279159[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3399450163802279159[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3399450163802279159[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3399450163802279159[3] = dt*state[12] + state[3];
   out_3399450163802279159[4] = dt*state[13] + state[4];
   out_3399450163802279159[5] = dt*state[14] + state[5];
   out_3399450163802279159[6] = state[6];
   out_3399450163802279159[7] = state[7];
   out_3399450163802279159[8] = state[8];
   out_3399450163802279159[9] = state[9];
   out_3399450163802279159[10] = state[10];
   out_3399450163802279159[11] = state[11];
   out_3399450163802279159[12] = state[12];
   out_3399450163802279159[13] = state[13];
   out_3399450163802279159[14] = state[14];
   out_3399450163802279159[15] = state[15];
   out_3399450163802279159[16] = state[16];
   out_3399450163802279159[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7117793343438049369) {
   out_7117793343438049369[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7117793343438049369[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7117793343438049369[2] = 0;
   out_7117793343438049369[3] = 0;
   out_7117793343438049369[4] = 0;
   out_7117793343438049369[5] = 0;
   out_7117793343438049369[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7117793343438049369[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7117793343438049369[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7117793343438049369[9] = 0;
   out_7117793343438049369[10] = 0;
   out_7117793343438049369[11] = 0;
   out_7117793343438049369[12] = 0;
   out_7117793343438049369[13] = 0;
   out_7117793343438049369[14] = 0;
   out_7117793343438049369[15] = 0;
   out_7117793343438049369[16] = 0;
   out_7117793343438049369[17] = 0;
   out_7117793343438049369[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7117793343438049369[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7117793343438049369[20] = 0;
   out_7117793343438049369[21] = 0;
   out_7117793343438049369[22] = 0;
   out_7117793343438049369[23] = 0;
   out_7117793343438049369[24] = 0;
   out_7117793343438049369[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7117793343438049369[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7117793343438049369[27] = 0;
   out_7117793343438049369[28] = 0;
   out_7117793343438049369[29] = 0;
   out_7117793343438049369[30] = 0;
   out_7117793343438049369[31] = 0;
   out_7117793343438049369[32] = 0;
   out_7117793343438049369[33] = 0;
   out_7117793343438049369[34] = 0;
   out_7117793343438049369[35] = 0;
   out_7117793343438049369[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7117793343438049369[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7117793343438049369[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7117793343438049369[39] = 0;
   out_7117793343438049369[40] = 0;
   out_7117793343438049369[41] = 0;
   out_7117793343438049369[42] = 0;
   out_7117793343438049369[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7117793343438049369[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7117793343438049369[45] = 0;
   out_7117793343438049369[46] = 0;
   out_7117793343438049369[47] = 0;
   out_7117793343438049369[48] = 0;
   out_7117793343438049369[49] = 0;
   out_7117793343438049369[50] = 0;
   out_7117793343438049369[51] = 0;
   out_7117793343438049369[52] = 0;
   out_7117793343438049369[53] = 0;
   out_7117793343438049369[54] = 0;
   out_7117793343438049369[55] = 0;
   out_7117793343438049369[56] = 0;
   out_7117793343438049369[57] = 1;
   out_7117793343438049369[58] = 0;
   out_7117793343438049369[59] = 0;
   out_7117793343438049369[60] = 0;
   out_7117793343438049369[61] = 0;
   out_7117793343438049369[62] = 0;
   out_7117793343438049369[63] = 0;
   out_7117793343438049369[64] = 0;
   out_7117793343438049369[65] = 0;
   out_7117793343438049369[66] = dt;
   out_7117793343438049369[67] = 0;
   out_7117793343438049369[68] = 0;
   out_7117793343438049369[69] = 0;
   out_7117793343438049369[70] = 0;
   out_7117793343438049369[71] = 0;
   out_7117793343438049369[72] = 0;
   out_7117793343438049369[73] = 0;
   out_7117793343438049369[74] = 0;
   out_7117793343438049369[75] = 0;
   out_7117793343438049369[76] = 1;
   out_7117793343438049369[77] = 0;
   out_7117793343438049369[78] = 0;
   out_7117793343438049369[79] = 0;
   out_7117793343438049369[80] = 0;
   out_7117793343438049369[81] = 0;
   out_7117793343438049369[82] = 0;
   out_7117793343438049369[83] = 0;
   out_7117793343438049369[84] = 0;
   out_7117793343438049369[85] = dt;
   out_7117793343438049369[86] = 0;
   out_7117793343438049369[87] = 0;
   out_7117793343438049369[88] = 0;
   out_7117793343438049369[89] = 0;
   out_7117793343438049369[90] = 0;
   out_7117793343438049369[91] = 0;
   out_7117793343438049369[92] = 0;
   out_7117793343438049369[93] = 0;
   out_7117793343438049369[94] = 0;
   out_7117793343438049369[95] = 1;
   out_7117793343438049369[96] = 0;
   out_7117793343438049369[97] = 0;
   out_7117793343438049369[98] = 0;
   out_7117793343438049369[99] = 0;
   out_7117793343438049369[100] = 0;
   out_7117793343438049369[101] = 0;
   out_7117793343438049369[102] = 0;
   out_7117793343438049369[103] = 0;
   out_7117793343438049369[104] = dt;
   out_7117793343438049369[105] = 0;
   out_7117793343438049369[106] = 0;
   out_7117793343438049369[107] = 0;
   out_7117793343438049369[108] = 0;
   out_7117793343438049369[109] = 0;
   out_7117793343438049369[110] = 0;
   out_7117793343438049369[111] = 0;
   out_7117793343438049369[112] = 0;
   out_7117793343438049369[113] = 0;
   out_7117793343438049369[114] = 1;
   out_7117793343438049369[115] = 0;
   out_7117793343438049369[116] = 0;
   out_7117793343438049369[117] = 0;
   out_7117793343438049369[118] = 0;
   out_7117793343438049369[119] = 0;
   out_7117793343438049369[120] = 0;
   out_7117793343438049369[121] = 0;
   out_7117793343438049369[122] = 0;
   out_7117793343438049369[123] = 0;
   out_7117793343438049369[124] = 0;
   out_7117793343438049369[125] = 0;
   out_7117793343438049369[126] = 0;
   out_7117793343438049369[127] = 0;
   out_7117793343438049369[128] = 0;
   out_7117793343438049369[129] = 0;
   out_7117793343438049369[130] = 0;
   out_7117793343438049369[131] = 0;
   out_7117793343438049369[132] = 0;
   out_7117793343438049369[133] = 1;
   out_7117793343438049369[134] = 0;
   out_7117793343438049369[135] = 0;
   out_7117793343438049369[136] = 0;
   out_7117793343438049369[137] = 0;
   out_7117793343438049369[138] = 0;
   out_7117793343438049369[139] = 0;
   out_7117793343438049369[140] = 0;
   out_7117793343438049369[141] = 0;
   out_7117793343438049369[142] = 0;
   out_7117793343438049369[143] = 0;
   out_7117793343438049369[144] = 0;
   out_7117793343438049369[145] = 0;
   out_7117793343438049369[146] = 0;
   out_7117793343438049369[147] = 0;
   out_7117793343438049369[148] = 0;
   out_7117793343438049369[149] = 0;
   out_7117793343438049369[150] = 0;
   out_7117793343438049369[151] = 0;
   out_7117793343438049369[152] = 1;
   out_7117793343438049369[153] = 0;
   out_7117793343438049369[154] = 0;
   out_7117793343438049369[155] = 0;
   out_7117793343438049369[156] = 0;
   out_7117793343438049369[157] = 0;
   out_7117793343438049369[158] = 0;
   out_7117793343438049369[159] = 0;
   out_7117793343438049369[160] = 0;
   out_7117793343438049369[161] = 0;
   out_7117793343438049369[162] = 0;
   out_7117793343438049369[163] = 0;
   out_7117793343438049369[164] = 0;
   out_7117793343438049369[165] = 0;
   out_7117793343438049369[166] = 0;
   out_7117793343438049369[167] = 0;
   out_7117793343438049369[168] = 0;
   out_7117793343438049369[169] = 0;
   out_7117793343438049369[170] = 0;
   out_7117793343438049369[171] = 1;
   out_7117793343438049369[172] = 0;
   out_7117793343438049369[173] = 0;
   out_7117793343438049369[174] = 0;
   out_7117793343438049369[175] = 0;
   out_7117793343438049369[176] = 0;
   out_7117793343438049369[177] = 0;
   out_7117793343438049369[178] = 0;
   out_7117793343438049369[179] = 0;
   out_7117793343438049369[180] = 0;
   out_7117793343438049369[181] = 0;
   out_7117793343438049369[182] = 0;
   out_7117793343438049369[183] = 0;
   out_7117793343438049369[184] = 0;
   out_7117793343438049369[185] = 0;
   out_7117793343438049369[186] = 0;
   out_7117793343438049369[187] = 0;
   out_7117793343438049369[188] = 0;
   out_7117793343438049369[189] = 0;
   out_7117793343438049369[190] = 1;
   out_7117793343438049369[191] = 0;
   out_7117793343438049369[192] = 0;
   out_7117793343438049369[193] = 0;
   out_7117793343438049369[194] = 0;
   out_7117793343438049369[195] = 0;
   out_7117793343438049369[196] = 0;
   out_7117793343438049369[197] = 0;
   out_7117793343438049369[198] = 0;
   out_7117793343438049369[199] = 0;
   out_7117793343438049369[200] = 0;
   out_7117793343438049369[201] = 0;
   out_7117793343438049369[202] = 0;
   out_7117793343438049369[203] = 0;
   out_7117793343438049369[204] = 0;
   out_7117793343438049369[205] = 0;
   out_7117793343438049369[206] = 0;
   out_7117793343438049369[207] = 0;
   out_7117793343438049369[208] = 0;
   out_7117793343438049369[209] = 1;
   out_7117793343438049369[210] = 0;
   out_7117793343438049369[211] = 0;
   out_7117793343438049369[212] = 0;
   out_7117793343438049369[213] = 0;
   out_7117793343438049369[214] = 0;
   out_7117793343438049369[215] = 0;
   out_7117793343438049369[216] = 0;
   out_7117793343438049369[217] = 0;
   out_7117793343438049369[218] = 0;
   out_7117793343438049369[219] = 0;
   out_7117793343438049369[220] = 0;
   out_7117793343438049369[221] = 0;
   out_7117793343438049369[222] = 0;
   out_7117793343438049369[223] = 0;
   out_7117793343438049369[224] = 0;
   out_7117793343438049369[225] = 0;
   out_7117793343438049369[226] = 0;
   out_7117793343438049369[227] = 0;
   out_7117793343438049369[228] = 1;
   out_7117793343438049369[229] = 0;
   out_7117793343438049369[230] = 0;
   out_7117793343438049369[231] = 0;
   out_7117793343438049369[232] = 0;
   out_7117793343438049369[233] = 0;
   out_7117793343438049369[234] = 0;
   out_7117793343438049369[235] = 0;
   out_7117793343438049369[236] = 0;
   out_7117793343438049369[237] = 0;
   out_7117793343438049369[238] = 0;
   out_7117793343438049369[239] = 0;
   out_7117793343438049369[240] = 0;
   out_7117793343438049369[241] = 0;
   out_7117793343438049369[242] = 0;
   out_7117793343438049369[243] = 0;
   out_7117793343438049369[244] = 0;
   out_7117793343438049369[245] = 0;
   out_7117793343438049369[246] = 0;
   out_7117793343438049369[247] = 1;
   out_7117793343438049369[248] = 0;
   out_7117793343438049369[249] = 0;
   out_7117793343438049369[250] = 0;
   out_7117793343438049369[251] = 0;
   out_7117793343438049369[252] = 0;
   out_7117793343438049369[253] = 0;
   out_7117793343438049369[254] = 0;
   out_7117793343438049369[255] = 0;
   out_7117793343438049369[256] = 0;
   out_7117793343438049369[257] = 0;
   out_7117793343438049369[258] = 0;
   out_7117793343438049369[259] = 0;
   out_7117793343438049369[260] = 0;
   out_7117793343438049369[261] = 0;
   out_7117793343438049369[262] = 0;
   out_7117793343438049369[263] = 0;
   out_7117793343438049369[264] = 0;
   out_7117793343438049369[265] = 0;
   out_7117793343438049369[266] = 1;
   out_7117793343438049369[267] = 0;
   out_7117793343438049369[268] = 0;
   out_7117793343438049369[269] = 0;
   out_7117793343438049369[270] = 0;
   out_7117793343438049369[271] = 0;
   out_7117793343438049369[272] = 0;
   out_7117793343438049369[273] = 0;
   out_7117793343438049369[274] = 0;
   out_7117793343438049369[275] = 0;
   out_7117793343438049369[276] = 0;
   out_7117793343438049369[277] = 0;
   out_7117793343438049369[278] = 0;
   out_7117793343438049369[279] = 0;
   out_7117793343438049369[280] = 0;
   out_7117793343438049369[281] = 0;
   out_7117793343438049369[282] = 0;
   out_7117793343438049369[283] = 0;
   out_7117793343438049369[284] = 0;
   out_7117793343438049369[285] = 1;
   out_7117793343438049369[286] = 0;
   out_7117793343438049369[287] = 0;
   out_7117793343438049369[288] = 0;
   out_7117793343438049369[289] = 0;
   out_7117793343438049369[290] = 0;
   out_7117793343438049369[291] = 0;
   out_7117793343438049369[292] = 0;
   out_7117793343438049369[293] = 0;
   out_7117793343438049369[294] = 0;
   out_7117793343438049369[295] = 0;
   out_7117793343438049369[296] = 0;
   out_7117793343438049369[297] = 0;
   out_7117793343438049369[298] = 0;
   out_7117793343438049369[299] = 0;
   out_7117793343438049369[300] = 0;
   out_7117793343438049369[301] = 0;
   out_7117793343438049369[302] = 0;
   out_7117793343438049369[303] = 0;
   out_7117793343438049369[304] = 1;
   out_7117793343438049369[305] = 0;
   out_7117793343438049369[306] = 0;
   out_7117793343438049369[307] = 0;
   out_7117793343438049369[308] = 0;
   out_7117793343438049369[309] = 0;
   out_7117793343438049369[310] = 0;
   out_7117793343438049369[311] = 0;
   out_7117793343438049369[312] = 0;
   out_7117793343438049369[313] = 0;
   out_7117793343438049369[314] = 0;
   out_7117793343438049369[315] = 0;
   out_7117793343438049369[316] = 0;
   out_7117793343438049369[317] = 0;
   out_7117793343438049369[318] = 0;
   out_7117793343438049369[319] = 0;
   out_7117793343438049369[320] = 0;
   out_7117793343438049369[321] = 0;
   out_7117793343438049369[322] = 0;
   out_7117793343438049369[323] = 1;
}
void h_4(double *state, double *unused, double *out_7151678483254898760) {
   out_7151678483254898760[0] = state[6] + state[9];
   out_7151678483254898760[1] = state[7] + state[10];
   out_7151678483254898760[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5324698458947972018) {
   out_5324698458947972018[0] = 0;
   out_5324698458947972018[1] = 0;
   out_5324698458947972018[2] = 0;
   out_5324698458947972018[3] = 0;
   out_5324698458947972018[4] = 0;
   out_5324698458947972018[5] = 0;
   out_5324698458947972018[6] = 1;
   out_5324698458947972018[7] = 0;
   out_5324698458947972018[8] = 0;
   out_5324698458947972018[9] = 1;
   out_5324698458947972018[10] = 0;
   out_5324698458947972018[11] = 0;
   out_5324698458947972018[12] = 0;
   out_5324698458947972018[13] = 0;
   out_5324698458947972018[14] = 0;
   out_5324698458947972018[15] = 0;
   out_5324698458947972018[16] = 0;
   out_5324698458947972018[17] = 0;
   out_5324698458947972018[18] = 0;
   out_5324698458947972018[19] = 0;
   out_5324698458947972018[20] = 0;
   out_5324698458947972018[21] = 0;
   out_5324698458947972018[22] = 0;
   out_5324698458947972018[23] = 0;
   out_5324698458947972018[24] = 0;
   out_5324698458947972018[25] = 1;
   out_5324698458947972018[26] = 0;
   out_5324698458947972018[27] = 0;
   out_5324698458947972018[28] = 1;
   out_5324698458947972018[29] = 0;
   out_5324698458947972018[30] = 0;
   out_5324698458947972018[31] = 0;
   out_5324698458947972018[32] = 0;
   out_5324698458947972018[33] = 0;
   out_5324698458947972018[34] = 0;
   out_5324698458947972018[35] = 0;
   out_5324698458947972018[36] = 0;
   out_5324698458947972018[37] = 0;
   out_5324698458947972018[38] = 0;
   out_5324698458947972018[39] = 0;
   out_5324698458947972018[40] = 0;
   out_5324698458947972018[41] = 0;
   out_5324698458947972018[42] = 0;
   out_5324698458947972018[43] = 0;
   out_5324698458947972018[44] = 1;
   out_5324698458947972018[45] = 0;
   out_5324698458947972018[46] = 0;
   out_5324698458947972018[47] = 1;
   out_5324698458947972018[48] = 0;
   out_5324698458947972018[49] = 0;
   out_5324698458947972018[50] = 0;
   out_5324698458947972018[51] = 0;
   out_5324698458947972018[52] = 0;
   out_5324698458947972018[53] = 0;
}
void h_10(double *state, double *unused, double *out_2988941107398924586) {
   out_2988941107398924586[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2988941107398924586[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2988941107398924586[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7064205239493264161) {
   out_7064205239493264161[0] = 0;
   out_7064205239493264161[1] = 9.8100000000000005*cos(state[1]);
   out_7064205239493264161[2] = 0;
   out_7064205239493264161[3] = 0;
   out_7064205239493264161[4] = -state[8];
   out_7064205239493264161[5] = state[7];
   out_7064205239493264161[6] = 0;
   out_7064205239493264161[7] = state[5];
   out_7064205239493264161[8] = -state[4];
   out_7064205239493264161[9] = 0;
   out_7064205239493264161[10] = 0;
   out_7064205239493264161[11] = 0;
   out_7064205239493264161[12] = 1;
   out_7064205239493264161[13] = 0;
   out_7064205239493264161[14] = 0;
   out_7064205239493264161[15] = 1;
   out_7064205239493264161[16] = 0;
   out_7064205239493264161[17] = 0;
   out_7064205239493264161[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7064205239493264161[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7064205239493264161[20] = 0;
   out_7064205239493264161[21] = state[8];
   out_7064205239493264161[22] = 0;
   out_7064205239493264161[23] = -state[6];
   out_7064205239493264161[24] = -state[5];
   out_7064205239493264161[25] = 0;
   out_7064205239493264161[26] = state[3];
   out_7064205239493264161[27] = 0;
   out_7064205239493264161[28] = 0;
   out_7064205239493264161[29] = 0;
   out_7064205239493264161[30] = 0;
   out_7064205239493264161[31] = 1;
   out_7064205239493264161[32] = 0;
   out_7064205239493264161[33] = 0;
   out_7064205239493264161[34] = 1;
   out_7064205239493264161[35] = 0;
   out_7064205239493264161[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7064205239493264161[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7064205239493264161[38] = 0;
   out_7064205239493264161[39] = -state[7];
   out_7064205239493264161[40] = state[6];
   out_7064205239493264161[41] = 0;
   out_7064205239493264161[42] = state[4];
   out_7064205239493264161[43] = -state[3];
   out_7064205239493264161[44] = 0;
   out_7064205239493264161[45] = 0;
   out_7064205239493264161[46] = 0;
   out_7064205239493264161[47] = 0;
   out_7064205239493264161[48] = 0;
   out_7064205239493264161[49] = 0;
   out_7064205239493264161[50] = 1;
   out_7064205239493264161[51] = 0;
   out_7064205239493264161[52] = 0;
   out_7064205239493264161[53] = 1;
}
void h_13(double *state, double *unused, double *out_8132861882635179526) {
   out_8132861882635179526[0] = state[3];
   out_8132861882635179526[1] = state[4];
   out_8132861882635179526[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2112424633615639217) {
   out_2112424633615639217[0] = 0;
   out_2112424633615639217[1] = 0;
   out_2112424633615639217[2] = 0;
   out_2112424633615639217[3] = 1;
   out_2112424633615639217[4] = 0;
   out_2112424633615639217[5] = 0;
   out_2112424633615639217[6] = 0;
   out_2112424633615639217[7] = 0;
   out_2112424633615639217[8] = 0;
   out_2112424633615639217[9] = 0;
   out_2112424633615639217[10] = 0;
   out_2112424633615639217[11] = 0;
   out_2112424633615639217[12] = 0;
   out_2112424633615639217[13] = 0;
   out_2112424633615639217[14] = 0;
   out_2112424633615639217[15] = 0;
   out_2112424633615639217[16] = 0;
   out_2112424633615639217[17] = 0;
   out_2112424633615639217[18] = 0;
   out_2112424633615639217[19] = 0;
   out_2112424633615639217[20] = 0;
   out_2112424633615639217[21] = 0;
   out_2112424633615639217[22] = 1;
   out_2112424633615639217[23] = 0;
   out_2112424633615639217[24] = 0;
   out_2112424633615639217[25] = 0;
   out_2112424633615639217[26] = 0;
   out_2112424633615639217[27] = 0;
   out_2112424633615639217[28] = 0;
   out_2112424633615639217[29] = 0;
   out_2112424633615639217[30] = 0;
   out_2112424633615639217[31] = 0;
   out_2112424633615639217[32] = 0;
   out_2112424633615639217[33] = 0;
   out_2112424633615639217[34] = 0;
   out_2112424633615639217[35] = 0;
   out_2112424633615639217[36] = 0;
   out_2112424633615639217[37] = 0;
   out_2112424633615639217[38] = 0;
   out_2112424633615639217[39] = 0;
   out_2112424633615639217[40] = 0;
   out_2112424633615639217[41] = 1;
   out_2112424633615639217[42] = 0;
   out_2112424633615639217[43] = 0;
   out_2112424633615639217[44] = 0;
   out_2112424633615639217[45] = 0;
   out_2112424633615639217[46] = 0;
   out_2112424633615639217[47] = 0;
   out_2112424633615639217[48] = 0;
   out_2112424633615639217[49] = 0;
   out_2112424633615639217[50] = 0;
   out_2112424633615639217[51] = 0;
   out_2112424633615639217[52] = 0;
   out_2112424633615639217[53] = 0;
}
void h_14(double *state, double *unused, double *out_7493722489878846780) {
   out_7493722489878846780[0] = state[6];
   out_7493722489878846780[1] = state[7];
   out_7493722489878846780[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5640899799481839174) {
   out_5640899799481839174[0] = 0;
   out_5640899799481839174[1] = 0;
   out_5640899799481839174[2] = 0;
   out_5640899799481839174[3] = 0;
   out_5640899799481839174[4] = 0;
   out_5640899799481839174[5] = 0;
   out_5640899799481839174[6] = 1;
   out_5640899799481839174[7] = 0;
   out_5640899799481839174[8] = 0;
   out_5640899799481839174[9] = 0;
   out_5640899799481839174[10] = 0;
   out_5640899799481839174[11] = 0;
   out_5640899799481839174[12] = 0;
   out_5640899799481839174[13] = 0;
   out_5640899799481839174[14] = 0;
   out_5640899799481839174[15] = 0;
   out_5640899799481839174[16] = 0;
   out_5640899799481839174[17] = 0;
   out_5640899799481839174[18] = 0;
   out_5640899799481839174[19] = 0;
   out_5640899799481839174[20] = 0;
   out_5640899799481839174[21] = 0;
   out_5640899799481839174[22] = 0;
   out_5640899799481839174[23] = 0;
   out_5640899799481839174[24] = 0;
   out_5640899799481839174[25] = 1;
   out_5640899799481839174[26] = 0;
   out_5640899799481839174[27] = 0;
   out_5640899799481839174[28] = 0;
   out_5640899799481839174[29] = 0;
   out_5640899799481839174[30] = 0;
   out_5640899799481839174[31] = 0;
   out_5640899799481839174[32] = 0;
   out_5640899799481839174[33] = 0;
   out_5640899799481839174[34] = 0;
   out_5640899799481839174[35] = 0;
   out_5640899799481839174[36] = 0;
   out_5640899799481839174[37] = 0;
   out_5640899799481839174[38] = 0;
   out_5640899799481839174[39] = 0;
   out_5640899799481839174[40] = 0;
   out_5640899799481839174[41] = 0;
   out_5640899799481839174[42] = 0;
   out_5640899799481839174[43] = 0;
   out_5640899799481839174[44] = 1;
   out_5640899799481839174[45] = 0;
   out_5640899799481839174[46] = 0;
   out_5640899799481839174[47] = 0;
   out_5640899799481839174[48] = 0;
   out_5640899799481839174[49] = 0;
   out_5640899799481839174[50] = 0;
   out_5640899799481839174[51] = 0;
   out_5640899799481839174[52] = 0;
   out_5640899799481839174[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4779041233668355035) {
  err_fun(nom_x, delta_x, out_4779041233668355035);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6150076879653088119) {
  inv_err_fun(nom_x, true_x, out_6150076879653088119);
}
void pose_H_mod_fun(double *state, double *out_6193522935070858792) {
  H_mod_fun(state, out_6193522935070858792);
}
void pose_f_fun(double *state, double dt, double *out_3399450163802279159) {
  f_fun(state,  dt, out_3399450163802279159);
}
void pose_F_fun(double *state, double dt, double *out_7117793343438049369) {
  F_fun(state,  dt, out_7117793343438049369);
}
void pose_h_4(double *state, double *unused, double *out_7151678483254898760) {
  h_4(state, unused, out_7151678483254898760);
}
void pose_H_4(double *state, double *unused, double *out_5324698458947972018) {
  H_4(state, unused, out_5324698458947972018);
}
void pose_h_10(double *state, double *unused, double *out_2988941107398924586) {
  h_10(state, unused, out_2988941107398924586);
}
void pose_H_10(double *state, double *unused, double *out_7064205239493264161) {
  H_10(state, unused, out_7064205239493264161);
}
void pose_h_13(double *state, double *unused, double *out_8132861882635179526) {
  h_13(state, unused, out_8132861882635179526);
}
void pose_H_13(double *state, double *unused, double *out_2112424633615639217) {
  H_13(state, unused, out_2112424633615639217);
}
void pose_h_14(double *state, double *unused, double *out_7493722489878846780) {
  h_14(state, unused, out_7493722489878846780);
}
void pose_H_14(double *state, double *unused, double *out_5640899799481839174) {
  H_14(state, unused, out_5640899799481839174);
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
