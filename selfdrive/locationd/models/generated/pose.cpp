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
void err_fun(double *nom_x, double *delta_x, double *out_2762310420758368335) {
   out_2762310420758368335[0] = delta_x[0] + nom_x[0];
   out_2762310420758368335[1] = delta_x[1] + nom_x[1];
   out_2762310420758368335[2] = delta_x[2] + nom_x[2];
   out_2762310420758368335[3] = delta_x[3] + nom_x[3];
   out_2762310420758368335[4] = delta_x[4] + nom_x[4];
   out_2762310420758368335[5] = delta_x[5] + nom_x[5];
   out_2762310420758368335[6] = delta_x[6] + nom_x[6];
   out_2762310420758368335[7] = delta_x[7] + nom_x[7];
   out_2762310420758368335[8] = delta_x[8] + nom_x[8];
   out_2762310420758368335[9] = delta_x[9] + nom_x[9];
   out_2762310420758368335[10] = delta_x[10] + nom_x[10];
   out_2762310420758368335[11] = delta_x[11] + nom_x[11];
   out_2762310420758368335[12] = delta_x[12] + nom_x[12];
   out_2762310420758368335[13] = delta_x[13] + nom_x[13];
   out_2762310420758368335[14] = delta_x[14] + nom_x[14];
   out_2762310420758368335[15] = delta_x[15] + nom_x[15];
   out_2762310420758368335[16] = delta_x[16] + nom_x[16];
   out_2762310420758368335[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4946458637261953472) {
   out_4946458637261953472[0] = -nom_x[0] + true_x[0];
   out_4946458637261953472[1] = -nom_x[1] + true_x[1];
   out_4946458637261953472[2] = -nom_x[2] + true_x[2];
   out_4946458637261953472[3] = -nom_x[3] + true_x[3];
   out_4946458637261953472[4] = -nom_x[4] + true_x[4];
   out_4946458637261953472[5] = -nom_x[5] + true_x[5];
   out_4946458637261953472[6] = -nom_x[6] + true_x[6];
   out_4946458637261953472[7] = -nom_x[7] + true_x[7];
   out_4946458637261953472[8] = -nom_x[8] + true_x[8];
   out_4946458637261953472[9] = -nom_x[9] + true_x[9];
   out_4946458637261953472[10] = -nom_x[10] + true_x[10];
   out_4946458637261953472[11] = -nom_x[11] + true_x[11];
   out_4946458637261953472[12] = -nom_x[12] + true_x[12];
   out_4946458637261953472[13] = -nom_x[13] + true_x[13];
   out_4946458637261953472[14] = -nom_x[14] + true_x[14];
   out_4946458637261953472[15] = -nom_x[15] + true_x[15];
   out_4946458637261953472[16] = -nom_x[16] + true_x[16];
   out_4946458637261953472[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4775082209311112640) {
   out_4775082209311112640[0] = 1.0;
   out_4775082209311112640[1] = 0.0;
   out_4775082209311112640[2] = 0.0;
   out_4775082209311112640[3] = 0.0;
   out_4775082209311112640[4] = 0.0;
   out_4775082209311112640[5] = 0.0;
   out_4775082209311112640[6] = 0.0;
   out_4775082209311112640[7] = 0.0;
   out_4775082209311112640[8] = 0.0;
   out_4775082209311112640[9] = 0.0;
   out_4775082209311112640[10] = 0.0;
   out_4775082209311112640[11] = 0.0;
   out_4775082209311112640[12] = 0.0;
   out_4775082209311112640[13] = 0.0;
   out_4775082209311112640[14] = 0.0;
   out_4775082209311112640[15] = 0.0;
   out_4775082209311112640[16] = 0.0;
   out_4775082209311112640[17] = 0.0;
   out_4775082209311112640[18] = 0.0;
   out_4775082209311112640[19] = 1.0;
   out_4775082209311112640[20] = 0.0;
   out_4775082209311112640[21] = 0.0;
   out_4775082209311112640[22] = 0.0;
   out_4775082209311112640[23] = 0.0;
   out_4775082209311112640[24] = 0.0;
   out_4775082209311112640[25] = 0.0;
   out_4775082209311112640[26] = 0.0;
   out_4775082209311112640[27] = 0.0;
   out_4775082209311112640[28] = 0.0;
   out_4775082209311112640[29] = 0.0;
   out_4775082209311112640[30] = 0.0;
   out_4775082209311112640[31] = 0.0;
   out_4775082209311112640[32] = 0.0;
   out_4775082209311112640[33] = 0.0;
   out_4775082209311112640[34] = 0.0;
   out_4775082209311112640[35] = 0.0;
   out_4775082209311112640[36] = 0.0;
   out_4775082209311112640[37] = 0.0;
   out_4775082209311112640[38] = 1.0;
   out_4775082209311112640[39] = 0.0;
   out_4775082209311112640[40] = 0.0;
   out_4775082209311112640[41] = 0.0;
   out_4775082209311112640[42] = 0.0;
   out_4775082209311112640[43] = 0.0;
   out_4775082209311112640[44] = 0.0;
   out_4775082209311112640[45] = 0.0;
   out_4775082209311112640[46] = 0.0;
   out_4775082209311112640[47] = 0.0;
   out_4775082209311112640[48] = 0.0;
   out_4775082209311112640[49] = 0.0;
   out_4775082209311112640[50] = 0.0;
   out_4775082209311112640[51] = 0.0;
   out_4775082209311112640[52] = 0.0;
   out_4775082209311112640[53] = 0.0;
   out_4775082209311112640[54] = 0.0;
   out_4775082209311112640[55] = 0.0;
   out_4775082209311112640[56] = 0.0;
   out_4775082209311112640[57] = 1.0;
   out_4775082209311112640[58] = 0.0;
   out_4775082209311112640[59] = 0.0;
   out_4775082209311112640[60] = 0.0;
   out_4775082209311112640[61] = 0.0;
   out_4775082209311112640[62] = 0.0;
   out_4775082209311112640[63] = 0.0;
   out_4775082209311112640[64] = 0.0;
   out_4775082209311112640[65] = 0.0;
   out_4775082209311112640[66] = 0.0;
   out_4775082209311112640[67] = 0.0;
   out_4775082209311112640[68] = 0.0;
   out_4775082209311112640[69] = 0.0;
   out_4775082209311112640[70] = 0.0;
   out_4775082209311112640[71] = 0.0;
   out_4775082209311112640[72] = 0.0;
   out_4775082209311112640[73] = 0.0;
   out_4775082209311112640[74] = 0.0;
   out_4775082209311112640[75] = 0.0;
   out_4775082209311112640[76] = 1.0;
   out_4775082209311112640[77] = 0.0;
   out_4775082209311112640[78] = 0.0;
   out_4775082209311112640[79] = 0.0;
   out_4775082209311112640[80] = 0.0;
   out_4775082209311112640[81] = 0.0;
   out_4775082209311112640[82] = 0.0;
   out_4775082209311112640[83] = 0.0;
   out_4775082209311112640[84] = 0.0;
   out_4775082209311112640[85] = 0.0;
   out_4775082209311112640[86] = 0.0;
   out_4775082209311112640[87] = 0.0;
   out_4775082209311112640[88] = 0.0;
   out_4775082209311112640[89] = 0.0;
   out_4775082209311112640[90] = 0.0;
   out_4775082209311112640[91] = 0.0;
   out_4775082209311112640[92] = 0.0;
   out_4775082209311112640[93] = 0.0;
   out_4775082209311112640[94] = 0.0;
   out_4775082209311112640[95] = 1.0;
   out_4775082209311112640[96] = 0.0;
   out_4775082209311112640[97] = 0.0;
   out_4775082209311112640[98] = 0.0;
   out_4775082209311112640[99] = 0.0;
   out_4775082209311112640[100] = 0.0;
   out_4775082209311112640[101] = 0.0;
   out_4775082209311112640[102] = 0.0;
   out_4775082209311112640[103] = 0.0;
   out_4775082209311112640[104] = 0.0;
   out_4775082209311112640[105] = 0.0;
   out_4775082209311112640[106] = 0.0;
   out_4775082209311112640[107] = 0.0;
   out_4775082209311112640[108] = 0.0;
   out_4775082209311112640[109] = 0.0;
   out_4775082209311112640[110] = 0.0;
   out_4775082209311112640[111] = 0.0;
   out_4775082209311112640[112] = 0.0;
   out_4775082209311112640[113] = 0.0;
   out_4775082209311112640[114] = 1.0;
   out_4775082209311112640[115] = 0.0;
   out_4775082209311112640[116] = 0.0;
   out_4775082209311112640[117] = 0.0;
   out_4775082209311112640[118] = 0.0;
   out_4775082209311112640[119] = 0.0;
   out_4775082209311112640[120] = 0.0;
   out_4775082209311112640[121] = 0.0;
   out_4775082209311112640[122] = 0.0;
   out_4775082209311112640[123] = 0.0;
   out_4775082209311112640[124] = 0.0;
   out_4775082209311112640[125] = 0.0;
   out_4775082209311112640[126] = 0.0;
   out_4775082209311112640[127] = 0.0;
   out_4775082209311112640[128] = 0.0;
   out_4775082209311112640[129] = 0.0;
   out_4775082209311112640[130] = 0.0;
   out_4775082209311112640[131] = 0.0;
   out_4775082209311112640[132] = 0.0;
   out_4775082209311112640[133] = 1.0;
   out_4775082209311112640[134] = 0.0;
   out_4775082209311112640[135] = 0.0;
   out_4775082209311112640[136] = 0.0;
   out_4775082209311112640[137] = 0.0;
   out_4775082209311112640[138] = 0.0;
   out_4775082209311112640[139] = 0.0;
   out_4775082209311112640[140] = 0.0;
   out_4775082209311112640[141] = 0.0;
   out_4775082209311112640[142] = 0.0;
   out_4775082209311112640[143] = 0.0;
   out_4775082209311112640[144] = 0.0;
   out_4775082209311112640[145] = 0.0;
   out_4775082209311112640[146] = 0.0;
   out_4775082209311112640[147] = 0.0;
   out_4775082209311112640[148] = 0.0;
   out_4775082209311112640[149] = 0.0;
   out_4775082209311112640[150] = 0.0;
   out_4775082209311112640[151] = 0.0;
   out_4775082209311112640[152] = 1.0;
   out_4775082209311112640[153] = 0.0;
   out_4775082209311112640[154] = 0.0;
   out_4775082209311112640[155] = 0.0;
   out_4775082209311112640[156] = 0.0;
   out_4775082209311112640[157] = 0.0;
   out_4775082209311112640[158] = 0.0;
   out_4775082209311112640[159] = 0.0;
   out_4775082209311112640[160] = 0.0;
   out_4775082209311112640[161] = 0.0;
   out_4775082209311112640[162] = 0.0;
   out_4775082209311112640[163] = 0.0;
   out_4775082209311112640[164] = 0.0;
   out_4775082209311112640[165] = 0.0;
   out_4775082209311112640[166] = 0.0;
   out_4775082209311112640[167] = 0.0;
   out_4775082209311112640[168] = 0.0;
   out_4775082209311112640[169] = 0.0;
   out_4775082209311112640[170] = 0.0;
   out_4775082209311112640[171] = 1.0;
   out_4775082209311112640[172] = 0.0;
   out_4775082209311112640[173] = 0.0;
   out_4775082209311112640[174] = 0.0;
   out_4775082209311112640[175] = 0.0;
   out_4775082209311112640[176] = 0.0;
   out_4775082209311112640[177] = 0.0;
   out_4775082209311112640[178] = 0.0;
   out_4775082209311112640[179] = 0.0;
   out_4775082209311112640[180] = 0.0;
   out_4775082209311112640[181] = 0.0;
   out_4775082209311112640[182] = 0.0;
   out_4775082209311112640[183] = 0.0;
   out_4775082209311112640[184] = 0.0;
   out_4775082209311112640[185] = 0.0;
   out_4775082209311112640[186] = 0.0;
   out_4775082209311112640[187] = 0.0;
   out_4775082209311112640[188] = 0.0;
   out_4775082209311112640[189] = 0.0;
   out_4775082209311112640[190] = 1.0;
   out_4775082209311112640[191] = 0.0;
   out_4775082209311112640[192] = 0.0;
   out_4775082209311112640[193] = 0.0;
   out_4775082209311112640[194] = 0.0;
   out_4775082209311112640[195] = 0.0;
   out_4775082209311112640[196] = 0.0;
   out_4775082209311112640[197] = 0.0;
   out_4775082209311112640[198] = 0.0;
   out_4775082209311112640[199] = 0.0;
   out_4775082209311112640[200] = 0.0;
   out_4775082209311112640[201] = 0.0;
   out_4775082209311112640[202] = 0.0;
   out_4775082209311112640[203] = 0.0;
   out_4775082209311112640[204] = 0.0;
   out_4775082209311112640[205] = 0.0;
   out_4775082209311112640[206] = 0.0;
   out_4775082209311112640[207] = 0.0;
   out_4775082209311112640[208] = 0.0;
   out_4775082209311112640[209] = 1.0;
   out_4775082209311112640[210] = 0.0;
   out_4775082209311112640[211] = 0.0;
   out_4775082209311112640[212] = 0.0;
   out_4775082209311112640[213] = 0.0;
   out_4775082209311112640[214] = 0.0;
   out_4775082209311112640[215] = 0.0;
   out_4775082209311112640[216] = 0.0;
   out_4775082209311112640[217] = 0.0;
   out_4775082209311112640[218] = 0.0;
   out_4775082209311112640[219] = 0.0;
   out_4775082209311112640[220] = 0.0;
   out_4775082209311112640[221] = 0.0;
   out_4775082209311112640[222] = 0.0;
   out_4775082209311112640[223] = 0.0;
   out_4775082209311112640[224] = 0.0;
   out_4775082209311112640[225] = 0.0;
   out_4775082209311112640[226] = 0.0;
   out_4775082209311112640[227] = 0.0;
   out_4775082209311112640[228] = 1.0;
   out_4775082209311112640[229] = 0.0;
   out_4775082209311112640[230] = 0.0;
   out_4775082209311112640[231] = 0.0;
   out_4775082209311112640[232] = 0.0;
   out_4775082209311112640[233] = 0.0;
   out_4775082209311112640[234] = 0.0;
   out_4775082209311112640[235] = 0.0;
   out_4775082209311112640[236] = 0.0;
   out_4775082209311112640[237] = 0.0;
   out_4775082209311112640[238] = 0.0;
   out_4775082209311112640[239] = 0.0;
   out_4775082209311112640[240] = 0.0;
   out_4775082209311112640[241] = 0.0;
   out_4775082209311112640[242] = 0.0;
   out_4775082209311112640[243] = 0.0;
   out_4775082209311112640[244] = 0.0;
   out_4775082209311112640[245] = 0.0;
   out_4775082209311112640[246] = 0.0;
   out_4775082209311112640[247] = 1.0;
   out_4775082209311112640[248] = 0.0;
   out_4775082209311112640[249] = 0.0;
   out_4775082209311112640[250] = 0.0;
   out_4775082209311112640[251] = 0.0;
   out_4775082209311112640[252] = 0.0;
   out_4775082209311112640[253] = 0.0;
   out_4775082209311112640[254] = 0.0;
   out_4775082209311112640[255] = 0.0;
   out_4775082209311112640[256] = 0.0;
   out_4775082209311112640[257] = 0.0;
   out_4775082209311112640[258] = 0.0;
   out_4775082209311112640[259] = 0.0;
   out_4775082209311112640[260] = 0.0;
   out_4775082209311112640[261] = 0.0;
   out_4775082209311112640[262] = 0.0;
   out_4775082209311112640[263] = 0.0;
   out_4775082209311112640[264] = 0.0;
   out_4775082209311112640[265] = 0.0;
   out_4775082209311112640[266] = 1.0;
   out_4775082209311112640[267] = 0.0;
   out_4775082209311112640[268] = 0.0;
   out_4775082209311112640[269] = 0.0;
   out_4775082209311112640[270] = 0.0;
   out_4775082209311112640[271] = 0.0;
   out_4775082209311112640[272] = 0.0;
   out_4775082209311112640[273] = 0.0;
   out_4775082209311112640[274] = 0.0;
   out_4775082209311112640[275] = 0.0;
   out_4775082209311112640[276] = 0.0;
   out_4775082209311112640[277] = 0.0;
   out_4775082209311112640[278] = 0.0;
   out_4775082209311112640[279] = 0.0;
   out_4775082209311112640[280] = 0.0;
   out_4775082209311112640[281] = 0.0;
   out_4775082209311112640[282] = 0.0;
   out_4775082209311112640[283] = 0.0;
   out_4775082209311112640[284] = 0.0;
   out_4775082209311112640[285] = 1.0;
   out_4775082209311112640[286] = 0.0;
   out_4775082209311112640[287] = 0.0;
   out_4775082209311112640[288] = 0.0;
   out_4775082209311112640[289] = 0.0;
   out_4775082209311112640[290] = 0.0;
   out_4775082209311112640[291] = 0.0;
   out_4775082209311112640[292] = 0.0;
   out_4775082209311112640[293] = 0.0;
   out_4775082209311112640[294] = 0.0;
   out_4775082209311112640[295] = 0.0;
   out_4775082209311112640[296] = 0.0;
   out_4775082209311112640[297] = 0.0;
   out_4775082209311112640[298] = 0.0;
   out_4775082209311112640[299] = 0.0;
   out_4775082209311112640[300] = 0.0;
   out_4775082209311112640[301] = 0.0;
   out_4775082209311112640[302] = 0.0;
   out_4775082209311112640[303] = 0.0;
   out_4775082209311112640[304] = 1.0;
   out_4775082209311112640[305] = 0.0;
   out_4775082209311112640[306] = 0.0;
   out_4775082209311112640[307] = 0.0;
   out_4775082209311112640[308] = 0.0;
   out_4775082209311112640[309] = 0.0;
   out_4775082209311112640[310] = 0.0;
   out_4775082209311112640[311] = 0.0;
   out_4775082209311112640[312] = 0.0;
   out_4775082209311112640[313] = 0.0;
   out_4775082209311112640[314] = 0.0;
   out_4775082209311112640[315] = 0.0;
   out_4775082209311112640[316] = 0.0;
   out_4775082209311112640[317] = 0.0;
   out_4775082209311112640[318] = 0.0;
   out_4775082209311112640[319] = 0.0;
   out_4775082209311112640[320] = 0.0;
   out_4775082209311112640[321] = 0.0;
   out_4775082209311112640[322] = 0.0;
   out_4775082209311112640[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8301426414689752095) {
   out_8301426414689752095[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8301426414689752095[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8301426414689752095[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8301426414689752095[3] = dt*state[12] + state[3];
   out_8301426414689752095[4] = dt*state[13] + state[4];
   out_8301426414689752095[5] = dt*state[14] + state[5];
   out_8301426414689752095[6] = state[6];
   out_8301426414689752095[7] = state[7];
   out_8301426414689752095[8] = state[8];
   out_8301426414689752095[9] = state[9];
   out_8301426414689752095[10] = state[10];
   out_8301426414689752095[11] = state[11];
   out_8301426414689752095[12] = state[12];
   out_8301426414689752095[13] = state[13];
   out_8301426414689752095[14] = state[14];
   out_8301426414689752095[15] = state[15];
   out_8301426414689752095[16] = state[16];
   out_8301426414689752095[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3550420757029977421) {
   out_3550420757029977421[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3550420757029977421[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3550420757029977421[2] = 0;
   out_3550420757029977421[3] = 0;
   out_3550420757029977421[4] = 0;
   out_3550420757029977421[5] = 0;
   out_3550420757029977421[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3550420757029977421[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3550420757029977421[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3550420757029977421[9] = 0;
   out_3550420757029977421[10] = 0;
   out_3550420757029977421[11] = 0;
   out_3550420757029977421[12] = 0;
   out_3550420757029977421[13] = 0;
   out_3550420757029977421[14] = 0;
   out_3550420757029977421[15] = 0;
   out_3550420757029977421[16] = 0;
   out_3550420757029977421[17] = 0;
   out_3550420757029977421[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3550420757029977421[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3550420757029977421[20] = 0;
   out_3550420757029977421[21] = 0;
   out_3550420757029977421[22] = 0;
   out_3550420757029977421[23] = 0;
   out_3550420757029977421[24] = 0;
   out_3550420757029977421[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3550420757029977421[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3550420757029977421[27] = 0;
   out_3550420757029977421[28] = 0;
   out_3550420757029977421[29] = 0;
   out_3550420757029977421[30] = 0;
   out_3550420757029977421[31] = 0;
   out_3550420757029977421[32] = 0;
   out_3550420757029977421[33] = 0;
   out_3550420757029977421[34] = 0;
   out_3550420757029977421[35] = 0;
   out_3550420757029977421[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3550420757029977421[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3550420757029977421[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3550420757029977421[39] = 0;
   out_3550420757029977421[40] = 0;
   out_3550420757029977421[41] = 0;
   out_3550420757029977421[42] = 0;
   out_3550420757029977421[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3550420757029977421[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3550420757029977421[45] = 0;
   out_3550420757029977421[46] = 0;
   out_3550420757029977421[47] = 0;
   out_3550420757029977421[48] = 0;
   out_3550420757029977421[49] = 0;
   out_3550420757029977421[50] = 0;
   out_3550420757029977421[51] = 0;
   out_3550420757029977421[52] = 0;
   out_3550420757029977421[53] = 0;
   out_3550420757029977421[54] = 0;
   out_3550420757029977421[55] = 0;
   out_3550420757029977421[56] = 0;
   out_3550420757029977421[57] = 1;
   out_3550420757029977421[58] = 0;
   out_3550420757029977421[59] = 0;
   out_3550420757029977421[60] = 0;
   out_3550420757029977421[61] = 0;
   out_3550420757029977421[62] = 0;
   out_3550420757029977421[63] = 0;
   out_3550420757029977421[64] = 0;
   out_3550420757029977421[65] = 0;
   out_3550420757029977421[66] = dt;
   out_3550420757029977421[67] = 0;
   out_3550420757029977421[68] = 0;
   out_3550420757029977421[69] = 0;
   out_3550420757029977421[70] = 0;
   out_3550420757029977421[71] = 0;
   out_3550420757029977421[72] = 0;
   out_3550420757029977421[73] = 0;
   out_3550420757029977421[74] = 0;
   out_3550420757029977421[75] = 0;
   out_3550420757029977421[76] = 1;
   out_3550420757029977421[77] = 0;
   out_3550420757029977421[78] = 0;
   out_3550420757029977421[79] = 0;
   out_3550420757029977421[80] = 0;
   out_3550420757029977421[81] = 0;
   out_3550420757029977421[82] = 0;
   out_3550420757029977421[83] = 0;
   out_3550420757029977421[84] = 0;
   out_3550420757029977421[85] = dt;
   out_3550420757029977421[86] = 0;
   out_3550420757029977421[87] = 0;
   out_3550420757029977421[88] = 0;
   out_3550420757029977421[89] = 0;
   out_3550420757029977421[90] = 0;
   out_3550420757029977421[91] = 0;
   out_3550420757029977421[92] = 0;
   out_3550420757029977421[93] = 0;
   out_3550420757029977421[94] = 0;
   out_3550420757029977421[95] = 1;
   out_3550420757029977421[96] = 0;
   out_3550420757029977421[97] = 0;
   out_3550420757029977421[98] = 0;
   out_3550420757029977421[99] = 0;
   out_3550420757029977421[100] = 0;
   out_3550420757029977421[101] = 0;
   out_3550420757029977421[102] = 0;
   out_3550420757029977421[103] = 0;
   out_3550420757029977421[104] = dt;
   out_3550420757029977421[105] = 0;
   out_3550420757029977421[106] = 0;
   out_3550420757029977421[107] = 0;
   out_3550420757029977421[108] = 0;
   out_3550420757029977421[109] = 0;
   out_3550420757029977421[110] = 0;
   out_3550420757029977421[111] = 0;
   out_3550420757029977421[112] = 0;
   out_3550420757029977421[113] = 0;
   out_3550420757029977421[114] = 1;
   out_3550420757029977421[115] = 0;
   out_3550420757029977421[116] = 0;
   out_3550420757029977421[117] = 0;
   out_3550420757029977421[118] = 0;
   out_3550420757029977421[119] = 0;
   out_3550420757029977421[120] = 0;
   out_3550420757029977421[121] = 0;
   out_3550420757029977421[122] = 0;
   out_3550420757029977421[123] = 0;
   out_3550420757029977421[124] = 0;
   out_3550420757029977421[125] = 0;
   out_3550420757029977421[126] = 0;
   out_3550420757029977421[127] = 0;
   out_3550420757029977421[128] = 0;
   out_3550420757029977421[129] = 0;
   out_3550420757029977421[130] = 0;
   out_3550420757029977421[131] = 0;
   out_3550420757029977421[132] = 0;
   out_3550420757029977421[133] = 1;
   out_3550420757029977421[134] = 0;
   out_3550420757029977421[135] = 0;
   out_3550420757029977421[136] = 0;
   out_3550420757029977421[137] = 0;
   out_3550420757029977421[138] = 0;
   out_3550420757029977421[139] = 0;
   out_3550420757029977421[140] = 0;
   out_3550420757029977421[141] = 0;
   out_3550420757029977421[142] = 0;
   out_3550420757029977421[143] = 0;
   out_3550420757029977421[144] = 0;
   out_3550420757029977421[145] = 0;
   out_3550420757029977421[146] = 0;
   out_3550420757029977421[147] = 0;
   out_3550420757029977421[148] = 0;
   out_3550420757029977421[149] = 0;
   out_3550420757029977421[150] = 0;
   out_3550420757029977421[151] = 0;
   out_3550420757029977421[152] = 1;
   out_3550420757029977421[153] = 0;
   out_3550420757029977421[154] = 0;
   out_3550420757029977421[155] = 0;
   out_3550420757029977421[156] = 0;
   out_3550420757029977421[157] = 0;
   out_3550420757029977421[158] = 0;
   out_3550420757029977421[159] = 0;
   out_3550420757029977421[160] = 0;
   out_3550420757029977421[161] = 0;
   out_3550420757029977421[162] = 0;
   out_3550420757029977421[163] = 0;
   out_3550420757029977421[164] = 0;
   out_3550420757029977421[165] = 0;
   out_3550420757029977421[166] = 0;
   out_3550420757029977421[167] = 0;
   out_3550420757029977421[168] = 0;
   out_3550420757029977421[169] = 0;
   out_3550420757029977421[170] = 0;
   out_3550420757029977421[171] = 1;
   out_3550420757029977421[172] = 0;
   out_3550420757029977421[173] = 0;
   out_3550420757029977421[174] = 0;
   out_3550420757029977421[175] = 0;
   out_3550420757029977421[176] = 0;
   out_3550420757029977421[177] = 0;
   out_3550420757029977421[178] = 0;
   out_3550420757029977421[179] = 0;
   out_3550420757029977421[180] = 0;
   out_3550420757029977421[181] = 0;
   out_3550420757029977421[182] = 0;
   out_3550420757029977421[183] = 0;
   out_3550420757029977421[184] = 0;
   out_3550420757029977421[185] = 0;
   out_3550420757029977421[186] = 0;
   out_3550420757029977421[187] = 0;
   out_3550420757029977421[188] = 0;
   out_3550420757029977421[189] = 0;
   out_3550420757029977421[190] = 1;
   out_3550420757029977421[191] = 0;
   out_3550420757029977421[192] = 0;
   out_3550420757029977421[193] = 0;
   out_3550420757029977421[194] = 0;
   out_3550420757029977421[195] = 0;
   out_3550420757029977421[196] = 0;
   out_3550420757029977421[197] = 0;
   out_3550420757029977421[198] = 0;
   out_3550420757029977421[199] = 0;
   out_3550420757029977421[200] = 0;
   out_3550420757029977421[201] = 0;
   out_3550420757029977421[202] = 0;
   out_3550420757029977421[203] = 0;
   out_3550420757029977421[204] = 0;
   out_3550420757029977421[205] = 0;
   out_3550420757029977421[206] = 0;
   out_3550420757029977421[207] = 0;
   out_3550420757029977421[208] = 0;
   out_3550420757029977421[209] = 1;
   out_3550420757029977421[210] = 0;
   out_3550420757029977421[211] = 0;
   out_3550420757029977421[212] = 0;
   out_3550420757029977421[213] = 0;
   out_3550420757029977421[214] = 0;
   out_3550420757029977421[215] = 0;
   out_3550420757029977421[216] = 0;
   out_3550420757029977421[217] = 0;
   out_3550420757029977421[218] = 0;
   out_3550420757029977421[219] = 0;
   out_3550420757029977421[220] = 0;
   out_3550420757029977421[221] = 0;
   out_3550420757029977421[222] = 0;
   out_3550420757029977421[223] = 0;
   out_3550420757029977421[224] = 0;
   out_3550420757029977421[225] = 0;
   out_3550420757029977421[226] = 0;
   out_3550420757029977421[227] = 0;
   out_3550420757029977421[228] = 1;
   out_3550420757029977421[229] = 0;
   out_3550420757029977421[230] = 0;
   out_3550420757029977421[231] = 0;
   out_3550420757029977421[232] = 0;
   out_3550420757029977421[233] = 0;
   out_3550420757029977421[234] = 0;
   out_3550420757029977421[235] = 0;
   out_3550420757029977421[236] = 0;
   out_3550420757029977421[237] = 0;
   out_3550420757029977421[238] = 0;
   out_3550420757029977421[239] = 0;
   out_3550420757029977421[240] = 0;
   out_3550420757029977421[241] = 0;
   out_3550420757029977421[242] = 0;
   out_3550420757029977421[243] = 0;
   out_3550420757029977421[244] = 0;
   out_3550420757029977421[245] = 0;
   out_3550420757029977421[246] = 0;
   out_3550420757029977421[247] = 1;
   out_3550420757029977421[248] = 0;
   out_3550420757029977421[249] = 0;
   out_3550420757029977421[250] = 0;
   out_3550420757029977421[251] = 0;
   out_3550420757029977421[252] = 0;
   out_3550420757029977421[253] = 0;
   out_3550420757029977421[254] = 0;
   out_3550420757029977421[255] = 0;
   out_3550420757029977421[256] = 0;
   out_3550420757029977421[257] = 0;
   out_3550420757029977421[258] = 0;
   out_3550420757029977421[259] = 0;
   out_3550420757029977421[260] = 0;
   out_3550420757029977421[261] = 0;
   out_3550420757029977421[262] = 0;
   out_3550420757029977421[263] = 0;
   out_3550420757029977421[264] = 0;
   out_3550420757029977421[265] = 0;
   out_3550420757029977421[266] = 1;
   out_3550420757029977421[267] = 0;
   out_3550420757029977421[268] = 0;
   out_3550420757029977421[269] = 0;
   out_3550420757029977421[270] = 0;
   out_3550420757029977421[271] = 0;
   out_3550420757029977421[272] = 0;
   out_3550420757029977421[273] = 0;
   out_3550420757029977421[274] = 0;
   out_3550420757029977421[275] = 0;
   out_3550420757029977421[276] = 0;
   out_3550420757029977421[277] = 0;
   out_3550420757029977421[278] = 0;
   out_3550420757029977421[279] = 0;
   out_3550420757029977421[280] = 0;
   out_3550420757029977421[281] = 0;
   out_3550420757029977421[282] = 0;
   out_3550420757029977421[283] = 0;
   out_3550420757029977421[284] = 0;
   out_3550420757029977421[285] = 1;
   out_3550420757029977421[286] = 0;
   out_3550420757029977421[287] = 0;
   out_3550420757029977421[288] = 0;
   out_3550420757029977421[289] = 0;
   out_3550420757029977421[290] = 0;
   out_3550420757029977421[291] = 0;
   out_3550420757029977421[292] = 0;
   out_3550420757029977421[293] = 0;
   out_3550420757029977421[294] = 0;
   out_3550420757029977421[295] = 0;
   out_3550420757029977421[296] = 0;
   out_3550420757029977421[297] = 0;
   out_3550420757029977421[298] = 0;
   out_3550420757029977421[299] = 0;
   out_3550420757029977421[300] = 0;
   out_3550420757029977421[301] = 0;
   out_3550420757029977421[302] = 0;
   out_3550420757029977421[303] = 0;
   out_3550420757029977421[304] = 1;
   out_3550420757029977421[305] = 0;
   out_3550420757029977421[306] = 0;
   out_3550420757029977421[307] = 0;
   out_3550420757029977421[308] = 0;
   out_3550420757029977421[309] = 0;
   out_3550420757029977421[310] = 0;
   out_3550420757029977421[311] = 0;
   out_3550420757029977421[312] = 0;
   out_3550420757029977421[313] = 0;
   out_3550420757029977421[314] = 0;
   out_3550420757029977421[315] = 0;
   out_3550420757029977421[316] = 0;
   out_3550420757029977421[317] = 0;
   out_3550420757029977421[318] = 0;
   out_3550420757029977421[319] = 0;
   out_3550420757029977421[320] = 0;
   out_3550420757029977421[321] = 0;
   out_3550420757029977421[322] = 0;
   out_3550420757029977421[323] = 1;
}
void h_4(double *state, double *unused, double *out_6563430256492903056) {
   out_6563430256492903056[0] = state[6] + state[9];
   out_6563430256492903056[1] = state[7] + state[10];
   out_6563430256492903056[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4169279456746708702) {
   out_4169279456746708702[0] = 0;
   out_4169279456746708702[1] = 0;
   out_4169279456746708702[2] = 0;
   out_4169279456746708702[3] = 0;
   out_4169279456746708702[4] = 0;
   out_4169279456746708702[5] = 0;
   out_4169279456746708702[6] = 1;
   out_4169279456746708702[7] = 0;
   out_4169279456746708702[8] = 0;
   out_4169279456746708702[9] = 1;
   out_4169279456746708702[10] = 0;
   out_4169279456746708702[11] = 0;
   out_4169279456746708702[12] = 0;
   out_4169279456746708702[13] = 0;
   out_4169279456746708702[14] = 0;
   out_4169279456746708702[15] = 0;
   out_4169279456746708702[16] = 0;
   out_4169279456746708702[17] = 0;
   out_4169279456746708702[18] = 0;
   out_4169279456746708702[19] = 0;
   out_4169279456746708702[20] = 0;
   out_4169279456746708702[21] = 0;
   out_4169279456746708702[22] = 0;
   out_4169279456746708702[23] = 0;
   out_4169279456746708702[24] = 0;
   out_4169279456746708702[25] = 1;
   out_4169279456746708702[26] = 0;
   out_4169279456746708702[27] = 0;
   out_4169279456746708702[28] = 1;
   out_4169279456746708702[29] = 0;
   out_4169279456746708702[30] = 0;
   out_4169279456746708702[31] = 0;
   out_4169279456746708702[32] = 0;
   out_4169279456746708702[33] = 0;
   out_4169279456746708702[34] = 0;
   out_4169279456746708702[35] = 0;
   out_4169279456746708702[36] = 0;
   out_4169279456746708702[37] = 0;
   out_4169279456746708702[38] = 0;
   out_4169279456746708702[39] = 0;
   out_4169279456746708702[40] = 0;
   out_4169279456746708702[41] = 0;
   out_4169279456746708702[42] = 0;
   out_4169279456746708702[43] = 0;
   out_4169279456746708702[44] = 1;
   out_4169279456746708702[45] = 0;
   out_4169279456746708702[46] = 0;
   out_4169279456746708702[47] = 1;
   out_4169279456746708702[48] = 0;
   out_4169279456746708702[49] = 0;
   out_4169279456746708702[50] = 0;
   out_4169279456746708702[51] = 0;
   out_4169279456746708702[52] = 0;
   out_4169279456746708702[53] = 0;
}
void h_10(double *state, double *unused, double *out_779779828848067105) {
   out_779779828848067105[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_779779828848067105[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_779779828848067105[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1944061734434103698) {
   out_1944061734434103698[0] = 0;
   out_1944061734434103698[1] = 9.8100000000000005*cos(state[1]);
   out_1944061734434103698[2] = 0;
   out_1944061734434103698[3] = 0;
   out_1944061734434103698[4] = -state[8];
   out_1944061734434103698[5] = state[7];
   out_1944061734434103698[6] = 0;
   out_1944061734434103698[7] = state[5];
   out_1944061734434103698[8] = -state[4];
   out_1944061734434103698[9] = 0;
   out_1944061734434103698[10] = 0;
   out_1944061734434103698[11] = 0;
   out_1944061734434103698[12] = 1;
   out_1944061734434103698[13] = 0;
   out_1944061734434103698[14] = 0;
   out_1944061734434103698[15] = 1;
   out_1944061734434103698[16] = 0;
   out_1944061734434103698[17] = 0;
   out_1944061734434103698[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1944061734434103698[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1944061734434103698[20] = 0;
   out_1944061734434103698[21] = state[8];
   out_1944061734434103698[22] = 0;
   out_1944061734434103698[23] = -state[6];
   out_1944061734434103698[24] = -state[5];
   out_1944061734434103698[25] = 0;
   out_1944061734434103698[26] = state[3];
   out_1944061734434103698[27] = 0;
   out_1944061734434103698[28] = 0;
   out_1944061734434103698[29] = 0;
   out_1944061734434103698[30] = 0;
   out_1944061734434103698[31] = 1;
   out_1944061734434103698[32] = 0;
   out_1944061734434103698[33] = 0;
   out_1944061734434103698[34] = 1;
   out_1944061734434103698[35] = 0;
   out_1944061734434103698[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1944061734434103698[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1944061734434103698[38] = 0;
   out_1944061734434103698[39] = -state[7];
   out_1944061734434103698[40] = state[6];
   out_1944061734434103698[41] = 0;
   out_1944061734434103698[42] = state[4];
   out_1944061734434103698[43] = -state[3];
   out_1944061734434103698[44] = 0;
   out_1944061734434103698[45] = 0;
   out_1944061734434103698[46] = 0;
   out_1944061734434103698[47] = 0;
   out_1944061734434103698[48] = 0;
   out_1944061734434103698[49] = 0;
   out_1944061734434103698[50] = 1;
   out_1944061734434103698[51] = 0;
   out_1944061734434103698[52] = 0;
   out_1944061734434103698[53] = 1;
}
void h_13(double *state, double *unused, double *out_8926596776619908930) {
   out_8926596776619908930[0] = state[3];
   out_8926596776619908930[1] = state[4];
   out_8926596776619908930[2] = state[5];
}
void H_13(double *state, double *unused, double *out_957005631414375901) {
   out_957005631414375901[0] = 0;
   out_957005631414375901[1] = 0;
   out_957005631414375901[2] = 0;
   out_957005631414375901[3] = 1;
   out_957005631414375901[4] = 0;
   out_957005631414375901[5] = 0;
   out_957005631414375901[6] = 0;
   out_957005631414375901[7] = 0;
   out_957005631414375901[8] = 0;
   out_957005631414375901[9] = 0;
   out_957005631414375901[10] = 0;
   out_957005631414375901[11] = 0;
   out_957005631414375901[12] = 0;
   out_957005631414375901[13] = 0;
   out_957005631414375901[14] = 0;
   out_957005631414375901[15] = 0;
   out_957005631414375901[16] = 0;
   out_957005631414375901[17] = 0;
   out_957005631414375901[18] = 0;
   out_957005631414375901[19] = 0;
   out_957005631414375901[20] = 0;
   out_957005631414375901[21] = 0;
   out_957005631414375901[22] = 1;
   out_957005631414375901[23] = 0;
   out_957005631414375901[24] = 0;
   out_957005631414375901[25] = 0;
   out_957005631414375901[26] = 0;
   out_957005631414375901[27] = 0;
   out_957005631414375901[28] = 0;
   out_957005631414375901[29] = 0;
   out_957005631414375901[30] = 0;
   out_957005631414375901[31] = 0;
   out_957005631414375901[32] = 0;
   out_957005631414375901[33] = 0;
   out_957005631414375901[34] = 0;
   out_957005631414375901[35] = 0;
   out_957005631414375901[36] = 0;
   out_957005631414375901[37] = 0;
   out_957005631414375901[38] = 0;
   out_957005631414375901[39] = 0;
   out_957005631414375901[40] = 0;
   out_957005631414375901[41] = 1;
   out_957005631414375901[42] = 0;
   out_957005631414375901[43] = 0;
   out_957005631414375901[44] = 0;
   out_957005631414375901[45] = 0;
   out_957005631414375901[46] = 0;
   out_957005631414375901[47] = 0;
   out_957005631414375901[48] = 0;
   out_957005631414375901[49] = 0;
   out_957005631414375901[50] = 0;
   out_957005631414375901[51] = 0;
   out_957005631414375901[52] = 0;
   out_957005631414375901[53] = 0;
}
void h_14(double *state, double *unused, double *out_1164946293048257452) {
   out_1164946293048257452[0] = state[6];
   out_1164946293048257452[1] = state[7];
   out_1164946293048257452[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7252067889042080998) {
   out_7252067889042080998[0] = 0;
   out_7252067889042080998[1] = 0;
   out_7252067889042080998[2] = 0;
   out_7252067889042080998[3] = 0;
   out_7252067889042080998[4] = 0;
   out_7252067889042080998[5] = 0;
   out_7252067889042080998[6] = 1;
   out_7252067889042080998[7] = 0;
   out_7252067889042080998[8] = 0;
   out_7252067889042080998[9] = 0;
   out_7252067889042080998[10] = 0;
   out_7252067889042080998[11] = 0;
   out_7252067889042080998[12] = 0;
   out_7252067889042080998[13] = 0;
   out_7252067889042080998[14] = 0;
   out_7252067889042080998[15] = 0;
   out_7252067889042080998[16] = 0;
   out_7252067889042080998[17] = 0;
   out_7252067889042080998[18] = 0;
   out_7252067889042080998[19] = 0;
   out_7252067889042080998[20] = 0;
   out_7252067889042080998[21] = 0;
   out_7252067889042080998[22] = 0;
   out_7252067889042080998[23] = 0;
   out_7252067889042080998[24] = 0;
   out_7252067889042080998[25] = 1;
   out_7252067889042080998[26] = 0;
   out_7252067889042080998[27] = 0;
   out_7252067889042080998[28] = 0;
   out_7252067889042080998[29] = 0;
   out_7252067889042080998[30] = 0;
   out_7252067889042080998[31] = 0;
   out_7252067889042080998[32] = 0;
   out_7252067889042080998[33] = 0;
   out_7252067889042080998[34] = 0;
   out_7252067889042080998[35] = 0;
   out_7252067889042080998[36] = 0;
   out_7252067889042080998[37] = 0;
   out_7252067889042080998[38] = 0;
   out_7252067889042080998[39] = 0;
   out_7252067889042080998[40] = 0;
   out_7252067889042080998[41] = 0;
   out_7252067889042080998[42] = 0;
   out_7252067889042080998[43] = 0;
   out_7252067889042080998[44] = 1;
   out_7252067889042080998[45] = 0;
   out_7252067889042080998[46] = 0;
   out_7252067889042080998[47] = 0;
   out_7252067889042080998[48] = 0;
   out_7252067889042080998[49] = 0;
   out_7252067889042080998[50] = 0;
   out_7252067889042080998[51] = 0;
   out_7252067889042080998[52] = 0;
   out_7252067889042080998[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2762310420758368335) {
  err_fun(nom_x, delta_x, out_2762310420758368335);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4946458637261953472) {
  inv_err_fun(nom_x, true_x, out_4946458637261953472);
}
void pose_H_mod_fun(double *state, double *out_4775082209311112640) {
  H_mod_fun(state, out_4775082209311112640);
}
void pose_f_fun(double *state, double dt, double *out_8301426414689752095) {
  f_fun(state,  dt, out_8301426414689752095);
}
void pose_F_fun(double *state, double dt, double *out_3550420757029977421) {
  F_fun(state,  dt, out_3550420757029977421);
}
void pose_h_4(double *state, double *unused, double *out_6563430256492903056) {
  h_4(state, unused, out_6563430256492903056);
}
void pose_H_4(double *state, double *unused, double *out_4169279456746708702) {
  H_4(state, unused, out_4169279456746708702);
}
void pose_h_10(double *state, double *unused, double *out_779779828848067105) {
  h_10(state, unused, out_779779828848067105);
}
void pose_H_10(double *state, double *unused, double *out_1944061734434103698) {
  H_10(state, unused, out_1944061734434103698);
}
void pose_h_13(double *state, double *unused, double *out_8926596776619908930) {
  h_13(state, unused, out_8926596776619908930);
}
void pose_H_13(double *state, double *unused, double *out_957005631414375901) {
  H_13(state, unused, out_957005631414375901);
}
void pose_h_14(double *state, double *unused, double *out_1164946293048257452) {
  h_14(state, unused, out_1164946293048257452);
}
void pose_H_14(double *state, double *unused, double *out_7252067889042080998) {
  H_14(state, unused, out_7252067889042080998);
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
