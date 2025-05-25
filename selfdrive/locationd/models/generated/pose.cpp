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
void err_fun(double *nom_x, double *delta_x, double *out_4705197801635651257) {
   out_4705197801635651257[0] = delta_x[0] + nom_x[0];
   out_4705197801635651257[1] = delta_x[1] + nom_x[1];
   out_4705197801635651257[2] = delta_x[2] + nom_x[2];
   out_4705197801635651257[3] = delta_x[3] + nom_x[3];
   out_4705197801635651257[4] = delta_x[4] + nom_x[4];
   out_4705197801635651257[5] = delta_x[5] + nom_x[5];
   out_4705197801635651257[6] = delta_x[6] + nom_x[6];
   out_4705197801635651257[7] = delta_x[7] + nom_x[7];
   out_4705197801635651257[8] = delta_x[8] + nom_x[8];
   out_4705197801635651257[9] = delta_x[9] + nom_x[9];
   out_4705197801635651257[10] = delta_x[10] + nom_x[10];
   out_4705197801635651257[11] = delta_x[11] + nom_x[11];
   out_4705197801635651257[12] = delta_x[12] + nom_x[12];
   out_4705197801635651257[13] = delta_x[13] + nom_x[13];
   out_4705197801635651257[14] = delta_x[14] + nom_x[14];
   out_4705197801635651257[15] = delta_x[15] + nom_x[15];
   out_4705197801635651257[16] = delta_x[16] + nom_x[16];
   out_4705197801635651257[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4283461203773073958) {
   out_4283461203773073958[0] = -nom_x[0] + true_x[0];
   out_4283461203773073958[1] = -nom_x[1] + true_x[1];
   out_4283461203773073958[2] = -nom_x[2] + true_x[2];
   out_4283461203773073958[3] = -nom_x[3] + true_x[3];
   out_4283461203773073958[4] = -nom_x[4] + true_x[4];
   out_4283461203773073958[5] = -nom_x[5] + true_x[5];
   out_4283461203773073958[6] = -nom_x[6] + true_x[6];
   out_4283461203773073958[7] = -nom_x[7] + true_x[7];
   out_4283461203773073958[8] = -nom_x[8] + true_x[8];
   out_4283461203773073958[9] = -nom_x[9] + true_x[9];
   out_4283461203773073958[10] = -nom_x[10] + true_x[10];
   out_4283461203773073958[11] = -nom_x[11] + true_x[11];
   out_4283461203773073958[12] = -nom_x[12] + true_x[12];
   out_4283461203773073958[13] = -nom_x[13] + true_x[13];
   out_4283461203773073958[14] = -nom_x[14] + true_x[14];
   out_4283461203773073958[15] = -nom_x[15] + true_x[15];
   out_4283461203773073958[16] = -nom_x[16] + true_x[16];
   out_4283461203773073958[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3022793235977438013) {
   out_3022793235977438013[0] = 1.0;
   out_3022793235977438013[1] = 0.0;
   out_3022793235977438013[2] = 0.0;
   out_3022793235977438013[3] = 0.0;
   out_3022793235977438013[4] = 0.0;
   out_3022793235977438013[5] = 0.0;
   out_3022793235977438013[6] = 0.0;
   out_3022793235977438013[7] = 0.0;
   out_3022793235977438013[8] = 0.0;
   out_3022793235977438013[9] = 0.0;
   out_3022793235977438013[10] = 0.0;
   out_3022793235977438013[11] = 0.0;
   out_3022793235977438013[12] = 0.0;
   out_3022793235977438013[13] = 0.0;
   out_3022793235977438013[14] = 0.0;
   out_3022793235977438013[15] = 0.0;
   out_3022793235977438013[16] = 0.0;
   out_3022793235977438013[17] = 0.0;
   out_3022793235977438013[18] = 0.0;
   out_3022793235977438013[19] = 1.0;
   out_3022793235977438013[20] = 0.0;
   out_3022793235977438013[21] = 0.0;
   out_3022793235977438013[22] = 0.0;
   out_3022793235977438013[23] = 0.0;
   out_3022793235977438013[24] = 0.0;
   out_3022793235977438013[25] = 0.0;
   out_3022793235977438013[26] = 0.0;
   out_3022793235977438013[27] = 0.0;
   out_3022793235977438013[28] = 0.0;
   out_3022793235977438013[29] = 0.0;
   out_3022793235977438013[30] = 0.0;
   out_3022793235977438013[31] = 0.0;
   out_3022793235977438013[32] = 0.0;
   out_3022793235977438013[33] = 0.0;
   out_3022793235977438013[34] = 0.0;
   out_3022793235977438013[35] = 0.0;
   out_3022793235977438013[36] = 0.0;
   out_3022793235977438013[37] = 0.0;
   out_3022793235977438013[38] = 1.0;
   out_3022793235977438013[39] = 0.0;
   out_3022793235977438013[40] = 0.0;
   out_3022793235977438013[41] = 0.0;
   out_3022793235977438013[42] = 0.0;
   out_3022793235977438013[43] = 0.0;
   out_3022793235977438013[44] = 0.0;
   out_3022793235977438013[45] = 0.0;
   out_3022793235977438013[46] = 0.0;
   out_3022793235977438013[47] = 0.0;
   out_3022793235977438013[48] = 0.0;
   out_3022793235977438013[49] = 0.0;
   out_3022793235977438013[50] = 0.0;
   out_3022793235977438013[51] = 0.0;
   out_3022793235977438013[52] = 0.0;
   out_3022793235977438013[53] = 0.0;
   out_3022793235977438013[54] = 0.0;
   out_3022793235977438013[55] = 0.0;
   out_3022793235977438013[56] = 0.0;
   out_3022793235977438013[57] = 1.0;
   out_3022793235977438013[58] = 0.0;
   out_3022793235977438013[59] = 0.0;
   out_3022793235977438013[60] = 0.0;
   out_3022793235977438013[61] = 0.0;
   out_3022793235977438013[62] = 0.0;
   out_3022793235977438013[63] = 0.0;
   out_3022793235977438013[64] = 0.0;
   out_3022793235977438013[65] = 0.0;
   out_3022793235977438013[66] = 0.0;
   out_3022793235977438013[67] = 0.0;
   out_3022793235977438013[68] = 0.0;
   out_3022793235977438013[69] = 0.0;
   out_3022793235977438013[70] = 0.0;
   out_3022793235977438013[71] = 0.0;
   out_3022793235977438013[72] = 0.0;
   out_3022793235977438013[73] = 0.0;
   out_3022793235977438013[74] = 0.0;
   out_3022793235977438013[75] = 0.0;
   out_3022793235977438013[76] = 1.0;
   out_3022793235977438013[77] = 0.0;
   out_3022793235977438013[78] = 0.0;
   out_3022793235977438013[79] = 0.0;
   out_3022793235977438013[80] = 0.0;
   out_3022793235977438013[81] = 0.0;
   out_3022793235977438013[82] = 0.0;
   out_3022793235977438013[83] = 0.0;
   out_3022793235977438013[84] = 0.0;
   out_3022793235977438013[85] = 0.0;
   out_3022793235977438013[86] = 0.0;
   out_3022793235977438013[87] = 0.0;
   out_3022793235977438013[88] = 0.0;
   out_3022793235977438013[89] = 0.0;
   out_3022793235977438013[90] = 0.0;
   out_3022793235977438013[91] = 0.0;
   out_3022793235977438013[92] = 0.0;
   out_3022793235977438013[93] = 0.0;
   out_3022793235977438013[94] = 0.0;
   out_3022793235977438013[95] = 1.0;
   out_3022793235977438013[96] = 0.0;
   out_3022793235977438013[97] = 0.0;
   out_3022793235977438013[98] = 0.0;
   out_3022793235977438013[99] = 0.0;
   out_3022793235977438013[100] = 0.0;
   out_3022793235977438013[101] = 0.0;
   out_3022793235977438013[102] = 0.0;
   out_3022793235977438013[103] = 0.0;
   out_3022793235977438013[104] = 0.0;
   out_3022793235977438013[105] = 0.0;
   out_3022793235977438013[106] = 0.0;
   out_3022793235977438013[107] = 0.0;
   out_3022793235977438013[108] = 0.0;
   out_3022793235977438013[109] = 0.0;
   out_3022793235977438013[110] = 0.0;
   out_3022793235977438013[111] = 0.0;
   out_3022793235977438013[112] = 0.0;
   out_3022793235977438013[113] = 0.0;
   out_3022793235977438013[114] = 1.0;
   out_3022793235977438013[115] = 0.0;
   out_3022793235977438013[116] = 0.0;
   out_3022793235977438013[117] = 0.0;
   out_3022793235977438013[118] = 0.0;
   out_3022793235977438013[119] = 0.0;
   out_3022793235977438013[120] = 0.0;
   out_3022793235977438013[121] = 0.0;
   out_3022793235977438013[122] = 0.0;
   out_3022793235977438013[123] = 0.0;
   out_3022793235977438013[124] = 0.0;
   out_3022793235977438013[125] = 0.0;
   out_3022793235977438013[126] = 0.0;
   out_3022793235977438013[127] = 0.0;
   out_3022793235977438013[128] = 0.0;
   out_3022793235977438013[129] = 0.0;
   out_3022793235977438013[130] = 0.0;
   out_3022793235977438013[131] = 0.0;
   out_3022793235977438013[132] = 0.0;
   out_3022793235977438013[133] = 1.0;
   out_3022793235977438013[134] = 0.0;
   out_3022793235977438013[135] = 0.0;
   out_3022793235977438013[136] = 0.0;
   out_3022793235977438013[137] = 0.0;
   out_3022793235977438013[138] = 0.0;
   out_3022793235977438013[139] = 0.0;
   out_3022793235977438013[140] = 0.0;
   out_3022793235977438013[141] = 0.0;
   out_3022793235977438013[142] = 0.0;
   out_3022793235977438013[143] = 0.0;
   out_3022793235977438013[144] = 0.0;
   out_3022793235977438013[145] = 0.0;
   out_3022793235977438013[146] = 0.0;
   out_3022793235977438013[147] = 0.0;
   out_3022793235977438013[148] = 0.0;
   out_3022793235977438013[149] = 0.0;
   out_3022793235977438013[150] = 0.0;
   out_3022793235977438013[151] = 0.0;
   out_3022793235977438013[152] = 1.0;
   out_3022793235977438013[153] = 0.0;
   out_3022793235977438013[154] = 0.0;
   out_3022793235977438013[155] = 0.0;
   out_3022793235977438013[156] = 0.0;
   out_3022793235977438013[157] = 0.0;
   out_3022793235977438013[158] = 0.0;
   out_3022793235977438013[159] = 0.0;
   out_3022793235977438013[160] = 0.0;
   out_3022793235977438013[161] = 0.0;
   out_3022793235977438013[162] = 0.0;
   out_3022793235977438013[163] = 0.0;
   out_3022793235977438013[164] = 0.0;
   out_3022793235977438013[165] = 0.0;
   out_3022793235977438013[166] = 0.0;
   out_3022793235977438013[167] = 0.0;
   out_3022793235977438013[168] = 0.0;
   out_3022793235977438013[169] = 0.0;
   out_3022793235977438013[170] = 0.0;
   out_3022793235977438013[171] = 1.0;
   out_3022793235977438013[172] = 0.0;
   out_3022793235977438013[173] = 0.0;
   out_3022793235977438013[174] = 0.0;
   out_3022793235977438013[175] = 0.0;
   out_3022793235977438013[176] = 0.0;
   out_3022793235977438013[177] = 0.0;
   out_3022793235977438013[178] = 0.0;
   out_3022793235977438013[179] = 0.0;
   out_3022793235977438013[180] = 0.0;
   out_3022793235977438013[181] = 0.0;
   out_3022793235977438013[182] = 0.0;
   out_3022793235977438013[183] = 0.0;
   out_3022793235977438013[184] = 0.0;
   out_3022793235977438013[185] = 0.0;
   out_3022793235977438013[186] = 0.0;
   out_3022793235977438013[187] = 0.0;
   out_3022793235977438013[188] = 0.0;
   out_3022793235977438013[189] = 0.0;
   out_3022793235977438013[190] = 1.0;
   out_3022793235977438013[191] = 0.0;
   out_3022793235977438013[192] = 0.0;
   out_3022793235977438013[193] = 0.0;
   out_3022793235977438013[194] = 0.0;
   out_3022793235977438013[195] = 0.0;
   out_3022793235977438013[196] = 0.0;
   out_3022793235977438013[197] = 0.0;
   out_3022793235977438013[198] = 0.0;
   out_3022793235977438013[199] = 0.0;
   out_3022793235977438013[200] = 0.0;
   out_3022793235977438013[201] = 0.0;
   out_3022793235977438013[202] = 0.0;
   out_3022793235977438013[203] = 0.0;
   out_3022793235977438013[204] = 0.0;
   out_3022793235977438013[205] = 0.0;
   out_3022793235977438013[206] = 0.0;
   out_3022793235977438013[207] = 0.0;
   out_3022793235977438013[208] = 0.0;
   out_3022793235977438013[209] = 1.0;
   out_3022793235977438013[210] = 0.0;
   out_3022793235977438013[211] = 0.0;
   out_3022793235977438013[212] = 0.0;
   out_3022793235977438013[213] = 0.0;
   out_3022793235977438013[214] = 0.0;
   out_3022793235977438013[215] = 0.0;
   out_3022793235977438013[216] = 0.0;
   out_3022793235977438013[217] = 0.0;
   out_3022793235977438013[218] = 0.0;
   out_3022793235977438013[219] = 0.0;
   out_3022793235977438013[220] = 0.0;
   out_3022793235977438013[221] = 0.0;
   out_3022793235977438013[222] = 0.0;
   out_3022793235977438013[223] = 0.0;
   out_3022793235977438013[224] = 0.0;
   out_3022793235977438013[225] = 0.0;
   out_3022793235977438013[226] = 0.0;
   out_3022793235977438013[227] = 0.0;
   out_3022793235977438013[228] = 1.0;
   out_3022793235977438013[229] = 0.0;
   out_3022793235977438013[230] = 0.0;
   out_3022793235977438013[231] = 0.0;
   out_3022793235977438013[232] = 0.0;
   out_3022793235977438013[233] = 0.0;
   out_3022793235977438013[234] = 0.0;
   out_3022793235977438013[235] = 0.0;
   out_3022793235977438013[236] = 0.0;
   out_3022793235977438013[237] = 0.0;
   out_3022793235977438013[238] = 0.0;
   out_3022793235977438013[239] = 0.0;
   out_3022793235977438013[240] = 0.0;
   out_3022793235977438013[241] = 0.0;
   out_3022793235977438013[242] = 0.0;
   out_3022793235977438013[243] = 0.0;
   out_3022793235977438013[244] = 0.0;
   out_3022793235977438013[245] = 0.0;
   out_3022793235977438013[246] = 0.0;
   out_3022793235977438013[247] = 1.0;
   out_3022793235977438013[248] = 0.0;
   out_3022793235977438013[249] = 0.0;
   out_3022793235977438013[250] = 0.0;
   out_3022793235977438013[251] = 0.0;
   out_3022793235977438013[252] = 0.0;
   out_3022793235977438013[253] = 0.0;
   out_3022793235977438013[254] = 0.0;
   out_3022793235977438013[255] = 0.0;
   out_3022793235977438013[256] = 0.0;
   out_3022793235977438013[257] = 0.0;
   out_3022793235977438013[258] = 0.0;
   out_3022793235977438013[259] = 0.0;
   out_3022793235977438013[260] = 0.0;
   out_3022793235977438013[261] = 0.0;
   out_3022793235977438013[262] = 0.0;
   out_3022793235977438013[263] = 0.0;
   out_3022793235977438013[264] = 0.0;
   out_3022793235977438013[265] = 0.0;
   out_3022793235977438013[266] = 1.0;
   out_3022793235977438013[267] = 0.0;
   out_3022793235977438013[268] = 0.0;
   out_3022793235977438013[269] = 0.0;
   out_3022793235977438013[270] = 0.0;
   out_3022793235977438013[271] = 0.0;
   out_3022793235977438013[272] = 0.0;
   out_3022793235977438013[273] = 0.0;
   out_3022793235977438013[274] = 0.0;
   out_3022793235977438013[275] = 0.0;
   out_3022793235977438013[276] = 0.0;
   out_3022793235977438013[277] = 0.0;
   out_3022793235977438013[278] = 0.0;
   out_3022793235977438013[279] = 0.0;
   out_3022793235977438013[280] = 0.0;
   out_3022793235977438013[281] = 0.0;
   out_3022793235977438013[282] = 0.0;
   out_3022793235977438013[283] = 0.0;
   out_3022793235977438013[284] = 0.0;
   out_3022793235977438013[285] = 1.0;
   out_3022793235977438013[286] = 0.0;
   out_3022793235977438013[287] = 0.0;
   out_3022793235977438013[288] = 0.0;
   out_3022793235977438013[289] = 0.0;
   out_3022793235977438013[290] = 0.0;
   out_3022793235977438013[291] = 0.0;
   out_3022793235977438013[292] = 0.0;
   out_3022793235977438013[293] = 0.0;
   out_3022793235977438013[294] = 0.0;
   out_3022793235977438013[295] = 0.0;
   out_3022793235977438013[296] = 0.0;
   out_3022793235977438013[297] = 0.0;
   out_3022793235977438013[298] = 0.0;
   out_3022793235977438013[299] = 0.0;
   out_3022793235977438013[300] = 0.0;
   out_3022793235977438013[301] = 0.0;
   out_3022793235977438013[302] = 0.0;
   out_3022793235977438013[303] = 0.0;
   out_3022793235977438013[304] = 1.0;
   out_3022793235977438013[305] = 0.0;
   out_3022793235977438013[306] = 0.0;
   out_3022793235977438013[307] = 0.0;
   out_3022793235977438013[308] = 0.0;
   out_3022793235977438013[309] = 0.0;
   out_3022793235977438013[310] = 0.0;
   out_3022793235977438013[311] = 0.0;
   out_3022793235977438013[312] = 0.0;
   out_3022793235977438013[313] = 0.0;
   out_3022793235977438013[314] = 0.0;
   out_3022793235977438013[315] = 0.0;
   out_3022793235977438013[316] = 0.0;
   out_3022793235977438013[317] = 0.0;
   out_3022793235977438013[318] = 0.0;
   out_3022793235977438013[319] = 0.0;
   out_3022793235977438013[320] = 0.0;
   out_3022793235977438013[321] = 0.0;
   out_3022793235977438013[322] = 0.0;
   out_3022793235977438013[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3860426828173334801) {
   out_3860426828173334801[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3860426828173334801[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3860426828173334801[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3860426828173334801[3] = dt*state[12] + state[3];
   out_3860426828173334801[4] = dt*state[13] + state[4];
   out_3860426828173334801[5] = dt*state[14] + state[5];
   out_3860426828173334801[6] = state[6];
   out_3860426828173334801[7] = state[7];
   out_3860426828173334801[8] = state[8];
   out_3860426828173334801[9] = state[9];
   out_3860426828173334801[10] = state[10];
   out_3860426828173334801[11] = state[11];
   out_3860426828173334801[12] = state[12];
   out_3860426828173334801[13] = state[13];
   out_3860426828173334801[14] = state[14];
   out_3860426828173334801[15] = state[15];
   out_3860426828173334801[16] = state[16];
   out_3860426828173334801[17] = state[17];
}
void F_fun(double *state, double dt, double *out_2434308811775089426) {
   out_2434308811775089426[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2434308811775089426[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2434308811775089426[2] = 0;
   out_2434308811775089426[3] = 0;
   out_2434308811775089426[4] = 0;
   out_2434308811775089426[5] = 0;
   out_2434308811775089426[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2434308811775089426[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2434308811775089426[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2434308811775089426[9] = 0;
   out_2434308811775089426[10] = 0;
   out_2434308811775089426[11] = 0;
   out_2434308811775089426[12] = 0;
   out_2434308811775089426[13] = 0;
   out_2434308811775089426[14] = 0;
   out_2434308811775089426[15] = 0;
   out_2434308811775089426[16] = 0;
   out_2434308811775089426[17] = 0;
   out_2434308811775089426[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2434308811775089426[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2434308811775089426[20] = 0;
   out_2434308811775089426[21] = 0;
   out_2434308811775089426[22] = 0;
   out_2434308811775089426[23] = 0;
   out_2434308811775089426[24] = 0;
   out_2434308811775089426[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2434308811775089426[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2434308811775089426[27] = 0;
   out_2434308811775089426[28] = 0;
   out_2434308811775089426[29] = 0;
   out_2434308811775089426[30] = 0;
   out_2434308811775089426[31] = 0;
   out_2434308811775089426[32] = 0;
   out_2434308811775089426[33] = 0;
   out_2434308811775089426[34] = 0;
   out_2434308811775089426[35] = 0;
   out_2434308811775089426[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2434308811775089426[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2434308811775089426[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2434308811775089426[39] = 0;
   out_2434308811775089426[40] = 0;
   out_2434308811775089426[41] = 0;
   out_2434308811775089426[42] = 0;
   out_2434308811775089426[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2434308811775089426[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2434308811775089426[45] = 0;
   out_2434308811775089426[46] = 0;
   out_2434308811775089426[47] = 0;
   out_2434308811775089426[48] = 0;
   out_2434308811775089426[49] = 0;
   out_2434308811775089426[50] = 0;
   out_2434308811775089426[51] = 0;
   out_2434308811775089426[52] = 0;
   out_2434308811775089426[53] = 0;
   out_2434308811775089426[54] = 0;
   out_2434308811775089426[55] = 0;
   out_2434308811775089426[56] = 0;
   out_2434308811775089426[57] = 1;
   out_2434308811775089426[58] = 0;
   out_2434308811775089426[59] = 0;
   out_2434308811775089426[60] = 0;
   out_2434308811775089426[61] = 0;
   out_2434308811775089426[62] = 0;
   out_2434308811775089426[63] = 0;
   out_2434308811775089426[64] = 0;
   out_2434308811775089426[65] = 0;
   out_2434308811775089426[66] = dt;
   out_2434308811775089426[67] = 0;
   out_2434308811775089426[68] = 0;
   out_2434308811775089426[69] = 0;
   out_2434308811775089426[70] = 0;
   out_2434308811775089426[71] = 0;
   out_2434308811775089426[72] = 0;
   out_2434308811775089426[73] = 0;
   out_2434308811775089426[74] = 0;
   out_2434308811775089426[75] = 0;
   out_2434308811775089426[76] = 1;
   out_2434308811775089426[77] = 0;
   out_2434308811775089426[78] = 0;
   out_2434308811775089426[79] = 0;
   out_2434308811775089426[80] = 0;
   out_2434308811775089426[81] = 0;
   out_2434308811775089426[82] = 0;
   out_2434308811775089426[83] = 0;
   out_2434308811775089426[84] = 0;
   out_2434308811775089426[85] = dt;
   out_2434308811775089426[86] = 0;
   out_2434308811775089426[87] = 0;
   out_2434308811775089426[88] = 0;
   out_2434308811775089426[89] = 0;
   out_2434308811775089426[90] = 0;
   out_2434308811775089426[91] = 0;
   out_2434308811775089426[92] = 0;
   out_2434308811775089426[93] = 0;
   out_2434308811775089426[94] = 0;
   out_2434308811775089426[95] = 1;
   out_2434308811775089426[96] = 0;
   out_2434308811775089426[97] = 0;
   out_2434308811775089426[98] = 0;
   out_2434308811775089426[99] = 0;
   out_2434308811775089426[100] = 0;
   out_2434308811775089426[101] = 0;
   out_2434308811775089426[102] = 0;
   out_2434308811775089426[103] = 0;
   out_2434308811775089426[104] = dt;
   out_2434308811775089426[105] = 0;
   out_2434308811775089426[106] = 0;
   out_2434308811775089426[107] = 0;
   out_2434308811775089426[108] = 0;
   out_2434308811775089426[109] = 0;
   out_2434308811775089426[110] = 0;
   out_2434308811775089426[111] = 0;
   out_2434308811775089426[112] = 0;
   out_2434308811775089426[113] = 0;
   out_2434308811775089426[114] = 1;
   out_2434308811775089426[115] = 0;
   out_2434308811775089426[116] = 0;
   out_2434308811775089426[117] = 0;
   out_2434308811775089426[118] = 0;
   out_2434308811775089426[119] = 0;
   out_2434308811775089426[120] = 0;
   out_2434308811775089426[121] = 0;
   out_2434308811775089426[122] = 0;
   out_2434308811775089426[123] = 0;
   out_2434308811775089426[124] = 0;
   out_2434308811775089426[125] = 0;
   out_2434308811775089426[126] = 0;
   out_2434308811775089426[127] = 0;
   out_2434308811775089426[128] = 0;
   out_2434308811775089426[129] = 0;
   out_2434308811775089426[130] = 0;
   out_2434308811775089426[131] = 0;
   out_2434308811775089426[132] = 0;
   out_2434308811775089426[133] = 1;
   out_2434308811775089426[134] = 0;
   out_2434308811775089426[135] = 0;
   out_2434308811775089426[136] = 0;
   out_2434308811775089426[137] = 0;
   out_2434308811775089426[138] = 0;
   out_2434308811775089426[139] = 0;
   out_2434308811775089426[140] = 0;
   out_2434308811775089426[141] = 0;
   out_2434308811775089426[142] = 0;
   out_2434308811775089426[143] = 0;
   out_2434308811775089426[144] = 0;
   out_2434308811775089426[145] = 0;
   out_2434308811775089426[146] = 0;
   out_2434308811775089426[147] = 0;
   out_2434308811775089426[148] = 0;
   out_2434308811775089426[149] = 0;
   out_2434308811775089426[150] = 0;
   out_2434308811775089426[151] = 0;
   out_2434308811775089426[152] = 1;
   out_2434308811775089426[153] = 0;
   out_2434308811775089426[154] = 0;
   out_2434308811775089426[155] = 0;
   out_2434308811775089426[156] = 0;
   out_2434308811775089426[157] = 0;
   out_2434308811775089426[158] = 0;
   out_2434308811775089426[159] = 0;
   out_2434308811775089426[160] = 0;
   out_2434308811775089426[161] = 0;
   out_2434308811775089426[162] = 0;
   out_2434308811775089426[163] = 0;
   out_2434308811775089426[164] = 0;
   out_2434308811775089426[165] = 0;
   out_2434308811775089426[166] = 0;
   out_2434308811775089426[167] = 0;
   out_2434308811775089426[168] = 0;
   out_2434308811775089426[169] = 0;
   out_2434308811775089426[170] = 0;
   out_2434308811775089426[171] = 1;
   out_2434308811775089426[172] = 0;
   out_2434308811775089426[173] = 0;
   out_2434308811775089426[174] = 0;
   out_2434308811775089426[175] = 0;
   out_2434308811775089426[176] = 0;
   out_2434308811775089426[177] = 0;
   out_2434308811775089426[178] = 0;
   out_2434308811775089426[179] = 0;
   out_2434308811775089426[180] = 0;
   out_2434308811775089426[181] = 0;
   out_2434308811775089426[182] = 0;
   out_2434308811775089426[183] = 0;
   out_2434308811775089426[184] = 0;
   out_2434308811775089426[185] = 0;
   out_2434308811775089426[186] = 0;
   out_2434308811775089426[187] = 0;
   out_2434308811775089426[188] = 0;
   out_2434308811775089426[189] = 0;
   out_2434308811775089426[190] = 1;
   out_2434308811775089426[191] = 0;
   out_2434308811775089426[192] = 0;
   out_2434308811775089426[193] = 0;
   out_2434308811775089426[194] = 0;
   out_2434308811775089426[195] = 0;
   out_2434308811775089426[196] = 0;
   out_2434308811775089426[197] = 0;
   out_2434308811775089426[198] = 0;
   out_2434308811775089426[199] = 0;
   out_2434308811775089426[200] = 0;
   out_2434308811775089426[201] = 0;
   out_2434308811775089426[202] = 0;
   out_2434308811775089426[203] = 0;
   out_2434308811775089426[204] = 0;
   out_2434308811775089426[205] = 0;
   out_2434308811775089426[206] = 0;
   out_2434308811775089426[207] = 0;
   out_2434308811775089426[208] = 0;
   out_2434308811775089426[209] = 1;
   out_2434308811775089426[210] = 0;
   out_2434308811775089426[211] = 0;
   out_2434308811775089426[212] = 0;
   out_2434308811775089426[213] = 0;
   out_2434308811775089426[214] = 0;
   out_2434308811775089426[215] = 0;
   out_2434308811775089426[216] = 0;
   out_2434308811775089426[217] = 0;
   out_2434308811775089426[218] = 0;
   out_2434308811775089426[219] = 0;
   out_2434308811775089426[220] = 0;
   out_2434308811775089426[221] = 0;
   out_2434308811775089426[222] = 0;
   out_2434308811775089426[223] = 0;
   out_2434308811775089426[224] = 0;
   out_2434308811775089426[225] = 0;
   out_2434308811775089426[226] = 0;
   out_2434308811775089426[227] = 0;
   out_2434308811775089426[228] = 1;
   out_2434308811775089426[229] = 0;
   out_2434308811775089426[230] = 0;
   out_2434308811775089426[231] = 0;
   out_2434308811775089426[232] = 0;
   out_2434308811775089426[233] = 0;
   out_2434308811775089426[234] = 0;
   out_2434308811775089426[235] = 0;
   out_2434308811775089426[236] = 0;
   out_2434308811775089426[237] = 0;
   out_2434308811775089426[238] = 0;
   out_2434308811775089426[239] = 0;
   out_2434308811775089426[240] = 0;
   out_2434308811775089426[241] = 0;
   out_2434308811775089426[242] = 0;
   out_2434308811775089426[243] = 0;
   out_2434308811775089426[244] = 0;
   out_2434308811775089426[245] = 0;
   out_2434308811775089426[246] = 0;
   out_2434308811775089426[247] = 1;
   out_2434308811775089426[248] = 0;
   out_2434308811775089426[249] = 0;
   out_2434308811775089426[250] = 0;
   out_2434308811775089426[251] = 0;
   out_2434308811775089426[252] = 0;
   out_2434308811775089426[253] = 0;
   out_2434308811775089426[254] = 0;
   out_2434308811775089426[255] = 0;
   out_2434308811775089426[256] = 0;
   out_2434308811775089426[257] = 0;
   out_2434308811775089426[258] = 0;
   out_2434308811775089426[259] = 0;
   out_2434308811775089426[260] = 0;
   out_2434308811775089426[261] = 0;
   out_2434308811775089426[262] = 0;
   out_2434308811775089426[263] = 0;
   out_2434308811775089426[264] = 0;
   out_2434308811775089426[265] = 0;
   out_2434308811775089426[266] = 1;
   out_2434308811775089426[267] = 0;
   out_2434308811775089426[268] = 0;
   out_2434308811775089426[269] = 0;
   out_2434308811775089426[270] = 0;
   out_2434308811775089426[271] = 0;
   out_2434308811775089426[272] = 0;
   out_2434308811775089426[273] = 0;
   out_2434308811775089426[274] = 0;
   out_2434308811775089426[275] = 0;
   out_2434308811775089426[276] = 0;
   out_2434308811775089426[277] = 0;
   out_2434308811775089426[278] = 0;
   out_2434308811775089426[279] = 0;
   out_2434308811775089426[280] = 0;
   out_2434308811775089426[281] = 0;
   out_2434308811775089426[282] = 0;
   out_2434308811775089426[283] = 0;
   out_2434308811775089426[284] = 0;
   out_2434308811775089426[285] = 1;
   out_2434308811775089426[286] = 0;
   out_2434308811775089426[287] = 0;
   out_2434308811775089426[288] = 0;
   out_2434308811775089426[289] = 0;
   out_2434308811775089426[290] = 0;
   out_2434308811775089426[291] = 0;
   out_2434308811775089426[292] = 0;
   out_2434308811775089426[293] = 0;
   out_2434308811775089426[294] = 0;
   out_2434308811775089426[295] = 0;
   out_2434308811775089426[296] = 0;
   out_2434308811775089426[297] = 0;
   out_2434308811775089426[298] = 0;
   out_2434308811775089426[299] = 0;
   out_2434308811775089426[300] = 0;
   out_2434308811775089426[301] = 0;
   out_2434308811775089426[302] = 0;
   out_2434308811775089426[303] = 0;
   out_2434308811775089426[304] = 1;
   out_2434308811775089426[305] = 0;
   out_2434308811775089426[306] = 0;
   out_2434308811775089426[307] = 0;
   out_2434308811775089426[308] = 0;
   out_2434308811775089426[309] = 0;
   out_2434308811775089426[310] = 0;
   out_2434308811775089426[311] = 0;
   out_2434308811775089426[312] = 0;
   out_2434308811775089426[313] = 0;
   out_2434308811775089426[314] = 0;
   out_2434308811775089426[315] = 0;
   out_2434308811775089426[316] = 0;
   out_2434308811775089426[317] = 0;
   out_2434308811775089426[318] = 0;
   out_2434308811775089426[319] = 0;
   out_2434308811775089426[320] = 0;
   out_2434308811775089426[321] = 0;
   out_2434308811775089426[322] = 0;
   out_2434308811775089426[323] = 1;
}
void h_4(double *state, double *unused, double *out_480079106382461241) {
   out_480079106382461241[0] = state[6] + state[9];
   out_480079106382461241[1] = state[7] + state[10];
   out_480079106382461241[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5921568430080383329) {
   out_5921568430080383329[0] = 0;
   out_5921568430080383329[1] = 0;
   out_5921568430080383329[2] = 0;
   out_5921568430080383329[3] = 0;
   out_5921568430080383329[4] = 0;
   out_5921568430080383329[5] = 0;
   out_5921568430080383329[6] = 1;
   out_5921568430080383329[7] = 0;
   out_5921568430080383329[8] = 0;
   out_5921568430080383329[9] = 1;
   out_5921568430080383329[10] = 0;
   out_5921568430080383329[11] = 0;
   out_5921568430080383329[12] = 0;
   out_5921568430080383329[13] = 0;
   out_5921568430080383329[14] = 0;
   out_5921568430080383329[15] = 0;
   out_5921568430080383329[16] = 0;
   out_5921568430080383329[17] = 0;
   out_5921568430080383329[18] = 0;
   out_5921568430080383329[19] = 0;
   out_5921568430080383329[20] = 0;
   out_5921568430080383329[21] = 0;
   out_5921568430080383329[22] = 0;
   out_5921568430080383329[23] = 0;
   out_5921568430080383329[24] = 0;
   out_5921568430080383329[25] = 1;
   out_5921568430080383329[26] = 0;
   out_5921568430080383329[27] = 0;
   out_5921568430080383329[28] = 1;
   out_5921568430080383329[29] = 0;
   out_5921568430080383329[30] = 0;
   out_5921568430080383329[31] = 0;
   out_5921568430080383329[32] = 0;
   out_5921568430080383329[33] = 0;
   out_5921568430080383329[34] = 0;
   out_5921568430080383329[35] = 0;
   out_5921568430080383329[36] = 0;
   out_5921568430080383329[37] = 0;
   out_5921568430080383329[38] = 0;
   out_5921568430080383329[39] = 0;
   out_5921568430080383329[40] = 0;
   out_5921568430080383329[41] = 0;
   out_5921568430080383329[42] = 0;
   out_5921568430080383329[43] = 0;
   out_5921568430080383329[44] = 1;
   out_5921568430080383329[45] = 0;
   out_5921568430080383329[46] = 0;
   out_5921568430080383329[47] = 1;
   out_5921568430080383329[48] = 0;
   out_5921568430080383329[49] = 0;
   out_5921568430080383329[50] = 0;
   out_5921568430080383329[51] = 0;
   out_5921568430080383329[52] = 0;
   out_5921568430080383329[53] = 0;
}
void h_10(double *state, double *unused, double *out_5059097033008134263) {
   out_5059097033008134263[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5059097033008134263[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5059097033008134263[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3858084587156191705) {
   out_3858084587156191705[0] = 0;
   out_3858084587156191705[1] = 9.8100000000000005*cos(state[1]);
   out_3858084587156191705[2] = 0;
   out_3858084587156191705[3] = 0;
   out_3858084587156191705[4] = -state[8];
   out_3858084587156191705[5] = state[7];
   out_3858084587156191705[6] = 0;
   out_3858084587156191705[7] = state[5];
   out_3858084587156191705[8] = -state[4];
   out_3858084587156191705[9] = 0;
   out_3858084587156191705[10] = 0;
   out_3858084587156191705[11] = 0;
   out_3858084587156191705[12] = 1;
   out_3858084587156191705[13] = 0;
   out_3858084587156191705[14] = 0;
   out_3858084587156191705[15] = 1;
   out_3858084587156191705[16] = 0;
   out_3858084587156191705[17] = 0;
   out_3858084587156191705[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3858084587156191705[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3858084587156191705[20] = 0;
   out_3858084587156191705[21] = state[8];
   out_3858084587156191705[22] = 0;
   out_3858084587156191705[23] = -state[6];
   out_3858084587156191705[24] = -state[5];
   out_3858084587156191705[25] = 0;
   out_3858084587156191705[26] = state[3];
   out_3858084587156191705[27] = 0;
   out_3858084587156191705[28] = 0;
   out_3858084587156191705[29] = 0;
   out_3858084587156191705[30] = 0;
   out_3858084587156191705[31] = 1;
   out_3858084587156191705[32] = 0;
   out_3858084587156191705[33] = 0;
   out_3858084587156191705[34] = 1;
   out_3858084587156191705[35] = 0;
   out_3858084587156191705[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3858084587156191705[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3858084587156191705[38] = 0;
   out_3858084587156191705[39] = -state[7];
   out_3858084587156191705[40] = state[6];
   out_3858084587156191705[41] = 0;
   out_3858084587156191705[42] = state[4];
   out_3858084587156191705[43] = -state[3];
   out_3858084587156191705[44] = 0;
   out_3858084587156191705[45] = 0;
   out_3858084587156191705[46] = 0;
   out_3858084587156191705[47] = 0;
   out_3858084587156191705[48] = 0;
   out_3858084587156191705[49] = 0;
   out_3858084587156191705[50] = 1;
   out_3858084587156191705[51] = 0;
   out_3858084587156191705[52] = 0;
   out_3858084587156191705[53] = 1;
}
void h_13(double *state, double *unused, double *out_7754600168369625286) {
   out_7754600168369625286[0] = state[3];
   out_7754600168369625286[1] = state[4];
   out_7754600168369625286[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2709294604748050528) {
   out_2709294604748050528[0] = 0;
   out_2709294604748050528[1] = 0;
   out_2709294604748050528[2] = 0;
   out_2709294604748050528[3] = 1;
   out_2709294604748050528[4] = 0;
   out_2709294604748050528[5] = 0;
   out_2709294604748050528[6] = 0;
   out_2709294604748050528[7] = 0;
   out_2709294604748050528[8] = 0;
   out_2709294604748050528[9] = 0;
   out_2709294604748050528[10] = 0;
   out_2709294604748050528[11] = 0;
   out_2709294604748050528[12] = 0;
   out_2709294604748050528[13] = 0;
   out_2709294604748050528[14] = 0;
   out_2709294604748050528[15] = 0;
   out_2709294604748050528[16] = 0;
   out_2709294604748050528[17] = 0;
   out_2709294604748050528[18] = 0;
   out_2709294604748050528[19] = 0;
   out_2709294604748050528[20] = 0;
   out_2709294604748050528[21] = 0;
   out_2709294604748050528[22] = 1;
   out_2709294604748050528[23] = 0;
   out_2709294604748050528[24] = 0;
   out_2709294604748050528[25] = 0;
   out_2709294604748050528[26] = 0;
   out_2709294604748050528[27] = 0;
   out_2709294604748050528[28] = 0;
   out_2709294604748050528[29] = 0;
   out_2709294604748050528[30] = 0;
   out_2709294604748050528[31] = 0;
   out_2709294604748050528[32] = 0;
   out_2709294604748050528[33] = 0;
   out_2709294604748050528[34] = 0;
   out_2709294604748050528[35] = 0;
   out_2709294604748050528[36] = 0;
   out_2709294604748050528[37] = 0;
   out_2709294604748050528[38] = 0;
   out_2709294604748050528[39] = 0;
   out_2709294604748050528[40] = 0;
   out_2709294604748050528[41] = 1;
   out_2709294604748050528[42] = 0;
   out_2709294604748050528[43] = 0;
   out_2709294604748050528[44] = 0;
   out_2709294604748050528[45] = 0;
   out_2709294604748050528[46] = 0;
   out_2709294604748050528[47] = 0;
   out_2709294604748050528[48] = 0;
   out_2709294604748050528[49] = 0;
   out_2709294604748050528[50] = 0;
   out_2709294604748050528[51] = 0;
   out_2709294604748050528[52] = 0;
   out_2709294604748050528[53] = 0;
}
void h_14(double *state, double *unused, double *out_5590979122691983286) {
   out_5590979122691983286[0] = state[6];
   out_5590979122691983286[1] = state[7];
   out_5590979122691983286[2] = state[8];
}
void H_14(double *state, double *unused, double *out_1958327573740898800) {
   out_1958327573740898800[0] = 0;
   out_1958327573740898800[1] = 0;
   out_1958327573740898800[2] = 0;
   out_1958327573740898800[3] = 0;
   out_1958327573740898800[4] = 0;
   out_1958327573740898800[5] = 0;
   out_1958327573740898800[6] = 1;
   out_1958327573740898800[7] = 0;
   out_1958327573740898800[8] = 0;
   out_1958327573740898800[9] = 0;
   out_1958327573740898800[10] = 0;
   out_1958327573740898800[11] = 0;
   out_1958327573740898800[12] = 0;
   out_1958327573740898800[13] = 0;
   out_1958327573740898800[14] = 0;
   out_1958327573740898800[15] = 0;
   out_1958327573740898800[16] = 0;
   out_1958327573740898800[17] = 0;
   out_1958327573740898800[18] = 0;
   out_1958327573740898800[19] = 0;
   out_1958327573740898800[20] = 0;
   out_1958327573740898800[21] = 0;
   out_1958327573740898800[22] = 0;
   out_1958327573740898800[23] = 0;
   out_1958327573740898800[24] = 0;
   out_1958327573740898800[25] = 1;
   out_1958327573740898800[26] = 0;
   out_1958327573740898800[27] = 0;
   out_1958327573740898800[28] = 0;
   out_1958327573740898800[29] = 0;
   out_1958327573740898800[30] = 0;
   out_1958327573740898800[31] = 0;
   out_1958327573740898800[32] = 0;
   out_1958327573740898800[33] = 0;
   out_1958327573740898800[34] = 0;
   out_1958327573740898800[35] = 0;
   out_1958327573740898800[36] = 0;
   out_1958327573740898800[37] = 0;
   out_1958327573740898800[38] = 0;
   out_1958327573740898800[39] = 0;
   out_1958327573740898800[40] = 0;
   out_1958327573740898800[41] = 0;
   out_1958327573740898800[42] = 0;
   out_1958327573740898800[43] = 0;
   out_1958327573740898800[44] = 1;
   out_1958327573740898800[45] = 0;
   out_1958327573740898800[46] = 0;
   out_1958327573740898800[47] = 0;
   out_1958327573740898800[48] = 0;
   out_1958327573740898800[49] = 0;
   out_1958327573740898800[50] = 0;
   out_1958327573740898800[51] = 0;
   out_1958327573740898800[52] = 0;
   out_1958327573740898800[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4705197801635651257) {
  err_fun(nom_x, delta_x, out_4705197801635651257);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4283461203773073958) {
  inv_err_fun(nom_x, true_x, out_4283461203773073958);
}
void pose_H_mod_fun(double *state, double *out_3022793235977438013) {
  H_mod_fun(state, out_3022793235977438013);
}
void pose_f_fun(double *state, double dt, double *out_3860426828173334801) {
  f_fun(state,  dt, out_3860426828173334801);
}
void pose_F_fun(double *state, double dt, double *out_2434308811775089426) {
  F_fun(state,  dt, out_2434308811775089426);
}
void pose_h_4(double *state, double *unused, double *out_480079106382461241) {
  h_4(state, unused, out_480079106382461241);
}
void pose_H_4(double *state, double *unused, double *out_5921568430080383329) {
  H_4(state, unused, out_5921568430080383329);
}
void pose_h_10(double *state, double *unused, double *out_5059097033008134263) {
  h_10(state, unused, out_5059097033008134263);
}
void pose_H_10(double *state, double *unused, double *out_3858084587156191705) {
  H_10(state, unused, out_3858084587156191705);
}
void pose_h_13(double *state, double *unused, double *out_7754600168369625286) {
  h_13(state, unused, out_7754600168369625286);
}
void pose_H_13(double *state, double *unused, double *out_2709294604748050528) {
  H_13(state, unused, out_2709294604748050528);
}
void pose_h_14(double *state, double *unused, double *out_5590979122691983286) {
  h_14(state, unused, out_5590979122691983286);
}
void pose_H_14(double *state, double *unused, double *out_1958327573740898800) {
  H_14(state, unused, out_1958327573740898800);
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
