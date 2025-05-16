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
void err_fun(double *nom_x, double *delta_x, double *out_8353524553736122963) {
   out_8353524553736122963[0] = delta_x[0] + nom_x[0];
   out_8353524553736122963[1] = delta_x[1] + nom_x[1];
   out_8353524553736122963[2] = delta_x[2] + nom_x[2];
   out_8353524553736122963[3] = delta_x[3] + nom_x[3];
   out_8353524553736122963[4] = delta_x[4] + nom_x[4];
   out_8353524553736122963[5] = delta_x[5] + nom_x[5];
   out_8353524553736122963[6] = delta_x[6] + nom_x[6];
   out_8353524553736122963[7] = delta_x[7] + nom_x[7];
   out_8353524553736122963[8] = delta_x[8] + nom_x[8];
   out_8353524553736122963[9] = delta_x[9] + nom_x[9];
   out_8353524553736122963[10] = delta_x[10] + nom_x[10];
   out_8353524553736122963[11] = delta_x[11] + nom_x[11];
   out_8353524553736122963[12] = delta_x[12] + nom_x[12];
   out_8353524553736122963[13] = delta_x[13] + nom_x[13];
   out_8353524553736122963[14] = delta_x[14] + nom_x[14];
   out_8353524553736122963[15] = delta_x[15] + nom_x[15];
   out_8353524553736122963[16] = delta_x[16] + nom_x[16];
   out_8353524553736122963[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8983699723827009156) {
   out_8983699723827009156[0] = -nom_x[0] + true_x[0];
   out_8983699723827009156[1] = -nom_x[1] + true_x[1];
   out_8983699723827009156[2] = -nom_x[2] + true_x[2];
   out_8983699723827009156[3] = -nom_x[3] + true_x[3];
   out_8983699723827009156[4] = -nom_x[4] + true_x[4];
   out_8983699723827009156[5] = -nom_x[5] + true_x[5];
   out_8983699723827009156[6] = -nom_x[6] + true_x[6];
   out_8983699723827009156[7] = -nom_x[7] + true_x[7];
   out_8983699723827009156[8] = -nom_x[8] + true_x[8];
   out_8983699723827009156[9] = -nom_x[9] + true_x[9];
   out_8983699723827009156[10] = -nom_x[10] + true_x[10];
   out_8983699723827009156[11] = -nom_x[11] + true_x[11];
   out_8983699723827009156[12] = -nom_x[12] + true_x[12];
   out_8983699723827009156[13] = -nom_x[13] + true_x[13];
   out_8983699723827009156[14] = -nom_x[14] + true_x[14];
   out_8983699723827009156[15] = -nom_x[15] + true_x[15];
   out_8983699723827009156[16] = -nom_x[16] + true_x[16];
   out_8983699723827009156[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2968799295363009369) {
   out_2968799295363009369[0] = 1.0;
   out_2968799295363009369[1] = 0.0;
   out_2968799295363009369[2] = 0.0;
   out_2968799295363009369[3] = 0.0;
   out_2968799295363009369[4] = 0.0;
   out_2968799295363009369[5] = 0.0;
   out_2968799295363009369[6] = 0.0;
   out_2968799295363009369[7] = 0.0;
   out_2968799295363009369[8] = 0.0;
   out_2968799295363009369[9] = 0.0;
   out_2968799295363009369[10] = 0.0;
   out_2968799295363009369[11] = 0.0;
   out_2968799295363009369[12] = 0.0;
   out_2968799295363009369[13] = 0.0;
   out_2968799295363009369[14] = 0.0;
   out_2968799295363009369[15] = 0.0;
   out_2968799295363009369[16] = 0.0;
   out_2968799295363009369[17] = 0.0;
   out_2968799295363009369[18] = 0.0;
   out_2968799295363009369[19] = 1.0;
   out_2968799295363009369[20] = 0.0;
   out_2968799295363009369[21] = 0.0;
   out_2968799295363009369[22] = 0.0;
   out_2968799295363009369[23] = 0.0;
   out_2968799295363009369[24] = 0.0;
   out_2968799295363009369[25] = 0.0;
   out_2968799295363009369[26] = 0.0;
   out_2968799295363009369[27] = 0.0;
   out_2968799295363009369[28] = 0.0;
   out_2968799295363009369[29] = 0.0;
   out_2968799295363009369[30] = 0.0;
   out_2968799295363009369[31] = 0.0;
   out_2968799295363009369[32] = 0.0;
   out_2968799295363009369[33] = 0.0;
   out_2968799295363009369[34] = 0.0;
   out_2968799295363009369[35] = 0.0;
   out_2968799295363009369[36] = 0.0;
   out_2968799295363009369[37] = 0.0;
   out_2968799295363009369[38] = 1.0;
   out_2968799295363009369[39] = 0.0;
   out_2968799295363009369[40] = 0.0;
   out_2968799295363009369[41] = 0.0;
   out_2968799295363009369[42] = 0.0;
   out_2968799295363009369[43] = 0.0;
   out_2968799295363009369[44] = 0.0;
   out_2968799295363009369[45] = 0.0;
   out_2968799295363009369[46] = 0.0;
   out_2968799295363009369[47] = 0.0;
   out_2968799295363009369[48] = 0.0;
   out_2968799295363009369[49] = 0.0;
   out_2968799295363009369[50] = 0.0;
   out_2968799295363009369[51] = 0.0;
   out_2968799295363009369[52] = 0.0;
   out_2968799295363009369[53] = 0.0;
   out_2968799295363009369[54] = 0.0;
   out_2968799295363009369[55] = 0.0;
   out_2968799295363009369[56] = 0.0;
   out_2968799295363009369[57] = 1.0;
   out_2968799295363009369[58] = 0.0;
   out_2968799295363009369[59] = 0.0;
   out_2968799295363009369[60] = 0.0;
   out_2968799295363009369[61] = 0.0;
   out_2968799295363009369[62] = 0.0;
   out_2968799295363009369[63] = 0.0;
   out_2968799295363009369[64] = 0.0;
   out_2968799295363009369[65] = 0.0;
   out_2968799295363009369[66] = 0.0;
   out_2968799295363009369[67] = 0.0;
   out_2968799295363009369[68] = 0.0;
   out_2968799295363009369[69] = 0.0;
   out_2968799295363009369[70] = 0.0;
   out_2968799295363009369[71] = 0.0;
   out_2968799295363009369[72] = 0.0;
   out_2968799295363009369[73] = 0.0;
   out_2968799295363009369[74] = 0.0;
   out_2968799295363009369[75] = 0.0;
   out_2968799295363009369[76] = 1.0;
   out_2968799295363009369[77] = 0.0;
   out_2968799295363009369[78] = 0.0;
   out_2968799295363009369[79] = 0.0;
   out_2968799295363009369[80] = 0.0;
   out_2968799295363009369[81] = 0.0;
   out_2968799295363009369[82] = 0.0;
   out_2968799295363009369[83] = 0.0;
   out_2968799295363009369[84] = 0.0;
   out_2968799295363009369[85] = 0.0;
   out_2968799295363009369[86] = 0.0;
   out_2968799295363009369[87] = 0.0;
   out_2968799295363009369[88] = 0.0;
   out_2968799295363009369[89] = 0.0;
   out_2968799295363009369[90] = 0.0;
   out_2968799295363009369[91] = 0.0;
   out_2968799295363009369[92] = 0.0;
   out_2968799295363009369[93] = 0.0;
   out_2968799295363009369[94] = 0.0;
   out_2968799295363009369[95] = 1.0;
   out_2968799295363009369[96] = 0.0;
   out_2968799295363009369[97] = 0.0;
   out_2968799295363009369[98] = 0.0;
   out_2968799295363009369[99] = 0.0;
   out_2968799295363009369[100] = 0.0;
   out_2968799295363009369[101] = 0.0;
   out_2968799295363009369[102] = 0.0;
   out_2968799295363009369[103] = 0.0;
   out_2968799295363009369[104] = 0.0;
   out_2968799295363009369[105] = 0.0;
   out_2968799295363009369[106] = 0.0;
   out_2968799295363009369[107] = 0.0;
   out_2968799295363009369[108] = 0.0;
   out_2968799295363009369[109] = 0.0;
   out_2968799295363009369[110] = 0.0;
   out_2968799295363009369[111] = 0.0;
   out_2968799295363009369[112] = 0.0;
   out_2968799295363009369[113] = 0.0;
   out_2968799295363009369[114] = 1.0;
   out_2968799295363009369[115] = 0.0;
   out_2968799295363009369[116] = 0.0;
   out_2968799295363009369[117] = 0.0;
   out_2968799295363009369[118] = 0.0;
   out_2968799295363009369[119] = 0.0;
   out_2968799295363009369[120] = 0.0;
   out_2968799295363009369[121] = 0.0;
   out_2968799295363009369[122] = 0.0;
   out_2968799295363009369[123] = 0.0;
   out_2968799295363009369[124] = 0.0;
   out_2968799295363009369[125] = 0.0;
   out_2968799295363009369[126] = 0.0;
   out_2968799295363009369[127] = 0.0;
   out_2968799295363009369[128] = 0.0;
   out_2968799295363009369[129] = 0.0;
   out_2968799295363009369[130] = 0.0;
   out_2968799295363009369[131] = 0.0;
   out_2968799295363009369[132] = 0.0;
   out_2968799295363009369[133] = 1.0;
   out_2968799295363009369[134] = 0.0;
   out_2968799295363009369[135] = 0.0;
   out_2968799295363009369[136] = 0.0;
   out_2968799295363009369[137] = 0.0;
   out_2968799295363009369[138] = 0.0;
   out_2968799295363009369[139] = 0.0;
   out_2968799295363009369[140] = 0.0;
   out_2968799295363009369[141] = 0.0;
   out_2968799295363009369[142] = 0.0;
   out_2968799295363009369[143] = 0.0;
   out_2968799295363009369[144] = 0.0;
   out_2968799295363009369[145] = 0.0;
   out_2968799295363009369[146] = 0.0;
   out_2968799295363009369[147] = 0.0;
   out_2968799295363009369[148] = 0.0;
   out_2968799295363009369[149] = 0.0;
   out_2968799295363009369[150] = 0.0;
   out_2968799295363009369[151] = 0.0;
   out_2968799295363009369[152] = 1.0;
   out_2968799295363009369[153] = 0.0;
   out_2968799295363009369[154] = 0.0;
   out_2968799295363009369[155] = 0.0;
   out_2968799295363009369[156] = 0.0;
   out_2968799295363009369[157] = 0.0;
   out_2968799295363009369[158] = 0.0;
   out_2968799295363009369[159] = 0.0;
   out_2968799295363009369[160] = 0.0;
   out_2968799295363009369[161] = 0.0;
   out_2968799295363009369[162] = 0.0;
   out_2968799295363009369[163] = 0.0;
   out_2968799295363009369[164] = 0.0;
   out_2968799295363009369[165] = 0.0;
   out_2968799295363009369[166] = 0.0;
   out_2968799295363009369[167] = 0.0;
   out_2968799295363009369[168] = 0.0;
   out_2968799295363009369[169] = 0.0;
   out_2968799295363009369[170] = 0.0;
   out_2968799295363009369[171] = 1.0;
   out_2968799295363009369[172] = 0.0;
   out_2968799295363009369[173] = 0.0;
   out_2968799295363009369[174] = 0.0;
   out_2968799295363009369[175] = 0.0;
   out_2968799295363009369[176] = 0.0;
   out_2968799295363009369[177] = 0.0;
   out_2968799295363009369[178] = 0.0;
   out_2968799295363009369[179] = 0.0;
   out_2968799295363009369[180] = 0.0;
   out_2968799295363009369[181] = 0.0;
   out_2968799295363009369[182] = 0.0;
   out_2968799295363009369[183] = 0.0;
   out_2968799295363009369[184] = 0.0;
   out_2968799295363009369[185] = 0.0;
   out_2968799295363009369[186] = 0.0;
   out_2968799295363009369[187] = 0.0;
   out_2968799295363009369[188] = 0.0;
   out_2968799295363009369[189] = 0.0;
   out_2968799295363009369[190] = 1.0;
   out_2968799295363009369[191] = 0.0;
   out_2968799295363009369[192] = 0.0;
   out_2968799295363009369[193] = 0.0;
   out_2968799295363009369[194] = 0.0;
   out_2968799295363009369[195] = 0.0;
   out_2968799295363009369[196] = 0.0;
   out_2968799295363009369[197] = 0.0;
   out_2968799295363009369[198] = 0.0;
   out_2968799295363009369[199] = 0.0;
   out_2968799295363009369[200] = 0.0;
   out_2968799295363009369[201] = 0.0;
   out_2968799295363009369[202] = 0.0;
   out_2968799295363009369[203] = 0.0;
   out_2968799295363009369[204] = 0.0;
   out_2968799295363009369[205] = 0.0;
   out_2968799295363009369[206] = 0.0;
   out_2968799295363009369[207] = 0.0;
   out_2968799295363009369[208] = 0.0;
   out_2968799295363009369[209] = 1.0;
   out_2968799295363009369[210] = 0.0;
   out_2968799295363009369[211] = 0.0;
   out_2968799295363009369[212] = 0.0;
   out_2968799295363009369[213] = 0.0;
   out_2968799295363009369[214] = 0.0;
   out_2968799295363009369[215] = 0.0;
   out_2968799295363009369[216] = 0.0;
   out_2968799295363009369[217] = 0.0;
   out_2968799295363009369[218] = 0.0;
   out_2968799295363009369[219] = 0.0;
   out_2968799295363009369[220] = 0.0;
   out_2968799295363009369[221] = 0.0;
   out_2968799295363009369[222] = 0.0;
   out_2968799295363009369[223] = 0.0;
   out_2968799295363009369[224] = 0.0;
   out_2968799295363009369[225] = 0.0;
   out_2968799295363009369[226] = 0.0;
   out_2968799295363009369[227] = 0.0;
   out_2968799295363009369[228] = 1.0;
   out_2968799295363009369[229] = 0.0;
   out_2968799295363009369[230] = 0.0;
   out_2968799295363009369[231] = 0.0;
   out_2968799295363009369[232] = 0.0;
   out_2968799295363009369[233] = 0.0;
   out_2968799295363009369[234] = 0.0;
   out_2968799295363009369[235] = 0.0;
   out_2968799295363009369[236] = 0.0;
   out_2968799295363009369[237] = 0.0;
   out_2968799295363009369[238] = 0.0;
   out_2968799295363009369[239] = 0.0;
   out_2968799295363009369[240] = 0.0;
   out_2968799295363009369[241] = 0.0;
   out_2968799295363009369[242] = 0.0;
   out_2968799295363009369[243] = 0.0;
   out_2968799295363009369[244] = 0.0;
   out_2968799295363009369[245] = 0.0;
   out_2968799295363009369[246] = 0.0;
   out_2968799295363009369[247] = 1.0;
   out_2968799295363009369[248] = 0.0;
   out_2968799295363009369[249] = 0.0;
   out_2968799295363009369[250] = 0.0;
   out_2968799295363009369[251] = 0.0;
   out_2968799295363009369[252] = 0.0;
   out_2968799295363009369[253] = 0.0;
   out_2968799295363009369[254] = 0.0;
   out_2968799295363009369[255] = 0.0;
   out_2968799295363009369[256] = 0.0;
   out_2968799295363009369[257] = 0.0;
   out_2968799295363009369[258] = 0.0;
   out_2968799295363009369[259] = 0.0;
   out_2968799295363009369[260] = 0.0;
   out_2968799295363009369[261] = 0.0;
   out_2968799295363009369[262] = 0.0;
   out_2968799295363009369[263] = 0.0;
   out_2968799295363009369[264] = 0.0;
   out_2968799295363009369[265] = 0.0;
   out_2968799295363009369[266] = 1.0;
   out_2968799295363009369[267] = 0.0;
   out_2968799295363009369[268] = 0.0;
   out_2968799295363009369[269] = 0.0;
   out_2968799295363009369[270] = 0.0;
   out_2968799295363009369[271] = 0.0;
   out_2968799295363009369[272] = 0.0;
   out_2968799295363009369[273] = 0.0;
   out_2968799295363009369[274] = 0.0;
   out_2968799295363009369[275] = 0.0;
   out_2968799295363009369[276] = 0.0;
   out_2968799295363009369[277] = 0.0;
   out_2968799295363009369[278] = 0.0;
   out_2968799295363009369[279] = 0.0;
   out_2968799295363009369[280] = 0.0;
   out_2968799295363009369[281] = 0.0;
   out_2968799295363009369[282] = 0.0;
   out_2968799295363009369[283] = 0.0;
   out_2968799295363009369[284] = 0.0;
   out_2968799295363009369[285] = 1.0;
   out_2968799295363009369[286] = 0.0;
   out_2968799295363009369[287] = 0.0;
   out_2968799295363009369[288] = 0.0;
   out_2968799295363009369[289] = 0.0;
   out_2968799295363009369[290] = 0.0;
   out_2968799295363009369[291] = 0.0;
   out_2968799295363009369[292] = 0.0;
   out_2968799295363009369[293] = 0.0;
   out_2968799295363009369[294] = 0.0;
   out_2968799295363009369[295] = 0.0;
   out_2968799295363009369[296] = 0.0;
   out_2968799295363009369[297] = 0.0;
   out_2968799295363009369[298] = 0.0;
   out_2968799295363009369[299] = 0.0;
   out_2968799295363009369[300] = 0.0;
   out_2968799295363009369[301] = 0.0;
   out_2968799295363009369[302] = 0.0;
   out_2968799295363009369[303] = 0.0;
   out_2968799295363009369[304] = 1.0;
   out_2968799295363009369[305] = 0.0;
   out_2968799295363009369[306] = 0.0;
   out_2968799295363009369[307] = 0.0;
   out_2968799295363009369[308] = 0.0;
   out_2968799295363009369[309] = 0.0;
   out_2968799295363009369[310] = 0.0;
   out_2968799295363009369[311] = 0.0;
   out_2968799295363009369[312] = 0.0;
   out_2968799295363009369[313] = 0.0;
   out_2968799295363009369[314] = 0.0;
   out_2968799295363009369[315] = 0.0;
   out_2968799295363009369[316] = 0.0;
   out_2968799295363009369[317] = 0.0;
   out_2968799295363009369[318] = 0.0;
   out_2968799295363009369[319] = 0.0;
   out_2968799295363009369[320] = 0.0;
   out_2968799295363009369[321] = 0.0;
   out_2968799295363009369[322] = 0.0;
   out_2968799295363009369[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4017585182654342073) {
   out_4017585182654342073[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4017585182654342073[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4017585182654342073[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4017585182654342073[3] = dt*state[12] + state[3];
   out_4017585182654342073[4] = dt*state[13] + state[4];
   out_4017585182654342073[5] = dt*state[14] + state[5];
   out_4017585182654342073[6] = state[6];
   out_4017585182654342073[7] = state[7];
   out_4017585182654342073[8] = state[8];
   out_4017585182654342073[9] = state[9];
   out_4017585182654342073[10] = state[10];
   out_4017585182654342073[11] = state[11];
   out_4017585182654342073[12] = state[12];
   out_4017585182654342073[13] = state[13];
   out_4017585182654342073[14] = state[14];
   out_4017585182654342073[15] = state[15];
   out_4017585182654342073[16] = state[16];
   out_4017585182654342073[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6528773045854873690) {
   out_6528773045854873690[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6528773045854873690[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6528773045854873690[2] = 0;
   out_6528773045854873690[3] = 0;
   out_6528773045854873690[4] = 0;
   out_6528773045854873690[5] = 0;
   out_6528773045854873690[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6528773045854873690[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6528773045854873690[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6528773045854873690[9] = 0;
   out_6528773045854873690[10] = 0;
   out_6528773045854873690[11] = 0;
   out_6528773045854873690[12] = 0;
   out_6528773045854873690[13] = 0;
   out_6528773045854873690[14] = 0;
   out_6528773045854873690[15] = 0;
   out_6528773045854873690[16] = 0;
   out_6528773045854873690[17] = 0;
   out_6528773045854873690[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6528773045854873690[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6528773045854873690[20] = 0;
   out_6528773045854873690[21] = 0;
   out_6528773045854873690[22] = 0;
   out_6528773045854873690[23] = 0;
   out_6528773045854873690[24] = 0;
   out_6528773045854873690[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6528773045854873690[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6528773045854873690[27] = 0;
   out_6528773045854873690[28] = 0;
   out_6528773045854873690[29] = 0;
   out_6528773045854873690[30] = 0;
   out_6528773045854873690[31] = 0;
   out_6528773045854873690[32] = 0;
   out_6528773045854873690[33] = 0;
   out_6528773045854873690[34] = 0;
   out_6528773045854873690[35] = 0;
   out_6528773045854873690[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6528773045854873690[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6528773045854873690[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6528773045854873690[39] = 0;
   out_6528773045854873690[40] = 0;
   out_6528773045854873690[41] = 0;
   out_6528773045854873690[42] = 0;
   out_6528773045854873690[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6528773045854873690[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6528773045854873690[45] = 0;
   out_6528773045854873690[46] = 0;
   out_6528773045854873690[47] = 0;
   out_6528773045854873690[48] = 0;
   out_6528773045854873690[49] = 0;
   out_6528773045854873690[50] = 0;
   out_6528773045854873690[51] = 0;
   out_6528773045854873690[52] = 0;
   out_6528773045854873690[53] = 0;
   out_6528773045854873690[54] = 0;
   out_6528773045854873690[55] = 0;
   out_6528773045854873690[56] = 0;
   out_6528773045854873690[57] = 1;
   out_6528773045854873690[58] = 0;
   out_6528773045854873690[59] = 0;
   out_6528773045854873690[60] = 0;
   out_6528773045854873690[61] = 0;
   out_6528773045854873690[62] = 0;
   out_6528773045854873690[63] = 0;
   out_6528773045854873690[64] = 0;
   out_6528773045854873690[65] = 0;
   out_6528773045854873690[66] = dt;
   out_6528773045854873690[67] = 0;
   out_6528773045854873690[68] = 0;
   out_6528773045854873690[69] = 0;
   out_6528773045854873690[70] = 0;
   out_6528773045854873690[71] = 0;
   out_6528773045854873690[72] = 0;
   out_6528773045854873690[73] = 0;
   out_6528773045854873690[74] = 0;
   out_6528773045854873690[75] = 0;
   out_6528773045854873690[76] = 1;
   out_6528773045854873690[77] = 0;
   out_6528773045854873690[78] = 0;
   out_6528773045854873690[79] = 0;
   out_6528773045854873690[80] = 0;
   out_6528773045854873690[81] = 0;
   out_6528773045854873690[82] = 0;
   out_6528773045854873690[83] = 0;
   out_6528773045854873690[84] = 0;
   out_6528773045854873690[85] = dt;
   out_6528773045854873690[86] = 0;
   out_6528773045854873690[87] = 0;
   out_6528773045854873690[88] = 0;
   out_6528773045854873690[89] = 0;
   out_6528773045854873690[90] = 0;
   out_6528773045854873690[91] = 0;
   out_6528773045854873690[92] = 0;
   out_6528773045854873690[93] = 0;
   out_6528773045854873690[94] = 0;
   out_6528773045854873690[95] = 1;
   out_6528773045854873690[96] = 0;
   out_6528773045854873690[97] = 0;
   out_6528773045854873690[98] = 0;
   out_6528773045854873690[99] = 0;
   out_6528773045854873690[100] = 0;
   out_6528773045854873690[101] = 0;
   out_6528773045854873690[102] = 0;
   out_6528773045854873690[103] = 0;
   out_6528773045854873690[104] = dt;
   out_6528773045854873690[105] = 0;
   out_6528773045854873690[106] = 0;
   out_6528773045854873690[107] = 0;
   out_6528773045854873690[108] = 0;
   out_6528773045854873690[109] = 0;
   out_6528773045854873690[110] = 0;
   out_6528773045854873690[111] = 0;
   out_6528773045854873690[112] = 0;
   out_6528773045854873690[113] = 0;
   out_6528773045854873690[114] = 1;
   out_6528773045854873690[115] = 0;
   out_6528773045854873690[116] = 0;
   out_6528773045854873690[117] = 0;
   out_6528773045854873690[118] = 0;
   out_6528773045854873690[119] = 0;
   out_6528773045854873690[120] = 0;
   out_6528773045854873690[121] = 0;
   out_6528773045854873690[122] = 0;
   out_6528773045854873690[123] = 0;
   out_6528773045854873690[124] = 0;
   out_6528773045854873690[125] = 0;
   out_6528773045854873690[126] = 0;
   out_6528773045854873690[127] = 0;
   out_6528773045854873690[128] = 0;
   out_6528773045854873690[129] = 0;
   out_6528773045854873690[130] = 0;
   out_6528773045854873690[131] = 0;
   out_6528773045854873690[132] = 0;
   out_6528773045854873690[133] = 1;
   out_6528773045854873690[134] = 0;
   out_6528773045854873690[135] = 0;
   out_6528773045854873690[136] = 0;
   out_6528773045854873690[137] = 0;
   out_6528773045854873690[138] = 0;
   out_6528773045854873690[139] = 0;
   out_6528773045854873690[140] = 0;
   out_6528773045854873690[141] = 0;
   out_6528773045854873690[142] = 0;
   out_6528773045854873690[143] = 0;
   out_6528773045854873690[144] = 0;
   out_6528773045854873690[145] = 0;
   out_6528773045854873690[146] = 0;
   out_6528773045854873690[147] = 0;
   out_6528773045854873690[148] = 0;
   out_6528773045854873690[149] = 0;
   out_6528773045854873690[150] = 0;
   out_6528773045854873690[151] = 0;
   out_6528773045854873690[152] = 1;
   out_6528773045854873690[153] = 0;
   out_6528773045854873690[154] = 0;
   out_6528773045854873690[155] = 0;
   out_6528773045854873690[156] = 0;
   out_6528773045854873690[157] = 0;
   out_6528773045854873690[158] = 0;
   out_6528773045854873690[159] = 0;
   out_6528773045854873690[160] = 0;
   out_6528773045854873690[161] = 0;
   out_6528773045854873690[162] = 0;
   out_6528773045854873690[163] = 0;
   out_6528773045854873690[164] = 0;
   out_6528773045854873690[165] = 0;
   out_6528773045854873690[166] = 0;
   out_6528773045854873690[167] = 0;
   out_6528773045854873690[168] = 0;
   out_6528773045854873690[169] = 0;
   out_6528773045854873690[170] = 0;
   out_6528773045854873690[171] = 1;
   out_6528773045854873690[172] = 0;
   out_6528773045854873690[173] = 0;
   out_6528773045854873690[174] = 0;
   out_6528773045854873690[175] = 0;
   out_6528773045854873690[176] = 0;
   out_6528773045854873690[177] = 0;
   out_6528773045854873690[178] = 0;
   out_6528773045854873690[179] = 0;
   out_6528773045854873690[180] = 0;
   out_6528773045854873690[181] = 0;
   out_6528773045854873690[182] = 0;
   out_6528773045854873690[183] = 0;
   out_6528773045854873690[184] = 0;
   out_6528773045854873690[185] = 0;
   out_6528773045854873690[186] = 0;
   out_6528773045854873690[187] = 0;
   out_6528773045854873690[188] = 0;
   out_6528773045854873690[189] = 0;
   out_6528773045854873690[190] = 1;
   out_6528773045854873690[191] = 0;
   out_6528773045854873690[192] = 0;
   out_6528773045854873690[193] = 0;
   out_6528773045854873690[194] = 0;
   out_6528773045854873690[195] = 0;
   out_6528773045854873690[196] = 0;
   out_6528773045854873690[197] = 0;
   out_6528773045854873690[198] = 0;
   out_6528773045854873690[199] = 0;
   out_6528773045854873690[200] = 0;
   out_6528773045854873690[201] = 0;
   out_6528773045854873690[202] = 0;
   out_6528773045854873690[203] = 0;
   out_6528773045854873690[204] = 0;
   out_6528773045854873690[205] = 0;
   out_6528773045854873690[206] = 0;
   out_6528773045854873690[207] = 0;
   out_6528773045854873690[208] = 0;
   out_6528773045854873690[209] = 1;
   out_6528773045854873690[210] = 0;
   out_6528773045854873690[211] = 0;
   out_6528773045854873690[212] = 0;
   out_6528773045854873690[213] = 0;
   out_6528773045854873690[214] = 0;
   out_6528773045854873690[215] = 0;
   out_6528773045854873690[216] = 0;
   out_6528773045854873690[217] = 0;
   out_6528773045854873690[218] = 0;
   out_6528773045854873690[219] = 0;
   out_6528773045854873690[220] = 0;
   out_6528773045854873690[221] = 0;
   out_6528773045854873690[222] = 0;
   out_6528773045854873690[223] = 0;
   out_6528773045854873690[224] = 0;
   out_6528773045854873690[225] = 0;
   out_6528773045854873690[226] = 0;
   out_6528773045854873690[227] = 0;
   out_6528773045854873690[228] = 1;
   out_6528773045854873690[229] = 0;
   out_6528773045854873690[230] = 0;
   out_6528773045854873690[231] = 0;
   out_6528773045854873690[232] = 0;
   out_6528773045854873690[233] = 0;
   out_6528773045854873690[234] = 0;
   out_6528773045854873690[235] = 0;
   out_6528773045854873690[236] = 0;
   out_6528773045854873690[237] = 0;
   out_6528773045854873690[238] = 0;
   out_6528773045854873690[239] = 0;
   out_6528773045854873690[240] = 0;
   out_6528773045854873690[241] = 0;
   out_6528773045854873690[242] = 0;
   out_6528773045854873690[243] = 0;
   out_6528773045854873690[244] = 0;
   out_6528773045854873690[245] = 0;
   out_6528773045854873690[246] = 0;
   out_6528773045854873690[247] = 1;
   out_6528773045854873690[248] = 0;
   out_6528773045854873690[249] = 0;
   out_6528773045854873690[250] = 0;
   out_6528773045854873690[251] = 0;
   out_6528773045854873690[252] = 0;
   out_6528773045854873690[253] = 0;
   out_6528773045854873690[254] = 0;
   out_6528773045854873690[255] = 0;
   out_6528773045854873690[256] = 0;
   out_6528773045854873690[257] = 0;
   out_6528773045854873690[258] = 0;
   out_6528773045854873690[259] = 0;
   out_6528773045854873690[260] = 0;
   out_6528773045854873690[261] = 0;
   out_6528773045854873690[262] = 0;
   out_6528773045854873690[263] = 0;
   out_6528773045854873690[264] = 0;
   out_6528773045854873690[265] = 0;
   out_6528773045854873690[266] = 1;
   out_6528773045854873690[267] = 0;
   out_6528773045854873690[268] = 0;
   out_6528773045854873690[269] = 0;
   out_6528773045854873690[270] = 0;
   out_6528773045854873690[271] = 0;
   out_6528773045854873690[272] = 0;
   out_6528773045854873690[273] = 0;
   out_6528773045854873690[274] = 0;
   out_6528773045854873690[275] = 0;
   out_6528773045854873690[276] = 0;
   out_6528773045854873690[277] = 0;
   out_6528773045854873690[278] = 0;
   out_6528773045854873690[279] = 0;
   out_6528773045854873690[280] = 0;
   out_6528773045854873690[281] = 0;
   out_6528773045854873690[282] = 0;
   out_6528773045854873690[283] = 0;
   out_6528773045854873690[284] = 0;
   out_6528773045854873690[285] = 1;
   out_6528773045854873690[286] = 0;
   out_6528773045854873690[287] = 0;
   out_6528773045854873690[288] = 0;
   out_6528773045854873690[289] = 0;
   out_6528773045854873690[290] = 0;
   out_6528773045854873690[291] = 0;
   out_6528773045854873690[292] = 0;
   out_6528773045854873690[293] = 0;
   out_6528773045854873690[294] = 0;
   out_6528773045854873690[295] = 0;
   out_6528773045854873690[296] = 0;
   out_6528773045854873690[297] = 0;
   out_6528773045854873690[298] = 0;
   out_6528773045854873690[299] = 0;
   out_6528773045854873690[300] = 0;
   out_6528773045854873690[301] = 0;
   out_6528773045854873690[302] = 0;
   out_6528773045854873690[303] = 0;
   out_6528773045854873690[304] = 1;
   out_6528773045854873690[305] = 0;
   out_6528773045854873690[306] = 0;
   out_6528773045854873690[307] = 0;
   out_6528773045854873690[308] = 0;
   out_6528773045854873690[309] = 0;
   out_6528773045854873690[310] = 0;
   out_6528773045854873690[311] = 0;
   out_6528773045854873690[312] = 0;
   out_6528773045854873690[313] = 0;
   out_6528773045854873690[314] = 0;
   out_6528773045854873690[315] = 0;
   out_6528773045854873690[316] = 0;
   out_6528773045854873690[317] = 0;
   out_6528773045854873690[318] = 0;
   out_6528773045854873690[319] = 0;
   out_6528773045854873690[320] = 0;
   out_6528773045854873690[321] = 0;
   out_6528773045854873690[322] = 0;
   out_6528773045854873690[323] = 1;
}
void h_4(double *state, double *unused, double *out_2292486705639675299) {
   out_2292486705639675299[0] = state[6] + state[9];
   out_2292486705639675299[1] = state[7] + state[10];
   out_2292486705639675299[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8072824320030371515) {
   out_8072824320030371515[0] = 0;
   out_8072824320030371515[1] = 0;
   out_8072824320030371515[2] = 0;
   out_8072824320030371515[3] = 0;
   out_8072824320030371515[4] = 0;
   out_8072824320030371515[5] = 0;
   out_8072824320030371515[6] = 1;
   out_8072824320030371515[7] = 0;
   out_8072824320030371515[8] = 0;
   out_8072824320030371515[9] = 1;
   out_8072824320030371515[10] = 0;
   out_8072824320030371515[11] = 0;
   out_8072824320030371515[12] = 0;
   out_8072824320030371515[13] = 0;
   out_8072824320030371515[14] = 0;
   out_8072824320030371515[15] = 0;
   out_8072824320030371515[16] = 0;
   out_8072824320030371515[17] = 0;
   out_8072824320030371515[18] = 0;
   out_8072824320030371515[19] = 0;
   out_8072824320030371515[20] = 0;
   out_8072824320030371515[21] = 0;
   out_8072824320030371515[22] = 0;
   out_8072824320030371515[23] = 0;
   out_8072824320030371515[24] = 0;
   out_8072824320030371515[25] = 1;
   out_8072824320030371515[26] = 0;
   out_8072824320030371515[27] = 0;
   out_8072824320030371515[28] = 1;
   out_8072824320030371515[29] = 0;
   out_8072824320030371515[30] = 0;
   out_8072824320030371515[31] = 0;
   out_8072824320030371515[32] = 0;
   out_8072824320030371515[33] = 0;
   out_8072824320030371515[34] = 0;
   out_8072824320030371515[35] = 0;
   out_8072824320030371515[36] = 0;
   out_8072824320030371515[37] = 0;
   out_8072824320030371515[38] = 0;
   out_8072824320030371515[39] = 0;
   out_8072824320030371515[40] = 0;
   out_8072824320030371515[41] = 0;
   out_8072824320030371515[42] = 0;
   out_8072824320030371515[43] = 0;
   out_8072824320030371515[44] = 1;
   out_8072824320030371515[45] = 0;
   out_8072824320030371515[46] = 0;
   out_8072824320030371515[47] = 1;
   out_8072824320030371515[48] = 0;
   out_8072824320030371515[49] = 0;
   out_8072824320030371515[50] = 0;
   out_8072824320030371515[51] = 0;
   out_8072824320030371515[52] = 0;
   out_8072824320030371515[53] = 0;
}
void h_10(double *state, double *unused, double *out_3625759173810396060) {
   out_3625759173810396060[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3625759173810396060[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3625759173810396060[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4958904843940480188) {
   out_4958904843940480188[0] = 0;
   out_4958904843940480188[1] = 9.8100000000000005*cos(state[1]);
   out_4958904843940480188[2] = 0;
   out_4958904843940480188[3] = 0;
   out_4958904843940480188[4] = -state[8];
   out_4958904843940480188[5] = state[7];
   out_4958904843940480188[6] = 0;
   out_4958904843940480188[7] = state[5];
   out_4958904843940480188[8] = -state[4];
   out_4958904843940480188[9] = 0;
   out_4958904843940480188[10] = 0;
   out_4958904843940480188[11] = 0;
   out_4958904843940480188[12] = 1;
   out_4958904843940480188[13] = 0;
   out_4958904843940480188[14] = 0;
   out_4958904843940480188[15] = 1;
   out_4958904843940480188[16] = 0;
   out_4958904843940480188[17] = 0;
   out_4958904843940480188[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4958904843940480188[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4958904843940480188[20] = 0;
   out_4958904843940480188[21] = state[8];
   out_4958904843940480188[22] = 0;
   out_4958904843940480188[23] = -state[6];
   out_4958904843940480188[24] = -state[5];
   out_4958904843940480188[25] = 0;
   out_4958904843940480188[26] = state[3];
   out_4958904843940480188[27] = 0;
   out_4958904843940480188[28] = 0;
   out_4958904843940480188[29] = 0;
   out_4958904843940480188[30] = 0;
   out_4958904843940480188[31] = 1;
   out_4958904843940480188[32] = 0;
   out_4958904843940480188[33] = 0;
   out_4958904843940480188[34] = 1;
   out_4958904843940480188[35] = 0;
   out_4958904843940480188[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4958904843940480188[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4958904843940480188[38] = 0;
   out_4958904843940480188[39] = -state[7];
   out_4958904843940480188[40] = state[6];
   out_4958904843940480188[41] = 0;
   out_4958904843940480188[42] = state[4];
   out_4958904843940480188[43] = -state[3];
   out_4958904843940480188[44] = 0;
   out_4958904843940480188[45] = 0;
   out_4958904843940480188[46] = 0;
   out_4958904843940480188[47] = 0;
   out_4958904843940480188[48] = 0;
   out_4958904843940480188[49] = 0;
   out_4958904843940480188[50] = 1;
   out_4958904843940480188[51] = 0;
   out_4958904843940480188[52] = 0;
   out_4958904843940480188[53] = 1;
}
void h_13(double *state, double *unused, double *out_2109651969206804491) {
   out_2109651969206804491[0] = state[3];
   out_2109651969206804491[1] = state[4];
   out_2109651969206804491[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2763288545362479172) {
   out_2763288545362479172[0] = 0;
   out_2763288545362479172[1] = 0;
   out_2763288545362479172[2] = 0;
   out_2763288545362479172[3] = 1;
   out_2763288545362479172[4] = 0;
   out_2763288545362479172[5] = 0;
   out_2763288545362479172[6] = 0;
   out_2763288545362479172[7] = 0;
   out_2763288545362479172[8] = 0;
   out_2763288545362479172[9] = 0;
   out_2763288545362479172[10] = 0;
   out_2763288545362479172[11] = 0;
   out_2763288545362479172[12] = 0;
   out_2763288545362479172[13] = 0;
   out_2763288545362479172[14] = 0;
   out_2763288545362479172[15] = 0;
   out_2763288545362479172[16] = 0;
   out_2763288545362479172[17] = 0;
   out_2763288545362479172[18] = 0;
   out_2763288545362479172[19] = 0;
   out_2763288545362479172[20] = 0;
   out_2763288545362479172[21] = 0;
   out_2763288545362479172[22] = 1;
   out_2763288545362479172[23] = 0;
   out_2763288545362479172[24] = 0;
   out_2763288545362479172[25] = 0;
   out_2763288545362479172[26] = 0;
   out_2763288545362479172[27] = 0;
   out_2763288545362479172[28] = 0;
   out_2763288545362479172[29] = 0;
   out_2763288545362479172[30] = 0;
   out_2763288545362479172[31] = 0;
   out_2763288545362479172[32] = 0;
   out_2763288545362479172[33] = 0;
   out_2763288545362479172[34] = 0;
   out_2763288545362479172[35] = 0;
   out_2763288545362479172[36] = 0;
   out_2763288545362479172[37] = 0;
   out_2763288545362479172[38] = 0;
   out_2763288545362479172[39] = 0;
   out_2763288545362479172[40] = 0;
   out_2763288545362479172[41] = 1;
   out_2763288545362479172[42] = 0;
   out_2763288545362479172[43] = 0;
   out_2763288545362479172[44] = 0;
   out_2763288545362479172[45] = 0;
   out_2763288545362479172[46] = 0;
   out_2763288545362479172[47] = 0;
   out_2763288545362479172[48] = 0;
   out_2763288545362479172[49] = 0;
   out_2763288545362479172[50] = 0;
   out_2763288545362479172[51] = 0;
   out_2763288545362479172[52] = 0;
   out_2763288545362479172[53] = 0;
}
void h_14(double *state, double *unused, double *out_2205317381569854105) {
   out_2205317381569854105[0] = state[6];
   out_2205317381569854105[1] = state[7];
   out_2205317381569854105[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4990035887734999219) {
   out_4990035887734999219[0] = 0;
   out_4990035887734999219[1] = 0;
   out_4990035887734999219[2] = 0;
   out_4990035887734999219[3] = 0;
   out_4990035887734999219[4] = 0;
   out_4990035887734999219[5] = 0;
   out_4990035887734999219[6] = 1;
   out_4990035887734999219[7] = 0;
   out_4990035887734999219[8] = 0;
   out_4990035887734999219[9] = 0;
   out_4990035887734999219[10] = 0;
   out_4990035887734999219[11] = 0;
   out_4990035887734999219[12] = 0;
   out_4990035887734999219[13] = 0;
   out_4990035887734999219[14] = 0;
   out_4990035887734999219[15] = 0;
   out_4990035887734999219[16] = 0;
   out_4990035887734999219[17] = 0;
   out_4990035887734999219[18] = 0;
   out_4990035887734999219[19] = 0;
   out_4990035887734999219[20] = 0;
   out_4990035887734999219[21] = 0;
   out_4990035887734999219[22] = 0;
   out_4990035887734999219[23] = 0;
   out_4990035887734999219[24] = 0;
   out_4990035887734999219[25] = 1;
   out_4990035887734999219[26] = 0;
   out_4990035887734999219[27] = 0;
   out_4990035887734999219[28] = 0;
   out_4990035887734999219[29] = 0;
   out_4990035887734999219[30] = 0;
   out_4990035887734999219[31] = 0;
   out_4990035887734999219[32] = 0;
   out_4990035887734999219[33] = 0;
   out_4990035887734999219[34] = 0;
   out_4990035887734999219[35] = 0;
   out_4990035887734999219[36] = 0;
   out_4990035887734999219[37] = 0;
   out_4990035887734999219[38] = 0;
   out_4990035887734999219[39] = 0;
   out_4990035887734999219[40] = 0;
   out_4990035887734999219[41] = 0;
   out_4990035887734999219[42] = 0;
   out_4990035887734999219[43] = 0;
   out_4990035887734999219[44] = 1;
   out_4990035887734999219[45] = 0;
   out_4990035887734999219[46] = 0;
   out_4990035887734999219[47] = 0;
   out_4990035887734999219[48] = 0;
   out_4990035887734999219[49] = 0;
   out_4990035887734999219[50] = 0;
   out_4990035887734999219[51] = 0;
   out_4990035887734999219[52] = 0;
   out_4990035887734999219[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8353524553736122963) {
  err_fun(nom_x, delta_x, out_8353524553736122963);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8983699723827009156) {
  inv_err_fun(nom_x, true_x, out_8983699723827009156);
}
void pose_H_mod_fun(double *state, double *out_2968799295363009369) {
  H_mod_fun(state, out_2968799295363009369);
}
void pose_f_fun(double *state, double dt, double *out_4017585182654342073) {
  f_fun(state,  dt, out_4017585182654342073);
}
void pose_F_fun(double *state, double dt, double *out_6528773045854873690) {
  F_fun(state,  dt, out_6528773045854873690);
}
void pose_h_4(double *state, double *unused, double *out_2292486705639675299) {
  h_4(state, unused, out_2292486705639675299);
}
void pose_H_4(double *state, double *unused, double *out_8072824320030371515) {
  H_4(state, unused, out_8072824320030371515);
}
void pose_h_10(double *state, double *unused, double *out_3625759173810396060) {
  h_10(state, unused, out_3625759173810396060);
}
void pose_H_10(double *state, double *unused, double *out_4958904843940480188) {
  H_10(state, unused, out_4958904843940480188);
}
void pose_h_13(double *state, double *unused, double *out_2109651969206804491) {
  h_13(state, unused, out_2109651969206804491);
}
void pose_H_13(double *state, double *unused, double *out_2763288545362479172) {
  H_13(state, unused, out_2763288545362479172);
}
void pose_h_14(double *state, double *unused, double *out_2205317381569854105) {
  h_14(state, unused, out_2205317381569854105);
}
void pose_H_14(double *state, double *unused, double *out_4990035887734999219) {
  H_14(state, unused, out_4990035887734999219);
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
