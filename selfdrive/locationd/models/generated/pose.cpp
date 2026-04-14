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
void err_fun(double *nom_x, double *delta_x, double *out_6197317699744068790) {
   out_6197317699744068790[0] = delta_x[0] + nom_x[0];
   out_6197317699744068790[1] = delta_x[1] + nom_x[1];
   out_6197317699744068790[2] = delta_x[2] + nom_x[2];
   out_6197317699744068790[3] = delta_x[3] + nom_x[3];
   out_6197317699744068790[4] = delta_x[4] + nom_x[4];
   out_6197317699744068790[5] = delta_x[5] + nom_x[5];
   out_6197317699744068790[6] = delta_x[6] + nom_x[6];
   out_6197317699744068790[7] = delta_x[7] + nom_x[7];
   out_6197317699744068790[8] = delta_x[8] + nom_x[8];
   out_6197317699744068790[9] = delta_x[9] + nom_x[9];
   out_6197317699744068790[10] = delta_x[10] + nom_x[10];
   out_6197317699744068790[11] = delta_x[11] + nom_x[11];
   out_6197317699744068790[12] = delta_x[12] + nom_x[12];
   out_6197317699744068790[13] = delta_x[13] + nom_x[13];
   out_6197317699744068790[14] = delta_x[14] + nom_x[14];
   out_6197317699744068790[15] = delta_x[15] + nom_x[15];
   out_6197317699744068790[16] = delta_x[16] + nom_x[16];
   out_6197317699744068790[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8495458674857675788) {
   out_8495458674857675788[0] = -nom_x[0] + true_x[0];
   out_8495458674857675788[1] = -nom_x[1] + true_x[1];
   out_8495458674857675788[2] = -nom_x[2] + true_x[2];
   out_8495458674857675788[3] = -nom_x[3] + true_x[3];
   out_8495458674857675788[4] = -nom_x[4] + true_x[4];
   out_8495458674857675788[5] = -nom_x[5] + true_x[5];
   out_8495458674857675788[6] = -nom_x[6] + true_x[6];
   out_8495458674857675788[7] = -nom_x[7] + true_x[7];
   out_8495458674857675788[8] = -nom_x[8] + true_x[8];
   out_8495458674857675788[9] = -nom_x[9] + true_x[9];
   out_8495458674857675788[10] = -nom_x[10] + true_x[10];
   out_8495458674857675788[11] = -nom_x[11] + true_x[11];
   out_8495458674857675788[12] = -nom_x[12] + true_x[12];
   out_8495458674857675788[13] = -nom_x[13] + true_x[13];
   out_8495458674857675788[14] = -nom_x[14] + true_x[14];
   out_8495458674857675788[15] = -nom_x[15] + true_x[15];
   out_8495458674857675788[16] = -nom_x[16] + true_x[16];
   out_8495458674857675788[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_7217558117937058871) {
   out_7217558117937058871[0] = 1.0;
   out_7217558117937058871[1] = 0.0;
   out_7217558117937058871[2] = 0.0;
   out_7217558117937058871[3] = 0.0;
   out_7217558117937058871[4] = 0.0;
   out_7217558117937058871[5] = 0.0;
   out_7217558117937058871[6] = 0.0;
   out_7217558117937058871[7] = 0.0;
   out_7217558117937058871[8] = 0.0;
   out_7217558117937058871[9] = 0.0;
   out_7217558117937058871[10] = 0.0;
   out_7217558117937058871[11] = 0.0;
   out_7217558117937058871[12] = 0.0;
   out_7217558117937058871[13] = 0.0;
   out_7217558117937058871[14] = 0.0;
   out_7217558117937058871[15] = 0.0;
   out_7217558117937058871[16] = 0.0;
   out_7217558117937058871[17] = 0.0;
   out_7217558117937058871[18] = 0.0;
   out_7217558117937058871[19] = 1.0;
   out_7217558117937058871[20] = 0.0;
   out_7217558117937058871[21] = 0.0;
   out_7217558117937058871[22] = 0.0;
   out_7217558117937058871[23] = 0.0;
   out_7217558117937058871[24] = 0.0;
   out_7217558117937058871[25] = 0.0;
   out_7217558117937058871[26] = 0.0;
   out_7217558117937058871[27] = 0.0;
   out_7217558117937058871[28] = 0.0;
   out_7217558117937058871[29] = 0.0;
   out_7217558117937058871[30] = 0.0;
   out_7217558117937058871[31] = 0.0;
   out_7217558117937058871[32] = 0.0;
   out_7217558117937058871[33] = 0.0;
   out_7217558117937058871[34] = 0.0;
   out_7217558117937058871[35] = 0.0;
   out_7217558117937058871[36] = 0.0;
   out_7217558117937058871[37] = 0.0;
   out_7217558117937058871[38] = 1.0;
   out_7217558117937058871[39] = 0.0;
   out_7217558117937058871[40] = 0.0;
   out_7217558117937058871[41] = 0.0;
   out_7217558117937058871[42] = 0.0;
   out_7217558117937058871[43] = 0.0;
   out_7217558117937058871[44] = 0.0;
   out_7217558117937058871[45] = 0.0;
   out_7217558117937058871[46] = 0.0;
   out_7217558117937058871[47] = 0.0;
   out_7217558117937058871[48] = 0.0;
   out_7217558117937058871[49] = 0.0;
   out_7217558117937058871[50] = 0.0;
   out_7217558117937058871[51] = 0.0;
   out_7217558117937058871[52] = 0.0;
   out_7217558117937058871[53] = 0.0;
   out_7217558117937058871[54] = 0.0;
   out_7217558117937058871[55] = 0.0;
   out_7217558117937058871[56] = 0.0;
   out_7217558117937058871[57] = 1.0;
   out_7217558117937058871[58] = 0.0;
   out_7217558117937058871[59] = 0.0;
   out_7217558117937058871[60] = 0.0;
   out_7217558117937058871[61] = 0.0;
   out_7217558117937058871[62] = 0.0;
   out_7217558117937058871[63] = 0.0;
   out_7217558117937058871[64] = 0.0;
   out_7217558117937058871[65] = 0.0;
   out_7217558117937058871[66] = 0.0;
   out_7217558117937058871[67] = 0.0;
   out_7217558117937058871[68] = 0.0;
   out_7217558117937058871[69] = 0.0;
   out_7217558117937058871[70] = 0.0;
   out_7217558117937058871[71] = 0.0;
   out_7217558117937058871[72] = 0.0;
   out_7217558117937058871[73] = 0.0;
   out_7217558117937058871[74] = 0.0;
   out_7217558117937058871[75] = 0.0;
   out_7217558117937058871[76] = 1.0;
   out_7217558117937058871[77] = 0.0;
   out_7217558117937058871[78] = 0.0;
   out_7217558117937058871[79] = 0.0;
   out_7217558117937058871[80] = 0.0;
   out_7217558117937058871[81] = 0.0;
   out_7217558117937058871[82] = 0.0;
   out_7217558117937058871[83] = 0.0;
   out_7217558117937058871[84] = 0.0;
   out_7217558117937058871[85] = 0.0;
   out_7217558117937058871[86] = 0.0;
   out_7217558117937058871[87] = 0.0;
   out_7217558117937058871[88] = 0.0;
   out_7217558117937058871[89] = 0.0;
   out_7217558117937058871[90] = 0.0;
   out_7217558117937058871[91] = 0.0;
   out_7217558117937058871[92] = 0.0;
   out_7217558117937058871[93] = 0.0;
   out_7217558117937058871[94] = 0.0;
   out_7217558117937058871[95] = 1.0;
   out_7217558117937058871[96] = 0.0;
   out_7217558117937058871[97] = 0.0;
   out_7217558117937058871[98] = 0.0;
   out_7217558117937058871[99] = 0.0;
   out_7217558117937058871[100] = 0.0;
   out_7217558117937058871[101] = 0.0;
   out_7217558117937058871[102] = 0.0;
   out_7217558117937058871[103] = 0.0;
   out_7217558117937058871[104] = 0.0;
   out_7217558117937058871[105] = 0.0;
   out_7217558117937058871[106] = 0.0;
   out_7217558117937058871[107] = 0.0;
   out_7217558117937058871[108] = 0.0;
   out_7217558117937058871[109] = 0.0;
   out_7217558117937058871[110] = 0.0;
   out_7217558117937058871[111] = 0.0;
   out_7217558117937058871[112] = 0.0;
   out_7217558117937058871[113] = 0.0;
   out_7217558117937058871[114] = 1.0;
   out_7217558117937058871[115] = 0.0;
   out_7217558117937058871[116] = 0.0;
   out_7217558117937058871[117] = 0.0;
   out_7217558117937058871[118] = 0.0;
   out_7217558117937058871[119] = 0.0;
   out_7217558117937058871[120] = 0.0;
   out_7217558117937058871[121] = 0.0;
   out_7217558117937058871[122] = 0.0;
   out_7217558117937058871[123] = 0.0;
   out_7217558117937058871[124] = 0.0;
   out_7217558117937058871[125] = 0.0;
   out_7217558117937058871[126] = 0.0;
   out_7217558117937058871[127] = 0.0;
   out_7217558117937058871[128] = 0.0;
   out_7217558117937058871[129] = 0.0;
   out_7217558117937058871[130] = 0.0;
   out_7217558117937058871[131] = 0.0;
   out_7217558117937058871[132] = 0.0;
   out_7217558117937058871[133] = 1.0;
   out_7217558117937058871[134] = 0.0;
   out_7217558117937058871[135] = 0.0;
   out_7217558117937058871[136] = 0.0;
   out_7217558117937058871[137] = 0.0;
   out_7217558117937058871[138] = 0.0;
   out_7217558117937058871[139] = 0.0;
   out_7217558117937058871[140] = 0.0;
   out_7217558117937058871[141] = 0.0;
   out_7217558117937058871[142] = 0.0;
   out_7217558117937058871[143] = 0.0;
   out_7217558117937058871[144] = 0.0;
   out_7217558117937058871[145] = 0.0;
   out_7217558117937058871[146] = 0.0;
   out_7217558117937058871[147] = 0.0;
   out_7217558117937058871[148] = 0.0;
   out_7217558117937058871[149] = 0.0;
   out_7217558117937058871[150] = 0.0;
   out_7217558117937058871[151] = 0.0;
   out_7217558117937058871[152] = 1.0;
   out_7217558117937058871[153] = 0.0;
   out_7217558117937058871[154] = 0.0;
   out_7217558117937058871[155] = 0.0;
   out_7217558117937058871[156] = 0.0;
   out_7217558117937058871[157] = 0.0;
   out_7217558117937058871[158] = 0.0;
   out_7217558117937058871[159] = 0.0;
   out_7217558117937058871[160] = 0.0;
   out_7217558117937058871[161] = 0.0;
   out_7217558117937058871[162] = 0.0;
   out_7217558117937058871[163] = 0.0;
   out_7217558117937058871[164] = 0.0;
   out_7217558117937058871[165] = 0.0;
   out_7217558117937058871[166] = 0.0;
   out_7217558117937058871[167] = 0.0;
   out_7217558117937058871[168] = 0.0;
   out_7217558117937058871[169] = 0.0;
   out_7217558117937058871[170] = 0.0;
   out_7217558117937058871[171] = 1.0;
   out_7217558117937058871[172] = 0.0;
   out_7217558117937058871[173] = 0.0;
   out_7217558117937058871[174] = 0.0;
   out_7217558117937058871[175] = 0.0;
   out_7217558117937058871[176] = 0.0;
   out_7217558117937058871[177] = 0.0;
   out_7217558117937058871[178] = 0.0;
   out_7217558117937058871[179] = 0.0;
   out_7217558117937058871[180] = 0.0;
   out_7217558117937058871[181] = 0.0;
   out_7217558117937058871[182] = 0.0;
   out_7217558117937058871[183] = 0.0;
   out_7217558117937058871[184] = 0.0;
   out_7217558117937058871[185] = 0.0;
   out_7217558117937058871[186] = 0.0;
   out_7217558117937058871[187] = 0.0;
   out_7217558117937058871[188] = 0.0;
   out_7217558117937058871[189] = 0.0;
   out_7217558117937058871[190] = 1.0;
   out_7217558117937058871[191] = 0.0;
   out_7217558117937058871[192] = 0.0;
   out_7217558117937058871[193] = 0.0;
   out_7217558117937058871[194] = 0.0;
   out_7217558117937058871[195] = 0.0;
   out_7217558117937058871[196] = 0.0;
   out_7217558117937058871[197] = 0.0;
   out_7217558117937058871[198] = 0.0;
   out_7217558117937058871[199] = 0.0;
   out_7217558117937058871[200] = 0.0;
   out_7217558117937058871[201] = 0.0;
   out_7217558117937058871[202] = 0.0;
   out_7217558117937058871[203] = 0.0;
   out_7217558117937058871[204] = 0.0;
   out_7217558117937058871[205] = 0.0;
   out_7217558117937058871[206] = 0.0;
   out_7217558117937058871[207] = 0.0;
   out_7217558117937058871[208] = 0.0;
   out_7217558117937058871[209] = 1.0;
   out_7217558117937058871[210] = 0.0;
   out_7217558117937058871[211] = 0.0;
   out_7217558117937058871[212] = 0.0;
   out_7217558117937058871[213] = 0.0;
   out_7217558117937058871[214] = 0.0;
   out_7217558117937058871[215] = 0.0;
   out_7217558117937058871[216] = 0.0;
   out_7217558117937058871[217] = 0.0;
   out_7217558117937058871[218] = 0.0;
   out_7217558117937058871[219] = 0.0;
   out_7217558117937058871[220] = 0.0;
   out_7217558117937058871[221] = 0.0;
   out_7217558117937058871[222] = 0.0;
   out_7217558117937058871[223] = 0.0;
   out_7217558117937058871[224] = 0.0;
   out_7217558117937058871[225] = 0.0;
   out_7217558117937058871[226] = 0.0;
   out_7217558117937058871[227] = 0.0;
   out_7217558117937058871[228] = 1.0;
   out_7217558117937058871[229] = 0.0;
   out_7217558117937058871[230] = 0.0;
   out_7217558117937058871[231] = 0.0;
   out_7217558117937058871[232] = 0.0;
   out_7217558117937058871[233] = 0.0;
   out_7217558117937058871[234] = 0.0;
   out_7217558117937058871[235] = 0.0;
   out_7217558117937058871[236] = 0.0;
   out_7217558117937058871[237] = 0.0;
   out_7217558117937058871[238] = 0.0;
   out_7217558117937058871[239] = 0.0;
   out_7217558117937058871[240] = 0.0;
   out_7217558117937058871[241] = 0.0;
   out_7217558117937058871[242] = 0.0;
   out_7217558117937058871[243] = 0.0;
   out_7217558117937058871[244] = 0.0;
   out_7217558117937058871[245] = 0.0;
   out_7217558117937058871[246] = 0.0;
   out_7217558117937058871[247] = 1.0;
   out_7217558117937058871[248] = 0.0;
   out_7217558117937058871[249] = 0.0;
   out_7217558117937058871[250] = 0.0;
   out_7217558117937058871[251] = 0.0;
   out_7217558117937058871[252] = 0.0;
   out_7217558117937058871[253] = 0.0;
   out_7217558117937058871[254] = 0.0;
   out_7217558117937058871[255] = 0.0;
   out_7217558117937058871[256] = 0.0;
   out_7217558117937058871[257] = 0.0;
   out_7217558117937058871[258] = 0.0;
   out_7217558117937058871[259] = 0.0;
   out_7217558117937058871[260] = 0.0;
   out_7217558117937058871[261] = 0.0;
   out_7217558117937058871[262] = 0.0;
   out_7217558117937058871[263] = 0.0;
   out_7217558117937058871[264] = 0.0;
   out_7217558117937058871[265] = 0.0;
   out_7217558117937058871[266] = 1.0;
   out_7217558117937058871[267] = 0.0;
   out_7217558117937058871[268] = 0.0;
   out_7217558117937058871[269] = 0.0;
   out_7217558117937058871[270] = 0.0;
   out_7217558117937058871[271] = 0.0;
   out_7217558117937058871[272] = 0.0;
   out_7217558117937058871[273] = 0.0;
   out_7217558117937058871[274] = 0.0;
   out_7217558117937058871[275] = 0.0;
   out_7217558117937058871[276] = 0.0;
   out_7217558117937058871[277] = 0.0;
   out_7217558117937058871[278] = 0.0;
   out_7217558117937058871[279] = 0.0;
   out_7217558117937058871[280] = 0.0;
   out_7217558117937058871[281] = 0.0;
   out_7217558117937058871[282] = 0.0;
   out_7217558117937058871[283] = 0.0;
   out_7217558117937058871[284] = 0.0;
   out_7217558117937058871[285] = 1.0;
   out_7217558117937058871[286] = 0.0;
   out_7217558117937058871[287] = 0.0;
   out_7217558117937058871[288] = 0.0;
   out_7217558117937058871[289] = 0.0;
   out_7217558117937058871[290] = 0.0;
   out_7217558117937058871[291] = 0.0;
   out_7217558117937058871[292] = 0.0;
   out_7217558117937058871[293] = 0.0;
   out_7217558117937058871[294] = 0.0;
   out_7217558117937058871[295] = 0.0;
   out_7217558117937058871[296] = 0.0;
   out_7217558117937058871[297] = 0.0;
   out_7217558117937058871[298] = 0.0;
   out_7217558117937058871[299] = 0.0;
   out_7217558117937058871[300] = 0.0;
   out_7217558117937058871[301] = 0.0;
   out_7217558117937058871[302] = 0.0;
   out_7217558117937058871[303] = 0.0;
   out_7217558117937058871[304] = 1.0;
   out_7217558117937058871[305] = 0.0;
   out_7217558117937058871[306] = 0.0;
   out_7217558117937058871[307] = 0.0;
   out_7217558117937058871[308] = 0.0;
   out_7217558117937058871[309] = 0.0;
   out_7217558117937058871[310] = 0.0;
   out_7217558117937058871[311] = 0.0;
   out_7217558117937058871[312] = 0.0;
   out_7217558117937058871[313] = 0.0;
   out_7217558117937058871[314] = 0.0;
   out_7217558117937058871[315] = 0.0;
   out_7217558117937058871[316] = 0.0;
   out_7217558117937058871[317] = 0.0;
   out_7217558117937058871[318] = 0.0;
   out_7217558117937058871[319] = 0.0;
   out_7217558117937058871[320] = 0.0;
   out_7217558117937058871[321] = 0.0;
   out_7217558117937058871[322] = 0.0;
   out_7217558117937058871[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_9009937482874928922) {
   out_9009937482874928922[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_9009937482874928922[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_9009937482874928922[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_9009937482874928922[3] = dt*state[12] + state[3];
   out_9009937482874928922[4] = dt*state[13] + state[4];
   out_9009937482874928922[5] = dt*state[14] + state[5];
   out_9009937482874928922[6] = state[6];
   out_9009937482874928922[7] = state[7];
   out_9009937482874928922[8] = state[8];
   out_9009937482874928922[9] = state[9];
   out_9009937482874928922[10] = state[10];
   out_9009937482874928922[11] = state[11];
   out_9009937482874928922[12] = state[12];
   out_9009937482874928922[13] = state[13];
   out_9009937482874928922[14] = state[14];
   out_9009937482874928922[15] = state[15];
   out_9009937482874928922[16] = state[16];
   out_9009937482874928922[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5828535467002130009) {
   out_5828535467002130009[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5828535467002130009[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5828535467002130009[2] = 0;
   out_5828535467002130009[3] = 0;
   out_5828535467002130009[4] = 0;
   out_5828535467002130009[5] = 0;
   out_5828535467002130009[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5828535467002130009[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5828535467002130009[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5828535467002130009[9] = 0;
   out_5828535467002130009[10] = 0;
   out_5828535467002130009[11] = 0;
   out_5828535467002130009[12] = 0;
   out_5828535467002130009[13] = 0;
   out_5828535467002130009[14] = 0;
   out_5828535467002130009[15] = 0;
   out_5828535467002130009[16] = 0;
   out_5828535467002130009[17] = 0;
   out_5828535467002130009[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5828535467002130009[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5828535467002130009[20] = 0;
   out_5828535467002130009[21] = 0;
   out_5828535467002130009[22] = 0;
   out_5828535467002130009[23] = 0;
   out_5828535467002130009[24] = 0;
   out_5828535467002130009[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5828535467002130009[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5828535467002130009[27] = 0;
   out_5828535467002130009[28] = 0;
   out_5828535467002130009[29] = 0;
   out_5828535467002130009[30] = 0;
   out_5828535467002130009[31] = 0;
   out_5828535467002130009[32] = 0;
   out_5828535467002130009[33] = 0;
   out_5828535467002130009[34] = 0;
   out_5828535467002130009[35] = 0;
   out_5828535467002130009[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5828535467002130009[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5828535467002130009[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5828535467002130009[39] = 0;
   out_5828535467002130009[40] = 0;
   out_5828535467002130009[41] = 0;
   out_5828535467002130009[42] = 0;
   out_5828535467002130009[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5828535467002130009[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5828535467002130009[45] = 0;
   out_5828535467002130009[46] = 0;
   out_5828535467002130009[47] = 0;
   out_5828535467002130009[48] = 0;
   out_5828535467002130009[49] = 0;
   out_5828535467002130009[50] = 0;
   out_5828535467002130009[51] = 0;
   out_5828535467002130009[52] = 0;
   out_5828535467002130009[53] = 0;
   out_5828535467002130009[54] = 0;
   out_5828535467002130009[55] = 0;
   out_5828535467002130009[56] = 0;
   out_5828535467002130009[57] = 1;
   out_5828535467002130009[58] = 0;
   out_5828535467002130009[59] = 0;
   out_5828535467002130009[60] = 0;
   out_5828535467002130009[61] = 0;
   out_5828535467002130009[62] = 0;
   out_5828535467002130009[63] = 0;
   out_5828535467002130009[64] = 0;
   out_5828535467002130009[65] = 0;
   out_5828535467002130009[66] = dt;
   out_5828535467002130009[67] = 0;
   out_5828535467002130009[68] = 0;
   out_5828535467002130009[69] = 0;
   out_5828535467002130009[70] = 0;
   out_5828535467002130009[71] = 0;
   out_5828535467002130009[72] = 0;
   out_5828535467002130009[73] = 0;
   out_5828535467002130009[74] = 0;
   out_5828535467002130009[75] = 0;
   out_5828535467002130009[76] = 1;
   out_5828535467002130009[77] = 0;
   out_5828535467002130009[78] = 0;
   out_5828535467002130009[79] = 0;
   out_5828535467002130009[80] = 0;
   out_5828535467002130009[81] = 0;
   out_5828535467002130009[82] = 0;
   out_5828535467002130009[83] = 0;
   out_5828535467002130009[84] = 0;
   out_5828535467002130009[85] = dt;
   out_5828535467002130009[86] = 0;
   out_5828535467002130009[87] = 0;
   out_5828535467002130009[88] = 0;
   out_5828535467002130009[89] = 0;
   out_5828535467002130009[90] = 0;
   out_5828535467002130009[91] = 0;
   out_5828535467002130009[92] = 0;
   out_5828535467002130009[93] = 0;
   out_5828535467002130009[94] = 0;
   out_5828535467002130009[95] = 1;
   out_5828535467002130009[96] = 0;
   out_5828535467002130009[97] = 0;
   out_5828535467002130009[98] = 0;
   out_5828535467002130009[99] = 0;
   out_5828535467002130009[100] = 0;
   out_5828535467002130009[101] = 0;
   out_5828535467002130009[102] = 0;
   out_5828535467002130009[103] = 0;
   out_5828535467002130009[104] = dt;
   out_5828535467002130009[105] = 0;
   out_5828535467002130009[106] = 0;
   out_5828535467002130009[107] = 0;
   out_5828535467002130009[108] = 0;
   out_5828535467002130009[109] = 0;
   out_5828535467002130009[110] = 0;
   out_5828535467002130009[111] = 0;
   out_5828535467002130009[112] = 0;
   out_5828535467002130009[113] = 0;
   out_5828535467002130009[114] = 1;
   out_5828535467002130009[115] = 0;
   out_5828535467002130009[116] = 0;
   out_5828535467002130009[117] = 0;
   out_5828535467002130009[118] = 0;
   out_5828535467002130009[119] = 0;
   out_5828535467002130009[120] = 0;
   out_5828535467002130009[121] = 0;
   out_5828535467002130009[122] = 0;
   out_5828535467002130009[123] = 0;
   out_5828535467002130009[124] = 0;
   out_5828535467002130009[125] = 0;
   out_5828535467002130009[126] = 0;
   out_5828535467002130009[127] = 0;
   out_5828535467002130009[128] = 0;
   out_5828535467002130009[129] = 0;
   out_5828535467002130009[130] = 0;
   out_5828535467002130009[131] = 0;
   out_5828535467002130009[132] = 0;
   out_5828535467002130009[133] = 1;
   out_5828535467002130009[134] = 0;
   out_5828535467002130009[135] = 0;
   out_5828535467002130009[136] = 0;
   out_5828535467002130009[137] = 0;
   out_5828535467002130009[138] = 0;
   out_5828535467002130009[139] = 0;
   out_5828535467002130009[140] = 0;
   out_5828535467002130009[141] = 0;
   out_5828535467002130009[142] = 0;
   out_5828535467002130009[143] = 0;
   out_5828535467002130009[144] = 0;
   out_5828535467002130009[145] = 0;
   out_5828535467002130009[146] = 0;
   out_5828535467002130009[147] = 0;
   out_5828535467002130009[148] = 0;
   out_5828535467002130009[149] = 0;
   out_5828535467002130009[150] = 0;
   out_5828535467002130009[151] = 0;
   out_5828535467002130009[152] = 1;
   out_5828535467002130009[153] = 0;
   out_5828535467002130009[154] = 0;
   out_5828535467002130009[155] = 0;
   out_5828535467002130009[156] = 0;
   out_5828535467002130009[157] = 0;
   out_5828535467002130009[158] = 0;
   out_5828535467002130009[159] = 0;
   out_5828535467002130009[160] = 0;
   out_5828535467002130009[161] = 0;
   out_5828535467002130009[162] = 0;
   out_5828535467002130009[163] = 0;
   out_5828535467002130009[164] = 0;
   out_5828535467002130009[165] = 0;
   out_5828535467002130009[166] = 0;
   out_5828535467002130009[167] = 0;
   out_5828535467002130009[168] = 0;
   out_5828535467002130009[169] = 0;
   out_5828535467002130009[170] = 0;
   out_5828535467002130009[171] = 1;
   out_5828535467002130009[172] = 0;
   out_5828535467002130009[173] = 0;
   out_5828535467002130009[174] = 0;
   out_5828535467002130009[175] = 0;
   out_5828535467002130009[176] = 0;
   out_5828535467002130009[177] = 0;
   out_5828535467002130009[178] = 0;
   out_5828535467002130009[179] = 0;
   out_5828535467002130009[180] = 0;
   out_5828535467002130009[181] = 0;
   out_5828535467002130009[182] = 0;
   out_5828535467002130009[183] = 0;
   out_5828535467002130009[184] = 0;
   out_5828535467002130009[185] = 0;
   out_5828535467002130009[186] = 0;
   out_5828535467002130009[187] = 0;
   out_5828535467002130009[188] = 0;
   out_5828535467002130009[189] = 0;
   out_5828535467002130009[190] = 1;
   out_5828535467002130009[191] = 0;
   out_5828535467002130009[192] = 0;
   out_5828535467002130009[193] = 0;
   out_5828535467002130009[194] = 0;
   out_5828535467002130009[195] = 0;
   out_5828535467002130009[196] = 0;
   out_5828535467002130009[197] = 0;
   out_5828535467002130009[198] = 0;
   out_5828535467002130009[199] = 0;
   out_5828535467002130009[200] = 0;
   out_5828535467002130009[201] = 0;
   out_5828535467002130009[202] = 0;
   out_5828535467002130009[203] = 0;
   out_5828535467002130009[204] = 0;
   out_5828535467002130009[205] = 0;
   out_5828535467002130009[206] = 0;
   out_5828535467002130009[207] = 0;
   out_5828535467002130009[208] = 0;
   out_5828535467002130009[209] = 1;
   out_5828535467002130009[210] = 0;
   out_5828535467002130009[211] = 0;
   out_5828535467002130009[212] = 0;
   out_5828535467002130009[213] = 0;
   out_5828535467002130009[214] = 0;
   out_5828535467002130009[215] = 0;
   out_5828535467002130009[216] = 0;
   out_5828535467002130009[217] = 0;
   out_5828535467002130009[218] = 0;
   out_5828535467002130009[219] = 0;
   out_5828535467002130009[220] = 0;
   out_5828535467002130009[221] = 0;
   out_5828535467002130009[222] = 0;
   out_5828535467002130009[223] = 0;
   out_5828535467002130009[224] = 0;
   out_5828535467002130009[225] = 0;
   out_5828535467002130009[226] = 0;
   out_5828535467002130009[227] = 0;
   out_5828535467002130009[228] = 1;
   out_5828535467002130009[229] = 0;
   out_5828535467002130009[230] = 0;
   out_5828535467002130009[231] = 0;
   out_5828535467002130009[232] = 0;
   out_5828535467002130009[233] = 0;
   out_5828535467002130009[234] = 0;
   out_5828535467002130009[235] = 0;
   out_5828535467002130009[236] = 0;
   out_5828535467002130009[237] = 0;
   out_5828535467002130009[238] = 0;
   out_5828535467002130009[239] = 0;
   out_5828535467002130009[240] = 0;
   out_5828535467002130009[241] = 0;
   out_5828535467002130009[242] = 0;
   out_5828535467002130009[243] = 0;
   out_5828535467002130009[244] = 0;
   out_5828535467002130009[245] = 0;
   out_5828535467002130009[246] = 0;
   out_5828535467002130009[247] = 1;
   out_5828535467002130009[248] = 0;
   out_5828535467002130009[249] = 0;
   out_5828535467002130009[250] = 0;
   out_5828535467002130009[251] = 0;
   out_5828535467002130009[252] = 0;
   out_5828535467002130009[253] = 0;
   out_5828535467002130009[254] = 0;
   out_5828535467002130009[255] = 0;
   out_5828535467002130009[256] = 0;
   out_5828535467002130009[257] = 0;
   out_5828535467002130009[258] = 0;
   out_5828535467002130009[259] = 0;
   out_5828535467002130009[260] = 0;
   out_5828535467002130009[261] = 0;
   out_5828535467002130009[262] = 0;
   out_5828535467002130009[263] = 0;
   out_5828535467002130009[264] = 0;
   out_5828535467002130009[265] = 0;
   out_5828535467002130009[266] = 1;
   out_5828535467002130009[267] = 0;
   out_5828535467002130009[268] = 0;
   out_5828535467002130009[269] = 0;
   out_5828535467002130009[270] = 0;
   out_5828535467002130009[271] = 0;
   out_5828535467002130009[272] = 0;
   out_5828535467002130009[273] = 0;
   out_5828535467002130009[274] = 0;
   out_5828535467002130009[275] = 0;
   out_5828535467002130009[276] = 0;
   out_5828535467002130009[277] = 0;
   out_5828535467002130009[278] = 0;
   out_5828535467002130009[279] = 0;
   out_5828535467002130009[280] = 0;
   out_5828535467002130009[281] = 0;
   out_5828535467002130009[282] = 0;
   out_5828535467002130009[283] = 0;
   out_5828535467002130009[284] = 0;
   out_5828535467002130009[285] = 1;
   out_5828535467002130009[286] = 0;
   out_5828535467002130009[287] = 0;
   out_5828535467002130009[288] = 0;
   out_5828535467002130009[289] = 0;
   out_5828535467002130009[290] = 0;
   out_5828535467002130009[291] = 0;
   out_5828535467002130009[292] = 0;
   out_5828535467002130009[293] = 0;
   out_5828535467002130009[294] = 0;
   out_5828535467002130009[295] = 0;
   out_5828535467002130009[296] = 0;
   out_5828535467002130009[297] = 0;
   out_5828535467002130009[298] = 0;
   out_5828535467002130009[299] = 0;
   out_5828535467002130009[300] = 0;
   out_5828535467002130009[301] = 0;
   out_5828535467002130009[302] = 0;
   out_5828535467002130009[303] = 0;
   out_5828535467002130009[304] = 1;
   out_5828535467002130009[305] = 0;
   out_5828535467002130009[306] = 0;
   out_5828535467002130009[307] = 0;
   out_5828535467002130009[308] = 0;
   out_5828535467002130009[309] = 0;
   out_5828535467002130009[310] = 0;
   out_5828535467002130009[311] = 0;
   out_5828535467002130009[312] = 0;
   out_5828535467002130009[313] = 0;
   out_5828535467002130009[314] = 0;
   out_5828535467002130009[315] = 0;
   out_5828535467002130009[316] = 0;
   out_5828535467002130009[317] = 0;
   out_5828535467002130009[318] = 0;
   out_5828535467002130009[319] = 0;
   out_5828535467002130009[320] = 0;
   out_5828535467002130009[321] = 0;
   out_5828535467002130009[322] = 0;
   out_5828535467002130009[323] = 1;
}
void h_4(double *state, double *unused, double *out_437538278502993915) {
   out_437538278502993915[0] = state[6] + state[9];
   out_437538278502993915[1] = state[7] + state[10];
   out_437538278502993915[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6614558918843176381) {
   out_6614558918843176381[0] = 0;
   out_6614558918843176381[1] = 0;
   out_6614558918843176381[2] = 0;
   out_6614558918843176381[3] = 0;
   out_6614558918843176381[4] = 0;
   out_6614558918843176381[5] = 0;
   out_6614558918843176381[6] = 1;
   out_6614558918843176381[7] = 0;
   out_6614558918843176381[8] = 0;
   out_6614558918843176381[9] = 1;
   out_6614558918843176381[10] = 0;
   out_6614558918843176381[11] = 0;
   out_6614558918843176381[12] = 0;
   out_6614558918843176381[13] = 0;
   out_6614558918843176381[14] = 0;
   out_6614558918843176381[15] = 0;
   out_6614558918843176381[16] = 0;
   out_6614558918843176381[17] = 0;
   out_6614558918843176381[18] = 0;
   out_6614558918843176381[19] = 0;
   out_6614558918843176381[20] = 0;
   out_6614558918843176381[21] = 0;
   out_6614558918843176381[22] = 0;
   out_6614558918843176381[23] = 0;
   out_6614558918843176381[24] = 0;
   out_6614558918843176381[25] = 1;
   out_6614558918843176381[26] = 0;
   out_6614558918843176381[27] = 0;
   out_6614558918843176381[28] = 1;
   out_6614558918843176381[29] = 0;
   out_6614558918843176381[30] = 0;
   out_6614558918843176381[31] = 0;
   out_6614558918843176381[32] = 0;
   out_6614558918843176381[33] = 0;
   out_6614558918843176381[34] = 0;
   out_6614558918843176381[35] = 0;
   out_6614558918843176381[36] = 0;
   out_6614558918843176381[37] = 0;
   out_6614558918843176381[38] = 0;
   out_6614558918843176381[39] = 0;
   out_6614558918843176381[40] = 0;
   out_6614558918843176381[41] = 0;
   out_6614558918843176381[42] = 0;
   out_6614558918843176381[43] = 0;
   out_6614558918843176381[44] = 1;
   out_6614558918843176381[45] = 0;
   out_6614558918843176381[46] = 0;
   out_6614558918843176381[47] = 1;
   out_6614558918843176381[48] = 0;
   out_6614558918843176381[49] = 0;
   out_6614558918843176381[50] = 0;
   out_6614558918843176381[51] = 0;
   out_6614558918843176381[52] = 0;
   out_6614558918843176381[53] = 0;
}
void h_10(double *state, double *unused, double *out_3185345221758858216) {
   out_3185345221758858216[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3185345221758858216[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3185345221758858216[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2487017710939471116) {
   out_2487017710939471116[0] = 0;
   out_2487017710939471116[1] = 9.8100000000000005*cos(state[1]);
   out_2487017710939471116[2] = 0;
   out_2487017710939471116[3] = 0;
   out_2487017710939471116[4] = -state[8];
   out_2487017710939471116[5] = state[7];
   out_2487017710939471116[6] = 0;
   out_2487017710939471116[7] = state[5];
   out_2487017710939471116[8] = -state[4];
   out_2487017710939471116[9] = 0;
   out_2487017710939471116[10] = 0;
   out_2487017710939471116[11] = 0;
   out_2487017710939471116[12] = 1;
   out_2487017710939471116[13] = 0;
   out_2487017710939471116[14] = 0;
   out_2487017710939471116[15] = 1;
   out_2487017710939471116[16] = 0;
   out_2487017710939471116[17] = 0;
   out_2487017710939471116[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2487017710939471116[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2487017710939471116[20] = 0;
   out_2487017710939471116[21] = state[8];
   out_2487017710939471116[22] = 0;
   out_2487017710939471116[23] = -state[6];
   out_2487017710939471116[24] = -state[5];
   out_2487017710939471116[25] = 0;
   out_2487017710939471116[26] = state[3];
   out_2487017710939471116[27] = 0;
   out_2487017710939471116[28] = 0;
   out_2487017710939471116[29] = 0;
   out_2487017710939471116[30] = 0;
   out_2487017710939471116[31] = 1;
   out_2487017710939471116[32] = 0;
   out_2487017710939471116[33] = 0;
   out_2487017710939471116[34] = 1;
   out_2487017710939471116[35] = 0;
   out_2487017710939471116[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2487017710939471116[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2487017710939471116[38] = 0;
   out_2487017710939471116[39] = -state[7];
   out_2487017710939471116[40] = state[6];
   out_2487017710939471116[41] = 0;
   out_2487017710939471116[42] = state[4];
   out_2487017710939471116[43] = -state[3];
   out_2487017710939471116[44] = 0;
   out_2487017710939471116[45] = 0;
   out_2487017710939471116[46] = 0;
   out_2487017710939471116[47] = 0;
   out_2487017710939471116[48] = 0;
   out_2487017710939471116[49] = 0;
   out_2487017710939471116[50] = 1;
   out_2487017710939471116[51] = 0;
   out_2487017710939471116[52] = 0;
   out_2487017710939471116[53] = 1;
}
void h_13(double *state, double *unused, double *out_8589966826250431028) {
   out_8589966826250431028[0] = state[3];
   out_8589966826250431028[1] = state[4];
   out_8589966826250431028[2] = state[5];
}
void H_13(double *state, double *unused, double *out_996072289473524548) {
   out_996072289473524548[0] = 0;
   out_996072289473524548[1] = 0;
   out_996072289473524548[2] = 0;
   out_996072289473524548[3] = 1;
   out_996072289473524548[4] = 0;
   out_996072289473524548[5] = 0;
   out_996072289473524548[6] = 0;
   out_996072289473524548[7] = 0;
   out_996072289473524548[8] = 0;
   out_996072289473524548[9] = 0;
   out_996072289473524548[10] = 0;
   out_996072289473524548[11] = 0;
   out_996072289473524548[12] = 0;
   out_996072289473524548[13] = 0;
   out_996072289473524548[14] = 0;
   out_996072289473524548[15] = 0;
   out_996072289473524548[16] = 0;
   out_996072289473524548[17] = 0;
   out_996072289473524548[18] = 0;
   out_996072289473524548[19] = 0;
   out_996072289473524548[20] = 0;
   out_996072289473524548[21] = 0;
   out_996072289473524548[22] = 1;
   out_996072289473524548[23] = 0;
   out_996072289473524548[24] = 0;
   out_996072289473524548[25] = 0;
   out_996072289473524548[26] = 0;
   out_996072289473524548[27] = 0;
   out_996072289473524548[28] = 0;
   out_996072289473524548[29] = 0;
   out_996072289473524548[30] = 0;
   out_996072289473524548[31] = 0;
   out_996072289473524548[32] = 0;
   out_996072289473524548[33] = 0;
   out_996072289473524548[34] = 0;
   out_996072289473524548[35] = 0;
   out_996072289473524548[36] = 0;
   out_996072289473524548[37] = 0;
   out_996072289473524548[38] = 0;
   out_996072289473524548[39] = 0;
   out_996072289473524548[40] = 0;
   out_996072289473524548[41] = 1;
   out_996072289473524548[42] = 0;
   out_996072289473524548[43] = 0;
   out_996072289473524548[44] = 0;
   out_996072289473524548[45] = 0;
   out_996072289473524548[46] = 0;
   out_996072289473524548[47] = 0;
   out_996072289473524548[48] = 0;
   out_996072289473524548[49] = 0;
   out_996072289473524548[50] = 0;
   out_996072289473524548[51] = 0;
   out_996072289473524548[52] = 0;
   out_996072289473524548[53] = 0;
}
void h_14(double *state, double *unused, double *out_3652909299986341134) {
   out_3652909299986341134[0] = state[6];
   out_3652909299986341134[1] = state[7];
   out_3652909299986341134[2] = state[8];
}
void H_14(double *state, double *unused, double *out_2651318062503691852) {
   out_2651318062503691852[0] = 0;
   out_2651318062503691852[1] = 0;
   out_2651318062503691852[2] = 0;
   out_2651318062503691852[3] = 0;
   out_2651318062503691852[4] = 0;
   out_2651318062503691852[5] = 0;
   out_2651318062503691852[6] = 1;
   out_2651318062503691852[7] = 0;
   out_2651318062503691852[8] = 0;
   out_2651318062503691852[9] = 0;
   out_2651318062503691852[10] = 0;
   out_2651318062503691852[11] = 0;
   out_2651318062503691852[12] = 0;
   out_2651318062503691852[13] = 0;
   out_2651318062503691852[14] = 0;
   out_2651318062503691852[15] = 0;
   out_2651318062503691852[16] = 0;
   out_2651318062503691852[17] = 0;
   out_2651318062503691852[18] = 0;
   out_2651318062503691852[19] = 0;
   out_2651318062503691852[20] = 0;
   out_2651318062503691852[21] = 0;
   out_2651318062503691852[22] = 0;
   out_2651318062503691852[23] = 0;
   out_2651318062503691852[24] = 0;
   out_2651318062503691852[25] = 1;
   out_2651318062503691852[26] = 0;
   out_2651318062503691852[27] = 0;
   out_2651318062503691852[28] = 0;
   out_2651318062503691852[29] = 0;
   out_2651318062503691852[30] = 0;
   out_2651318062503691852[31] = 0;
   out_2651318062503691852[32] = 0;
   out_2651318062503691852[33] = 0;
   out_2651318062503691852[34] = 0;
   out_2651318062503691852[35] = 0;
   out_2651318062503691852[36] = 0;
   out_2651318062503691852[37] = 0;
   out_2651318062503691852[38] = 0;
   out_2651318062503691852[39] = 0;
   out_2651318062503691852[40] = 0;
   out_2651318062503691852[41] = 0;
   out_2651318062503691852[42] = 0;
   out_2651318062503691852[43] = 0;
   out_2651318062503691852[44] = 1;
   out_2651318062503691852[45] = 0;
   out_2651318062503691852[46] = 0;
   out_2651318062503691852[47] = 0;
   out_2651318062503691852[48] = 0;
   out_2651318062503691852[49] = 0;
   out_2651318062503691852[50] = 0;
   out_2651318062503691852[51] = 0;
   out_2651318062503691852[52] = 0;
   out_2651318062503691852[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6197317699744068790) {
  err_fun(nom_x, delta_x, out_6197317699744068790);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8495458674857675788) {
  inv_err_fun(nom_x, true_x, out_8495458674857675788);
}
void pose_H_mod_fun(double *state, double *out_7217558117937058871) {
  H_mod_fun(state, out_7217558117937058871);
}
void pose_f_fun(double *state, double dt, double *out_9009937482874928922) {
  f_fun(state,  dt, out_9009937482874928922);
}
void pose_F_fun(double *state, double dt, double *out_5828535467002130009) {
  F_fun(state,  dt, out_5828535467002130009);
}
void pose_h_4(double *state, double *unused, double *out_437538278502993915) {
  h_4(state, unused, out_437538278502993915);
}
void pose_H_4(double *state, double *unused, double *out_6614558918843176381) {
  H_4(state, unused, out_6614558918843176381);
}
void pose_h_10(double *state, double *unused, double *out_3185345221758858216) {
  h_10(state, unused, out_3185345221758858216);
}
void pose_H_10(double *state, double *unused, double *out_2487017710939471116) {
  H_10(state, unused, out_2487017710939471116);
}
void pose_h_13(double *state, double *unused, double *out_8589966826250431028) {
  h_13(state, unused, out_8589966826250431028);
}
void pose_H_13(double *state, double *unused, double *out_996072289473524548) {
  H_13(state, unused, out_996072289473524548);
}
void pose_h_14(double *state, double *unused, double *out_3652909299986341134) {
  h_14(state, unused, out_3652909299986341134);
}
void pose_H_14(double *state, double *unused, double *out_2651318062503691852) {
  H_14(state, unused, out_2651318062503691852);
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
