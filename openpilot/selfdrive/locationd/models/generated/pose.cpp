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
void err_fun(double *nom_x, double *delta_x, double *out_1038245486379722605) {
   out_1038245486379722605[0] = delta_x[0] + nom_x[0];
   out_1038245486379722605[1] = delta_x[1] + nom_x[1];
   out_1038245486379722605[2] = delta_x[2] + nom_x[2];
   out_1038245486379722605[3] = delta_x[3] + nom_x[3];
   out_1038245486379722605[4] = delta_x[4] + nom_x[4];
   out_1038245486379722605[5] = delta_x[5] + nom_x[5];
   out_1038245486379722605[6] = delta_x[6] + nom_x[6];
   out_1038245486379722605[7] = delta_x[7] + nom_x[7];
   out_1038245486379722605[8] = delta_x[8] + nom_x[8];
   out_1038245486379722605[9] = delta_x[9] + nom_x[9];
   out_1038245486379722605[10] = delta_x[10] + nom_x[10];
   out_1038245486379722605[11] = delta_x[11] + nom_x[11];
   out_1038245486379722605[12] = delta_x[12] + nom_x[12];
   out_1038245486379722605[13] = delta_x[13] + nom_x[13];
   out_1038245486379722605[14] = delta_x[14] + nom_x[14];
   out_1038245486379722605[15] = delta_x[15] + nom_x[15];
   out_1038245486379722605[16] = delta_x[16] + nom_x[16];
   out_1038245486379722605[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6881182548000141707) {
   out_6881182548000141707[0] = -nom_x[0] + true_x[0];
   out_6881182548000141707[1] = -nom_x[1] + true_x[1];
   out_6881182548000141707[2] = -nom_x[2] + true_x[2];
   out_6881182548000141707[3] = -nom_x[3] + true_x[3];
   out_6881182548000141707[4] = -nom_x[4] + true_x[4];
   out_6881182548000141707[5] = -nom_x[5] + true_x[5];
   out_6881182548000141707[6] = -nom_x[6] + true_x[6];
   out_6881182548000141707[7] = -nom_x[7] + true_x[7];
   out_6881182548000141707[8] = -nom_x[8] + true_x[8];
   out_6881182548000141707[9] = -nom_x[9] + true_x[9];
   out_6881182548000141707[10] = -nom_x[10] + true_x[10];
   out_6881182548000141707[11] = -nom_x[11] + true_x[11];
   out_6881182548000141707[12] = -nom_x[12] + true_x[12];
   out_6881182548000141707[13] = -nom_x[13] + true_x[13];
   out_6881182548000141707[14] = -nom_x[14] + true_x[14];
   out_6881182548000141707[15] = -nom_x[15] + true_x[15];
   out_6881182548000141707[16] = -nom_x[16] + true_x[16];
   out_6881182548000141707[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4548887689658881741) {
   out_4548887689658881741[0] = 1.0;
   out_4548887689658881741[1] = 0.0;
   out_4548887689658881741[2] = 0.0;
   out_4548887689658881741[3] = 0.0;
   out_4548887689658881741[4] = 0.0;
   out_4548887689658881741[5] = 0.0;
   out_4548887689658881741[6] = 0.0;
   out_4548887689658881741[7] = 0.0;
   out_4548887689658881741[8] = 0.0;
   out_4548887689658881741[9] = 0.0;
   out_4548887689658881741[10] = 0.0;
   out_4548887689658881741[11] = 0.0;
   out_4548887689658881741[12] = 0.0;
   out_4548887689658881741[13] = 0.0;
   out_4548887689658881741[14] = 0.0;
   out_4548887689658881741[15] = 0.0;
   out_4548887689658881741[16] = 0.0;
   out_4548887689658881741[17] = 0.0;
   out_4548887689658881741[18] = 0.0;
   out_4548887689658881741[19] = 1.0;
   out_4548887689658881741[20] = 0.0;
   out_4548887689658881741[21] = 0.0;
   out_4548887689658881741[22] = 0.0;
   out_4548887689658881741[23] = 0.0;
   out_4548887689658881741[24] = 0.0;
   out_4548887689658881741[25] = 0.0;
   out_4548887689658881741[26] = 0.0;
   out_4548887689658881741[27] = 0.0;
   out_4548887689658881741[28] = 0.0;
   out_4548887689658881741[29] = 0.0;
   out_4548887689658881741[30] = 0.0;
   out_4548887689658881741[31] = 0.0;
   out_4548887689658881741[32] = 0.0;
   out_4548887689658881741[33] = 0.0;
   out_4548887689658881741[34] = 0.0;
   out_4548887689658881741[35] = 0.0;
   out_4548887689658881741[36] = 0.0;
   out_4548887689658881741[37] = 0.0;
   out_4548887689658881741[38] = 1.0;
   out_4548887689658881741[39] = 0.0;
   out_4548887689658881741[40] = 0.0;
   out_4548887689658881741[41] = 0.0;
   out_4548887689658881741[42] = 0.0;
   out_4548887689658881741[43] = 0.0;
   out_4548887689658881741[44] = 0.0;
   out_4548887689658881741[45] = 0.0;
   out_4548887689658881741[46] = 0.0;
   out_4548887689658881741[47] = 0.0;
   out_4548887689658881741[48] = 0.0;
   out_4548887689658881741[49] = 0.0;
   out_4548887689658881741[50] = 0.0;
   out_4548887689658881741[51] = 0.0;
   out_4548887689658881741[52] = 0.0;
   out_4548887689658881741[53] = 0.0;
   out_4548887689658881741[54] = 0.0;
   out_4548887689658881741[55] = 0.0;
   out_4548887689658881741[56] = 0.0;
   out_4548887689658881741[57] = 1.0;
   out_4548887689658881741[58] = 0.0;
   out_4548887689658881741[59] = 0.0;
   out_4548887689658881741[60] = 0.0;
   out_4548887689658881741[61] = 0.0;
   out_4548887689658881741[62] = 0.0;
   out_4548887689658881741[63] = 0.0;
   out_4548887689658881741[64] = 0.0;
   out_4548887689658881741[65] = 0.0;
   out_4548887689658881741[66] = 0.0;
   out_4548887689658881741[67] = 0.0;
   out_4548887689658881741[68] = 0.0;
   out_4548887689658881741[69] = 0.0;
   out_4548887689658881741[70] = 0.0;
   out_4548887689658881741[71] = 0.0;
   out_4548887689658881741[72] = 0.0;
   out_4548887689658881741[73] = 0.0;
   out_4548887689658881741[74] = 0.0;
   out_4548887689658881741[75] = 0.0;
   out_4548887689658881741[76] = 1.0;
   out_4548887689658881741[77] = 0.0;
   out_4548887689658881741[78] = 0.0;
   out_4548887689658881741[79] = 0.0;
   out_4548887689658881741[80] = 0.0;
   out_4548887689658881741[81] = 0.0;
   out_4548887689658881741[82] = 0.0;
   out_4548887689658881741[83] = 0.0;
   out_4548887689658881741[84] = 0.0;
   out_4548887689658881741[85] = 0.0;
   out_4548887689658881741[86] = 0.0;
   out_4548887689658881741[87] = 0.0;
   out_4548887689658881741[88] = 0.0;
   out_4548887689658881741[89] = 0.0;
   out_4548887689658881741[90] = 0.0;
   out_4548887689658881741[91] = 0.0;
   out_4548887689658881741[92] = 0.0;
   out_4548887689658881741[93] = 0.0;
   out_4548887689658881741[94] = 0.0;
   out_4548887689658881741[95] = 1.0;
   out_4548887689658881741[96] = 0.0;
   out_4548887689658881741[97] = 0.0;
   out_4548887689658881741[98] = 0.0;
   out_4548887689658881741[99] = 0.0;
   out_4548887689658881741[100] = 0.0;
   out_4548887689658881741[101] = 0.0;
   out_4548887689658881741[102] = 0.0;
   out_4548887689658881741[103] = 0.0;
   out_4548887689658881741[104] = 0.0;
   out_4548887689658881741[105] = 0.0;
   out_4548887689658881741[106] = 0.0;
   out_4548887689658881741[107] = 0.0;
   out_4548887689658881741[108] = 0.0;
   out_4548887689658881741[109] = 0.0;
   out_4548887689658881741[110] = 0.0;
   out_4548887689658881741[111] = 0.0;
   out_4548887689658881741[112] = 0.0;
   out_4548887689658881741[113] = 0.0;
   out_4548887689658881741[114] = 1.0;
   out_4548887689658881741[115] = 0.0;
   out_4548887689658881741[116] = 0.0;
   out_4548887689658881741[117] = 0.0;
   out_4548887689658881741[118] = 0.0;
   out_4548887689658881741[119] = 0.0;
   out_4548887689658881741[120] = 0.0;
   out_4548887689658881741[121] = 0.0;
   out_4548887689658881741[122] = 0.0;
   out_4548887689658881741[123] = 0.0;
   out_4548887689658881741[124] = 0.0;
   out_4548887689658881741[125] = 0.0;
   out_4548887689658881741[126] = 0.0;
   out_4548887689658881741[127] = 0.0;
   out_4548887689658881741[128] = 0.0;
   out_4548887689658881741[129] = 0.0;
   out_4548887689658881741[130] = 0.0;
   out_4548887689658881741[131] = 0.0;
   out_4548887689658881741[132] = 0.0;
   out_4548887689658881741[133] = 1.0;
   out_4548887689658881741[134] = 0.0;
   out_4548887689658881741[135] = 0.0;
   out_4548887689658881741[136] = 0.0;
   out_4548887689658881741[137] = 0.0;
   out_4548887689658881741[138] = 0.0;
   out_4548887689658881741[139] = 0.0;
   out_4548887689658881741[140] = 0.0;
   out_4548887689658881741[141] = 0.0;
   out_4548887689658881741[142] = 0.0;
   out_4548887689658881741[143] = 0.0;
   out_4548887689658881741[144] = 0.0;
   out_4548887689658881741[145] = 0.0;
   out_4548887689658881741[146] = 0.0;
   out_4548887689658881741[147] = 0.0;
   out_4548887689658881741[148] = 0.0;
   out_4548887689658881741[149] = 0.0;
   out_4548887689658881741[150] = 0.0;
   out_4548887689658881741[151] = 0.0;
   out_4548887689658881741[152] = 1.0;
   out_4548887689658881741[153] = 0.0;
   out_4548887689658881741[154] = 0.0;
   out_4548887689658881741[155] = 0.0;
   out_4548887689658881741[156] = 0.0;
   out_4548887689658881741[157] = 0.0;
   out_4548887689658881741[158] = 0.0;
   out_4548887689658881741[159] = 0.0;
   out_4548887689658881741[160] = 0.0;
   out_4548887689658881741[161] = 0.0;
   out_4548887689658881741[162] = 0.0;
   out_4548887689658881741[163] = 0.0;
   out_4548887689658881741[164] = 0.0;
   out_4548887689658881741[165] = 0.0;
   out_4548887689658881741[166] = 0.0;
   out_4548887689658881741[167] = 0.0;
   out_4548887689658881741[168] = 0.0;
   out_4548887689658881741[169] = 0.0;
   out_4548887689658881741[170] = 0.0;
   out_4548887689658881741[171] = 1.0;
   out_4548887689658881741[172] = 0.0;
   out_4548887689658881741[173] = 0.0;
   out_4548887689658881741[174] = 0.0;
   out_4548887689658881741[175] = 0.0;
   out_4548887689658881741[176] = 0.0;
   out_4548887689658881741[177] = 0.0;
   out_4548887689658881741[178] = 0.0;
   out_4548887689658881741[179] = 0.0;
   out_4548887689658881741[180] = 0.0;
   out_4548887689658881741[181] = 0.0;
   out_4548887689658881741[182] = 0.0;
   out_4548887689658881741[183] = 0.0;
   out_4548887689658881741[184] = 0.0;
   out_4548887689658881741[185] = 0.0;
   out_4548887689658881741[186] = 0.0;
   out_4548887689658881741[187] = 0.0;
   out_4548887689658881741[188] = 0.0;
   out_4548887689658881741[189] = 0.0;
   out_4548887689658881741[190] = 1.0;
   out_4548887689658881741[191] = 0.0;
   out_4548887689658881741[192] = 0.0;
   out_4548887689658881741[193] = 0.0;
   out_4548887689658881741[194] = 0.0;
   out_4548887689658881741[195] = 0.0;
   out_4548887689658881741[196] = 0.0;
   out_4548887689658881741[197] = 0.0;
   out_4548887689658881741[198] = 0.0;
   out_4548887689658881741[199] = 0.0;
   out_4548887689658881741[200] = 0.0;
   out_4548887689658881741[201] = 0.0;
   out_4548887689658881741[202] = 0.0;
   out_4548887689658881741[203] = 0.0;
   out_4548887689658881741[204] = 0.0;
   out_4548887689658881741[205] = 0.0;
   out_4548887689658881741[206] = 0.0;
   out_4548887689658881741[207] = 0.0;
   out_4548887689658881741[208] = 0.0;
   out_4548887689658881741[209] = 1.0;
   out_4548887689658881741[210] = 0.0;
   out_4548887689658881741[211] = 0.0;
   out_4548887689658881741[212] = 0.0;
   out_4548887689658881741[213] = 0.0;
   out_4548887689658881741[214] = 0.0;
   out_4548887689658881741[215] = 0.0;
   out_4548887689658881741[216] = 0.0;
   out_4548887689658881741[217] = 0.0;
   out_4548887689658881741[218] = 0.0;
   out_4548887689658881741[219] = 0.0;
   out_4548887689658881741[220] = 0.0;
   out_4548887689658881741[221] = 0.0;
   out_4548887689658881741[222] = 0.0;
   out_4548887689658881741[223] = 0.0;
   out_4548887689658881741[224] = 0.0;
   out_4548887689658881741[225] = 0.0;
   out_4548887689658881741[226] = 0.0;
   out_4548887689658881741[227] = 0.0;
   out_4548887689658881741[228] = 1.0;
   out_4548887689658881741[229] = 0.0;
   out_4548887689658881741[230] = 0.0;
   out_4548887689658881741[231] = 0.0;
   out_4548887689658881741[232] = 0.0;
   out_4548887689658881741[233] = 0.0;
   out_4548887689658881741[234] = 0.0;
   out_4548887689658881741[235] = 0.0;
   out_4548887689658881741[236] = 0.0;
   out_4548887689658881741[237] = 0.0;
   out_4548887689658881741[238] = 0.0;
   out_4548887689658881741[239] = 0.0;
   out_4548887689658881741[240] = 0.0;
   out_4548887689658881741[241] = 0.0;
   out_4548887689658881741[242] = 0.0;
   out_4548887689658881741[243] = 0.0;
   out_4548887689658881741[244] = 0.0;
   out_4548887689658881741[245] = 0.0;
   out_4548887689658881741[246] = 0.0;
   out_4548887689658881741[247] = 1.0;
   out_4548887689658881741[248] = 0.0;
   out_4548887689658881741[249] = 0.0;
   out_4548887689658881741[250] = 0.0;
   out_4548887689658881741[251] = 0.0;
   out_4548887689658881741[252] = 0.0;
   out_4548887689658881741[253] = 0.0;
   out_4548887689658881741[254] = 0.0;
   out_4548887689658881741[255] = 0.0;
   out_4548887689658881741[256] = 0.0;
   out_4548887689658881741[257] = 0.0;
   out_4548887689658881741[258] = 0.0;
   out_4548887689658881741[259] = 0.0;
   out_4548887689658881741[260] = 0.0;
   out_4548887689658881741[261] = 0.0;
   out_4548887689658881741[262] = 0.0;
   out_4548887689658881741[263] = 0.0;
   out_4548887689658881741[264] = 0.0;
   out_4548887689658881741[265] = 0.0;
   out_4548887689658881741[266] = 1.0;
   out_4548887689658881741[267] = 0.0;
   out_4548887689658881741[268] = 0.0;
   out_4548887689658881741[269] = 0.0;
   out_4548887689658881741[270] = 0.0;
   out_4548887689658881741[271] = 0.0;
   out_4548887689658881741[272] = 0.0;
   out_4548887689658881741[273] = 0.0;
   out_4548887689658881741[274] = 0.0;
   out_4548887689658881741[275] = 0.0;
   out_4548887689658881741[276] = 0.0;
   out_4548887689658881741[277] = 0.0;
   out_4548887689658881741[278] = 0.0;
   out_4548887689658881741[279] = 0.0;
   out_4548887689658881741[280] = 0.0;
   out_4548887689658881741[281] = 0.0;
   out_4548887689658881741[282] = 0.0;
   out_4548887689658881741[283] = 0.0;
   out_4548887689658881741[284] = 0.0;
   out_4548887689658881741[285] = 1.0;
   out_4548887689658881741[286] = 0.0;
   out_4548887689658881741[287] = 0.0;
   out_4548887689658881741[288] = 0.0;
   out_4548887689658881741[289] = 0.0;
   out_4548887689658881741[290] = 0.0;
   out_4548887689658881741[291] = 0.0;
   out_4548887689658881741[292] = 0.0;
   out_4548887689658881741[293] = 0.0;
   out_4548887689658881741[294] = 0.0;
   out_4548887689658881741[295] = 0.0;
   out_4548887689658881741[296] = 0.0;
   out_4548887689658881741[297] = 0.0;
   out_4548887689658881741[298] = 0.0;
   out_4548887689658881741[299] = 0.0;
   out_4548887689658881741[300] = 0.0;
   out_4548887689658881741[301] = 0.0;
   out_4548887689658881741[302] = 0.0;
   out_4548887689658881741[303] = 0.0;
   out_4548887689658881741[304] = 1.0;
   out_4548887689658881741[305] = 0.0;
   out_4548887689658881741[306] = 0.0;
   out_4548887689658881741[307] = 0.0;
   out_4548887689658881741[308] = 0.0;
   out_4548887689658881741[309] = 0.0;
   out_4548887689658881741[310] = 0.0;
   out_4548887689658881741[311] = 0.0;
   out_4548887689658881741[312] = 0.0;
   out_4548887689658881741[313] = 0.0;
   out_4548887689658881741[314] = 0.0;
   out_4548887689658881741[315] = 0.0;
   out_4548887689658881741[316] = 0.0;
   out_4548887689658881741[317] = 0.0;
   out_4548887689658881741[318] = 0.0;
   out_4548887689658881741[319] = 0.0;
   out_4548887689658881741[320] = 0.0;
   out_4548887689658881741[321] = 0.0;
   out_4548887689658881741[322] = 0.0;
   out_4548887689658881741[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8947099584724172048) {
   out_8947099584724172048[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8947099584724172048[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8947099584724172048[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8947099584724172048[3] = dt*state[12] + state[3];
   out_8947099584724172048[4] = dt*state[13] + state[4];
   out_8947099584724172048[5] = dt*state[14] + state[5];
   out_8947099584724172048[6] = state[6];
   out_8947099584724172048[7] = state[7];
   out_8947099584724172048[8] = state[8];
   out_8947099584724172048[9] = state[9];
   out_8947099584724172048[10] = state[10];
   out_8947099584724172048[11] = state[11];
   out_8947099584724172048[12] = state[12];
   out_8947099584724172048[13] = state[13];
   out_8947099584724172048[14] = state[14];
   out_8947099584724172048[15] = state[15];
   out_8947099584724172048[16] = state[16];
   out_8947099584724172048[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7639000112762156858) {
   out_7639000112762156858[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7639000112762156858[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7639000112762156858[2] = 0;
   out_7639000112762156858[3] = 0;
   out_7639000112762156858[4] = 0;
   out_7639000112762156858[5] = 0;
   out_7639000112762156858[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7639000112762156858[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7639000112762156858[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7639000112762156858[9] = 0;
   out_7639000112762156858[10] = 0;
   out_7639000112762156858[11] = 0;
   out_7639000112762156858[12] = 0;
   out_7639000112762156858[13] = 0;
   out_7639000112762156858[14] = 0;
   out_7639000112762156858[15] = 0;
   out_7639000112762156858[16] = 0;
   out_7639000112762156858[17] = 0;
   out_7639000112762156858[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7639000112762156858[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7639000112762156858[20] = 0;
   out_7639000112762156858[21] = 0;
   out_7639000112762156858[22] = 0;
   out_7639000112762156858[23] = 0;
   out_7639000112762156858[24] = 0;
   out_7639000112762156858[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7639000112762156858[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7639000112762156858[27] = 0;
   out_7639000112762156858[28] = 0;
   out_7639000112762156858[29] = 0;
   out_7639000112762156858[30] = 0;
   out_7639000112762156858[31] = 0;
   out_7639000112762156858[32] = 0;
   out_7639000112762156858[33] = 0;
   out_7639000112762156858[34] = 0;
   out_7639000112762156858[35] = 0;
   out_7639000112762156858[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7639000112762156858[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7639000112762156858[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7639000112762156858[39] = 0;
   out_7639000112762156858[40] = 0;
   out_7639000112762156858[41] = 0;
   out_7639000112762156858[42] = 0;
   out_7639000112762156858[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7639000112762156858[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7639000112762156858[45] = 0;
   out_7639000112762156858[46] = 0;
   out_7639000112762156858[47] = 0;
   out_7639000112762156858[48] = 0;
   out_7639000112762156858[49] = 0;
   out_7639000112762156858[50] = 0;
   out_7639000112762156858[51] = 0;
   out_7639000112762156858[52] = 0;
   out_7639000112762156858[53] = 0;
   out_7639000112762156858[54] = 0;
   out_7639000112762156858[55] = 0;
   out_7639000112762156858[56] = 0;
   out_7639000112762156858[57] = 1;
   out_7639000112762156858[58] = 0;
   out_7639000112762156858[59] = 0;
   out_7639000112762156858[60] = 0;
   out_7639000112762156858[61] = 0;
   out_7639000112762156858[62] = 0;
   out_7639000112762156858[63] = 0;
   out_7639000112762156858[64] = 0;
   out_7639000112762156858[65] = 0;
   out_7639000112762156858[66] = dt;
   out_7639000112762156858[67] = 0;
   out_7639000112762156858[68] = 0;
   out_7639000112762156858[69] = 0;
   out_7639000112762156858[70] = 0;
   out_7639000112762156858[71] = 0;
   out_7639000112762156858[72] = 0;
   out_7639000112762156858[73] = 0;
   out_7639000112762156858[74] = 0;
   out_7639000112762156858[75] = 0;
   out_7639000112762156858[76] = 1;
   out_7639000112762156858[77] = 0;
   out_7639000112762156858[78] = 0;
   out_7639000112762156858[79] = 0;
   out_7639000112762156858[80] = 0;
   out_7639000112762156858[81] = 0;
   out_7639000112762156858[82] = 0;
   out_7639000112762156858[83] = 0;
   out_7639000112762156858[84] = 0;
   out_7639000112762156858[85] = dt;
   out_7639000112762156858[86] = 0;
   out_7639000112762156858[87] = 0;
   out_7639000112762156858[88] = 0;
   out_7639000112762156858[89] = 0;
   out_7639000112762156858[90] = 0;
   out_7639000112762156858[91] = 0;
   out_7639000112762156858[92] = 0;
   out_7639000112762156858[93] = 0;
   out_7639000112762156858[94] = 0;
   out_7639000112762156858[95] = 1;
   out_7639000112762156858[96] = 0;
   out_7639000112762156858[97] = 0;
   out_7639000112762156858[98] = 0;
   out_7639000112762156858[99] = 0;
   out_7639000112762156858[100] = 0;
   out_7639000112762156858[101] = 0;
   out_7639000112762156858[102] = 0;
   out_7639000112762156858[103] = 0;
   out_7639000112762156858[104] = dt;
   out_7639000112762156858[105] = 0;
   out_7639000112762156858[106] = 0;
   out_7639000112762156858[107] = 0;
   out_7639000112762156858[108] = 0;
   out_7639000112762156858[109] = 0;
   out_7639000112762156858[110] = 0;
   out_7639000112762156858[111] = 0;
   out_7639000112762156858[112] = 0;
   out_7639000112762156858[113] = 0;
   out_7639000112762156858[114] = 1;
   out_7639000112762156858[115] = 0;
   out_7639000112762156858[116] = 0;
   out_7639000112762156858[117] = 0;
   out_7639000112762156858[118] = 0;
   out_7639000112762156858[119] = 0;
   out_7639000112762156858[120] = 0;
   out_7639000112762156858[121] = 0;
   out_7639000112762156858[122] = 0;
   out_7639000112762156858[123] = 0;
   out_7639000112762156858[124] = 0;
   out_7639000112762156858[125] = 0;
   out_7639000112762156858[126] = 0;
   out_7639000112762156858[127] = 0;
   out_7639000112762156858[128] = 0;
   out_7639000112762156858[129] = 0;
   out_7639000112762156858[130] = 0;
   out_7639000112762156858[131] = 0;
   out_7639000112762156858[132] = 0;
   out_7639000112762156858[133] = 1;
   out_7639000112762156858[134] = 0;
   out_7639000112762156858[135] = 0;
   out_7639000112762156858[136] = 0;
   out_7639000112762156858[137] = 0;
   out_7639000112762156858[138] = 0;
   out_7639000112762156858[139] = 0;
   out_7639000112762156858[140] = 0;
   out_7639000112762156858[141] = 0;
   out_7639000112762156858[142] = 0;
   out_7639000112762156858[143] = 0;
   out_7639000112762156858[144] = 0;
   out_7639000112762156858[145] = 0;
   out_7639000112762156858[146] = 0;
   out_7639000112762156858[147] = 0;
   out_7639000112762156858[148] = 0;
   out_7639000112762156858[149] = 0;
   out_7639000112762156858[150] = 0;
   out_7639000112762156858[151] = 0;
   out_7639000112762156858[152] = 1;
   out_7639000112762156858[153] = 0;
   out_7639000112762156858[154] = 0;
   out_7639000112762156858[155] = 0;
   out_7639000112762156858[156] = 0;
   out_7639000112762156858[157] = 0;
   out_7639000112762156858[158] = 0;
   out_7639000112762156858[159] = 0;
   out_7639000112762156858[160] = 0;
   out_7639000112762156858[161] = 0;
   out_7639000112762156858[162] = 0;
   out_7639000112762156858[163] = 0;
   out_7639000112762156858[164] = 0;
   out_7639000112762156858[165] = 0;
   out_7639000112762156858[166] = 0;
   out_7639000112762156858[167] = 0;
   out_7639000112762156858[168] = 0;
   out_7639000112762156858[169] = 0;
   out_7639000112762156858[170] = 0;
   out_7639000112762156858[171] = 1;
   out_7639000112762156858[172] = 0;
   out_7639000112762156858[173] = 0;
   out_7639000112762156858[174] = 0;
   out_7639000112762156858[175] = 0;
   out_7639000112762156858[176] = 0;
   out_7639000112762156858[177] = 0;
   out_7639000112762156858[178] = 0;
   out_7639000112762156858[179] = 0;
   out_7639000112762156858[180] = 0;
   out_7639000112762156858[181] = 0;
   out_7639000112762156858[182] = 0;
   out_7639000112762156858[183] = 0;
   out_7639000112762156858[184] = 0;
   out_7639000112762156858[185] = 0;
   out_7639000112762156858[186] = 0;
   out_7639000112762156858[187] = 0;
   out_7639000112762156858[188] = 0;
   out_7639000112762156858[189] = 0;
   out_7639000112762156858[190] = 1;
   out_7639000112762156858[191] = 0;
   out_7639000112762156858[192] = 0;
   out_7639000112762156858[193] = 0;
   out_7639000112762156858[194] = 0;
   out_7639000112762156858[195] = 0;
   out_7639000112762156858[196] = 0;
   out_7639000112762156858[197] = 0;
   out_7639000112762156858[198] = 0;
   out_7639000112762156858[199] = 0;
   out_7639000112762156858[200] = 0;
   out_7639000112762156858[201] = 0;
   out_7639000112762156858[202] = 0;
   out_7639000112762156858[203] = 0;
   out_7639000112762156858[204] = 0;
   out_7639000112762156858[205] = 0;
   out_7639000112762156858[206] = 0;
   out_7639000112762156858[207] = 0;
   out_7639000112762156858[208] = 0;
   out_7639000112762156858[209] = 1;
   out_7639000112762156858[210] = 0;
   out_7639000112762156858[211] = 0;
   out_7639000112762156858[212] = 0;
   out_7639000112762156858[213] = 0;
   out_7639000112762156858[214] = 0;
   out_7639000112762156858[215] = 0;
   out_7639000112762156858[216] = 0;
   out_7639000112762156858[217] = 0;
   out_7639000112762156858[218] = 0;
   out_7639000112762156858[219] = 0;
   out_7639000112762156858[220] = 0;
   out_7639000112762156858[221] = 0;
   out_7639000112762156858[222] = 0;
   out_7639000112762156858[223] = 0;
   out_7639000112762156858[224] = 0;
   out_7639000112762156858[225] = 0;
   out_7639000112762156858[226] = 0;
   out_7639000112762156858[227] = 0;
   out_7639000112762156858[228] = 1;
   out_7639000112762156858[229] = 0;
   out_7639000112762156858[230] = 0;
   out_7639000112762156858[231] = 0;
   out_7639000112762156858[232] = 0;
   out_7639000112762156858[233] = 0;
   out_7639000112762156858[234] = 0;
   out_7639000112762156858[235] = 0;
   out_7639000112762156858[236] = 0;
   out_7639000112762156858[237] = 0;
   out_7639000112762156858[238] = 0;
   out_7639000112762156858[239] = 0;
   out_7639000112762156858[240] = 0;
   out_7639000112762156858[241] = 0;
   out_7639000112762156858[242] = 0;
   out_7639000112762156858[243] = 0;
   out_7639000112762156858[244] = 0;
   out_7639000112762156858[245] = 0;
   out_7639000112762156858[246] = 0;
   out_7639000112762156858[247] = 1;
   out_7639000112762156858[248] = 0;
   out_7639000112762156858[249] = 0;
   out_7639000112762156858[250] = 0;
   out_7639000112762156858[251] = 0;
   out_7639000112762156858[252] = 0;
   out_7639000112762156858[253] = 0;
   out_7639000112762156858[254] = 0;
   out_7639000112762156858[255] = 0;
   out_7639000112762156858[256] = 0;
   out_7639000112762156858[257] = 0;
   out_7639000112762156858[258] = 0;
   out_7639000112762156858[259] = 0;
   out_7639000112762156858[260] = 0;
   out_7639000112762156858[261] = 0;
   out_7639000112762156858[262] = 0;
   out_7639000112762156858[263] = 0;
   out_7639000112762156858[264] = 0;
   out_7639000112762156858[265] = 0;
   out_7639000112762156858[266] = 1;
   out_7639000112762156858[267] = 0;
   out_7639000112762156858[268] = 0;
   out_7639000112762156858[269] = 0;
   out_7639000112762156858[270] = 0;
   out_7639000112762156858[271] = 0;
   out_7639000112762156858[272] = 0;
   out_7639000112762156858[273] = 0;
   out_7639000112762156858[274] = 0;
   out_7639000112762156858[275] = 0;
   out_7639000112762156858[276] = 0;
   out_7639000112762156858[277] = 0;
   out_7639000112762156858[278] = 0;
   out_7639000112762156858[279] = 0;
   out_7639000112762156858[280] = 0;
   out_7639000112762156858[281] = 0;
   out_7639000112762156858[282] = 0;
   out_7639000112762156858[283] = 0;
   out_7639000112762156858[284] = 0;
   out_7639000112762156858[285] = 1;
   out_7639000112762156858[286] = 0;
   out_7639000112762156858[287] = 0;
   out_7639000112762156858[288] = 0;
   out_7639000112762156858[289] = 0;
   out_7639000112762156858[290] = 0;
   out_7639000112762156858[291] = 0;
   out_7639000112762156858[292] = 0;
   out_7639000112762156858[293] = 0;
   out_7639000112762156858[294] = 0;
   out_7639000112762156858[295] = 0;
   out_7639000112762156858[296] = 0;
   out_7639000112762156858[297] = 0;
   out_7639000112762156858[298] = 0;
   out_7639000112762156858[299] = 0;
   out_7639000112762156858[300] = 0;
   out_7639000112762156858[301] = 0;
   out_7639000112762156858[302] = 0;
   out_7639000112762156858[303] = 0;
   out_7639000112762156858[304] = 1;
   out_7639000112762156858[305] = 0;
   out_7639000112762156858[306] = 0;
   out_7639000112762156858[307] = 0;
   out_7639000112762156858[308] = 0;
   out_7639000112762156858[309] = 0;
   out_7639000112762156858[310] = 0;
   out_7639000112762156858[311] = 0;
   out_7639000112762156858[312] = 0;
   out_7639000112762156858[313] = 0;
   out_7639000112762156858[314] = 0;
   out_7639000112762156858[315] = 0;
   out_7639000112762156858[316] = 0;
   out_7639000112762156858[317] = 0;
   out_7639000112762156858[318] = 0;
   out_7639000112762156858[319] = 0;
   out_7639000112762156858[320] = 0;
   out_7639000112762156858[321] = 0;
   out_7639000112762156858[322] = 0;
   out_7639000112762156858[323] = 1;
}
void h_4(double *state, double *unused, double *out_7743631117813214181) {
   out_7743631117813214181[0] = state[6] + state[9];
   out_7743631117813214181[1] = state[7] + state[10];
   out_7743631117813214181[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3699910554686061615) {
   out_3699910554686061615[0] = 0;
   out_3699910554686061615[1] = 0;
   out_3699910554686061615[2] = 0;
   out_3699910554686061615[3] = 0;
   out_3699910554686061615[4] = 0;
   out_3699910554686061615[5] = 0;
   out_3699910554686061615[6] = 1;
   out_3699910554686061615[7] = 0;
   out_3699910554686061615[8] = 0;
   out_3699910554686061615[9] = 1;
   out_3699910554686061615[10] = 0;
   out_3699910554686061615[11] = 0;
   out_3699910554686061615[12] = 0;
   out_3699910554686061615[13] = 0;
   out_3699910554686061615[14] = 0;
   out_3699910554686061615[15] = 0;
   out_3699910554686061615[16] = 0;
   out_3699910554686061615[17] = 0;
   out_3699910554686061615[18] = 0;
   out_3699910554686061615[19] = 0;
   out_3699910554686061615[20] = 0;
   out_3699910554686061615[21] = 0;
   out_3699910554686061615[22] = 0;
   out_3699910554686061615[23] = 0;
   out_3699910554686061615[24] = 0;
   out_3699910554686061615[25] = 1;
   out_3699910554686061615[26] = 0;
   out_3699910554686061615[27] = 0;
   out_3699910554686061615[28] = 1;
   out_3699910554686061615[29] = 0;
   out_3699910554686061615[30] = 0;
   out_3699910554686061615[31] = 0;
   out_3699910554686061615[32] = 0;
   out_3699910554686061615[33] = 0;
   out_3699910554686061615[34] = 0;
   out_3699910554686061615[35] = 0;
   out_3699910554686061615[36] = 0;
   out_3699910554686061615[37] = 0;
   out_3699910554686061615[38] = 0;
   out_3699910554686061615[39] = 0;
   out_3699910554686061615[40] = 0;
   out_3699910554686061615[41] = 0;
   out_3699910554686061615[42] = 0;
   out_3699910554686061615[43] = 0;
   out_3699910554686061615[44] = 1;
   out_3699910554686061615[45] = 0;
   out_3699910554686061615[46] = 0;
   out_3699910554686061615[47] = 1;
   out_3699910554686061615[48] = 0;
   out_3699910554686061615[49] = 0;
   out_3699910554686061615[50] = 0;
   out_3699910554686061615[51] = 0;
   out_3699910554686061615[52] = 0;
   out_3699910554686061615[53] = 0;
}
void h_10(double *state, double *unused, double *out_7197434803533860657) {
   out_7197434803533860657[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7197434803533860657[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7197434803533860657[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7135399281517169195) {
   out_7135399281517169195[0] = 0;
   out_7135399281517169195[1] = 9.8100000000000005*cos(state[1]);
   out_7135399281517169195[2] = 0;
   out_7135399281517169195[3] = 0;
   out_7135399281517169195[4] = -state[8];
   out_7135399281517169195[5] = state[7];
   out_7135399281517169195[6] = 0;
   out_7135399281517169195[7] = state[5];
   out_7135399281517169195[8] = -state[4];
   out_7135399281517169195[9] = 0;
   out_7135399281517169195[10] = 0;
   out_7135399281517169195[11] = 0;
   out_7135399281517169195[12] = 1;
   out_7135399281517169195[13] = 0;
   out_7135399281517169195[14] = 0;
   out_7135399281517169195[15] = 1;
   out_7135399281517169195[16] = 0;
   out_7135399281517169195[17] = 0;
   out_7135399281517169195[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7135399281517169195[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7135399281517169195[20] = 0;
   out_7135399281517169195[21] = state[8];
   out_7135399281517169195[22] = 0;
   out_7135399281517169195[23] = -state[6];
   out_7135399281517169195[24] = -state[5];
   out_7135399281517169195[25] = 0;
   out_7135399281517169195[26] = state[3];
   out_7135399281517169195[27] = 0;
   out_7135399281517169195[28] = 0;
   out_7135399281517169195[29] = 0;
   out_7135399281517169195[30] = 0;
   out_7135399281517169195[31] = 1;
   out_7135399281517169195[32] = 0;
   out_7135399281517169195[33] = 0;
   out_7135399281517169195[34] = 1;
   out_7135399281517169195[35] = 0;
   out_7135399281517169195[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7135399281517169195[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7135399281517169195[38] = 0;
   out_7135399281517169195[39] = -state[7];
   out_7135399281517169195[40] = state[6];
   out_7135399281517169195[41] = 0;
   out_7135399281517169195[42] = state[4];
   out_7135399281517169195[43] = -state[3];
   out_7135399281517169195[44] = 0;
   out_7135399281517169195[45] = 0;
   out_7135399281517169195[46] = 0;
   out_7135399281517169195[47] = 0;
   out_7135399281517169195[48] = 0;
   out_7135399281517169195[49] = 0;
   out_7135399281517169195[50] = 1;
   out_7135399281517169195[51] = 0;
   out_7135399281517169195[52] = 0;
   out_7135399281517169195[53] = 1;
}
void h_13(double *state, double *unused, double *out_3579424207122696687) {
   out_3579424207122696687[0] = state[3];
   out_3579424207122696687[1] = state[4];
   out_3579424207122696687[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6912184380018394416) {
   out_6912184380018394416[0] = 0;
   out_6912184380018394416[1] = 0;
   out_6912184380018394416[2] = 0;
   out_6912184380018394416[3] = 1;
   out_6912184380018394416[4] = 0;
   out_6912184380018394416[5] = 0;
   out_6912184380018394416[6] = 0;
   out_6912184380018394416[7] = 0;
   out_6912184380018394416[8] = 0;
   out_6912184380018394416[9] = 0;
   out_6912184380018394416[10] = 0;
   out_6912184380018394416[11] = 0;
   out_6912184380018394416[12] = 0;
   out_6912184380018394416[13] = 0;
   out_6912184380018394416[14] = 0;
   out_6912184380018394416[15] = 0;
   out_6912184380018394416[16] = 0;
   out_6912184380018394416[17] = 0;
   out_6912184380018394416[18] = 0;
   out_6912184380018394416[19] = 0;
   out_6912184380018394416[20] = 0;
   out_6912184380018394416[21] = 0;
   out_6912184380018394416[22] = 1;
   out_6912184380018394416[23] = 0;
   out_6912184380018394416[24] = 0;
   out_6912184380018394416[25] = 0;
   out_6912184380018394416[26] = 0;
   out_6912184380018394416[27] = 0;
   out_6912184380018394416[28] = 0;
   out_6912184380018394416[29] = 0;
   out_6912184380018394416[30] = 0;
   out_6912184380018394416[31] = 0;
   out_6912184380018394416[32] = 0;
   out_6912184380018394416[33] = 0;
   out_6912184380018394416[34] = 0;
   out_6912184380018394416[35] = 0;
   out_6912184380018394416[36] = 0;
   out_6912184380018394416[37] = 0;
   out_6912184380018394416[38] = 0;
   out_6912184380018394416[39] = 0;
   out_6912184380018394416[40] = 0;
   out_6912184380018394416[41] = 1;
   out_6912184380018394416[42] = 0;
   out_6912184380018394416[43] = 0;
   out_6912184380018394416[44] = 0;
   out_6912184380018394416[45] = 0;
   out_6912184380018394416[46] = 0;
   out_6912184380018394416[47] = 0;
   out_6912184380018394416[48] = 0;
   out_6912184380018394416[49] = 0;
   out_6912184380018394416[50] = 0;
   out_6912184380018394416[51] = 0;
   out_6912184380018394416[52] = 0;
   out_6912184380018394416[53] = 0;
}
void h_14(double *state, double *unused, double *out_6838227868173306480) {
   out_6838227868173306480[0] = state[6];
   out_6838227868173306480[1] = state[7];
   out_6838227868173306480[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7663151411025546144) {
   out_7663151411025546144[0] = 0;
   out_7663151411025546144[1] = 0;
   out_7663151411025546144[2] = 0;
   out_7663151411025546144[3] = 0;
   out_7663151411025546144[4] = 0;
   out_7663151411025546144[5] = 0;
   out_7663151411025546144[6] = 1;
   out_7663151411025546144[7] = 0;
   out_7663151411025546144[8] = 0;
   out_7663151411025546144[9] = 0;
   out_7663151411025546144[10] = 0;
   out_7663151411025546144[11] = 0;
   out_7663151411025546144[12] = 0;
   out_7663151411025546144[13] = 0;
   out_7663151411025546144[14] = 0;
   out_7663151411025546144[15] = 0;
   out_7663151411025546144[16] = 0;
   out_7663151411025546144[17] = 0;
   out_7663151411025546144[18] = 0;
   out_7663151411025546144[19] = 0;
   out_7663151411025546144[20] = 0;
   out_7663151411025546144[21] = 0;
   out_7663151411025546144[22] = 0;
   out_7663151411025546144[23] = 0;
   out_7663151411025546144[24] = 0;
   out_7663151411025546144[25] = 1;
   out_7663151411025546144[26] = 0;
   out_7663151411025546144[27] = 0;
   out_7663151411025546144[28] = 0;
   out_7663151411025546144[29] = 0;
   out_7663151411025546144[30] = 0;
   out_7663151411025546144[31] = 0;
   out_7663151411025546144[32] = 0;
   out_7663151411025546144[33] = 0;
   out_7663151411025546144[34] = 0;
   out_7663151411025546144[35] = 0;
   out_7663151411025546144[36] = 0;
   out_7663151411025546144[37] = 0;
   out_7663151411025546144[38] = 0;
   out_7663151411025546144[39] = 0;
   out_7663151411025546144[40] = 0;
   out_7663151411025546144[41] = 0;
   out_7663151411025546144[42] = 0;
   out_7663151411025546144[43] = 0;
   out_7663151411025546144[44] = 1;
   out_7663151411025546144[45] = 0;
   out_7663151411025546144[46] = 0;
   out_7663151411025546144[47] = 0;
   out_7663151411025546144[48] = 0;
   out_7663151411025546144[49] = 0;
   out_7663151411025546144[50] = 0;
   out_7663151411025546144[51] = 0;
   out_7663151411025546144[52] = 0;
   out_7663151411025546144[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_1038245486379722605) {
  err_fun(nom_x, delta_x, out_1038245486379722605);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6881182548000141707) {
  inv_err_fun(nom_x, true_x, out_6881182548000141707);
}
void pose_H_mod_fun(double *state, double *out_4548887689658881741) {
  H_mod_fun(state, out_4548887689658881741);
}
void pose_f_fun(double *state, double dt, double *out_8947099584724172048) {
  f_fun(state,  dt, out_8947099584724172048);
}
void pose_F_fun(double *state, double dt, double *out_7639000112762156858) {
  F_fun(state,  dt, out_7639000112762156858);
}
void pose_h_4(double *state, double *unused, double *out_7743631117813214181) {
  h_4(state, unused, out_7743631117813214181);
}
void pose_H_4(double *state, double *unused, double *out_3699910554686061615) {
  H_4(state, unused, out_3699910554686061615);
}
void pose_h_10(double *state, double *unused, double *out_7197434803533860657) {
  h_10(state, unused, out_7197434803533860657);
}
void pose_H_10(double *state, double *unused, double *out_7135399281517169195) {
  H_10(state, unused, out_7135399281517169195);
}
void pose_h_13(double *state, double *unused, double *out_3579424207122696687) {
  h_13(state, unused, out_3579424207122696687);
}
void pose_H_13(double *state, double *unused, double *out_6912184380018394416) {
  H_13(state, unused, out_6912184380018394416);
}
void pose_h_14(double *state, double *unused, double *out_6838227868173306480) {
  h_14(state, unused, out_6838227868173306480);
}
void pose_H_14(double *state, double *unused, double *out_7663151411025546144) {
  H_14(state, unused, out_7663151411025546144);
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
