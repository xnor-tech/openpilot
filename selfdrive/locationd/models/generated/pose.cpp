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
void err_fun(double *nom_x, double *delta_x, double *out_9147823097001100337) {
   out_9147823097001100337[0] = delta_x[0] + nom_x[0];
   out_9147823097001100337[1] = delta_x[1] + nom_x[1];
   out_9147823097001100337[2] = delta_x[2] + nom_x[2];
   out_9147823097001100337[3] = delta_x[3] + nom_x[3];
   out_9147823097001100337[4] = delta_x[4] + nom_x[4];
   out_9147823097001100337[5] = delta_x[5] + nom_x[5];
   out_9147823097001100337[6] = delta_x[6] + nom_x[6];
   out_9147823097001100337[7] = delta_x[7] + nom_x[7];
   out_9147823097001100337[8] = delta_x[8] + nom_x[8];
   out_9147823097001100337[9] = delta_x[9] + nom_x[9];
   out_9147823097001100337[10] = delta_x[10] + nom_x[10];
   out_9147823097001100337[11] = delta_x[11] + nom_x[11];
   out_9147823097001100337[12] = delta_x[12] + nom_x[12];
   out_9147823097001100337[13] = delta_x[13] + nom_x[13];
   out_9147823097001100337[14] = delta_x[14] + nom_x[14];
   out_9147823097001100337[15] = delta_x[15] + nom_x[15];
   out_9147823097001100337[16] = delta_x[16] + nom_x[16];
   out_9147823097001100337[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6976768038486665276) {
   out_6976768038486665276[0] = -nom_x[0] + true_x[0];
   out_6976768038486665276[1] = -nom_x[1] + true_x[1];
   out_6976768038486665276[2] = -nom_x[2] + true_x[2];
   out_6976768038486665276[3] = -nom_x[3] + true_x[3];
   out_6976768038486665276[4] = -nom_x[4] + true_x[4];
   out_6976768038486665276[5] = -nom_x[5] + true_x[5];
   out_6976768038486665276[6] = -nom_x[6] + true_x[6];
   out_6976768038486665276[7] = -nom_x[7] + true_x[7];
   out_6976768038486665276[8] = -nom_x[8] + true_x[8];
   out_6976768038486665276[9] = -nom_x[9] + true_x[9];
   out_6976768038486665276[10] = -nom_x[10] + true_x[10];
   out_6976768038486665276[11] = -nom_x[11] + true_x[11];
   out_6976768038486665276[12] = -nom_x[12] + true_x[12];
   out_6976768038486665276[13] = -nom_x[13] + true_x[13];
   out_6976768038486665276[14] = -nom_x[14] + true_x[14];
   out_6976768038486665276[15] = -nom_x[15] + true_x[15];
   out_6976768038486665276[16] = -nom_x[16] + true_x[16];
   out_6976768038486665276[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2524355098589883270) {
   out_2524355098589883270[0] = 1.0;
   out_2524355098589883270[1] = 0.0;
   out_2524355098589883270[2] = 0.0;
   out_2524355098589883270[3] = 0.0;
   out_2524355098589883270[4] = 0.0;
   out_2524355098589883270[5] = 0.0;
   out_2524355098589883270[6] = 0.0;
   out_2524355098589883270[7] = 0.0;
   out_2524355098589883270[8] = 0.0;
   out_2524355098589883270[9] = 0.0;
   out_2524355098589883270[10] = 0.0;
   out_2524355098589883270[11] = 0.0;
   out_2524355098589883270[12] = 0.0;
   out_2524355098589883270[13] = 0.0;
   out_2524355098589883270[14] = 0.0;
   out_2524355098589883270[15] = 0.0;
   out_2524355098589883270[16] = 0.0;
   out_2524355098589883270[17] = 0.0;
   out_2524355098589883270[18] = 0.0;
   out_2524355098589883270[19] = 1.0;
   out_2524355098589883270[20] = 0.0;
   out_2524355098589883270[21] = 0.0;
   out_2524355098589883270[22] = 0.0;
   out_2524355098589883270[23] = 0.0;
   out_2524355098589883270[24] = 0.0;
   out_2524355098589883270[25] = 0.0;
   out_2524355098589883270[26] = 0.0;
   out_2524355098589883270[27] = 0.0;
   out_2524355098589883270[28] = 0.0;
   out_2524355098589883270[29] = 0.0;
   out_2524355098589883270[30] = 0.0;
   out_2524355098589883270[31] = 0.0;
   out_2524355098589883270[32] = 0.0;
   out_2524355098589883270[33] = 0.0;
   out_2524355098589883270[34] = 0.0;
   out_2524355098589883270[35] = 0.0;
   out_2524355098589883270[36] = 0.0;
   out_2524355098589883270[37] = 0.0;
   out_2524355098589883270[38] = 1.0;
   out_2524355098589883270[39] = 0.0;
   out_2524355098589883270[40] = 0.0;
   out_2524355098589883270[41] = 0.0;
   out_2524355098589883270[42] = 0.0;
   out_2524355098589883270[43] = 0.0;
   out_2524355098589883270[44] = 0.0;
   out_2524355098589883270[45] = 0.0;
   out_2524355098589883270[46] = 0.0;
   out_2524355098589883270[47] = 0.0;
   out_2524355098589883270[48] = 0.0;
   out_2524355098589883270[49] = 0.0;
   out_2524355098589883270[50] = 0.0;
   out_2524355098589883270[51] = 0.0;
   out_2524355098589883270[52] = 0.0;
   out_2524355098589883270[53] = 0.0;
   out_2524355098589883270[54] = 0.0;
   out_2524355098589883270[55] = 0.0;
   out_2524355098589883270[56] = 0.0;
   out_2524355098589883270[57] = 1.0;
   out_2524355098589883270[58] = 0.0;
   out_2524355098589883270[59] = 0.0;
   out_2524355098589883270[60] = 0.0;
   out_2524355098589883270[61] = 0.0;
   out_2524355098589883270[62] = 0.0;
   out_2524355098589883270[63] = 0.0;
   out_2524355098589883270[64] = 0.0;
   out_2524355098589883270[65] = 0.0;
   out_2524355098589883270[66] = 0.0;
   out_2524355098589883270[67] = 0.0;
   out_2524355098589883270[68] = 0.0;
   out_2524355098589883270[69] = 0.0;
   out_2524355098589883270[70] = 0.0;
   out_2524355098589883270[71] = 0.0;
   out_2524355098589883270[72] = 0.0;
   out_2524355098589883270[73] = 0.0;
   out_2524355098589883270[74] = 0.0;
   out_2524355098589883270[75] = 0.0;
   out_2524355098589883270[76] = 1.0;
   out_2524355098589883270[77] = 0.0;
   out_2524355098589883270[78] = 0.0;
   out_2524355098589883270[79] = 0.0;
   out_2524355098589883270[80] = 0.0;
   out_2524355098589883270[81] = 0.0;
   out_2524355098589883270[82] = 0.0;
   out_2524355098589883270[83] = 0.0;
   out_2524355098589883270[84] = 0.0;
   out_2524355098589883270[85] = 0.0;
   out_2524355098589883270[86] = 0.0;
   out_2524355098589883270[87] = 0.0;
   out_2524355098589883270[88] = 0.0;
   out_2524355098589883270[89] = 0.0;
   out_2524355098589883270[90] = 0.0;
   out_2524355098589883270[91] = 0.0;
   out_2524355098589883270[92] = 0.0;
   out_2524355098589883270[93] = 0.0;
   out_2524355098589883270[94] = 0.0;
   out_2524355098589883270[95] = 1.0;
   out_2524355098589883270[96] = 0.0;
   out_2524355098589883270[97] = 0.0;
   out_2524355098589883270[98] = 0.0;
   out_2524355098589883270[99] = 0.0;
   out_2524355098589883270[100] = 0.0;
   out_2524355098589883270[101] = 0.0;
   out_2524355098589883270[102] = 0.0;
   out_2524355098589883270[103] = 0.0;
   out_2524355098589883270[104] = 0.0;
   out_2524355098589883270[105] = 0.0;
   out_2524355098589883270[106] = 0.0;
   out_2524355098589883270[107] = 0.0;
   out_2524355098589883270[108] = 0.0;
   out_2524355098589883270[109] = 0.0;
   out_2524355098589883270[110] = 0.0;
   out_2524355098589883270[111] = 0.0;
   out_2524355098589883270[112] = 0.0;
   out_2524355098589883270[113] = 0.0;
   out_2524355098589883270[114] = 1.0;
   out_2524355098589883270[115] = 0.0;
   out_2524355098589883270[116] = 0.0;
   out_2524355098589883270[117] = 0.0;
   out_2524355098589883270[118] = 0.0;
   out_2524355098589883270[119] = 0.0;
   out_2524355098589883270[120] = 0.0;
   out_2524355098589883270[121] = 0.0;
   out_2524355098589883270[122] = 0.0;
   out_2524355098589883270[123] = 0.0;
   out_2524355098589883270[124] = 0.0;
   out_2524355098589883270[125] = 0.0;
   out_2524355098589883270[126] = 0.0;
   out_2524355098589883270[127] = 0.0;
   out_2524355098589883270[128] = 0.0;
   out_2524355098589883270[129] = 0.0;
   out_2524355098589883270[130] = 0.0;
   out_2524355098589883270[131] = 0.0;
   out_2524355098589883270[132] = 0.0;
   out_2524355098589883270[133] = 1.0;
   out_2524355098589883270[134] = 0.0;
   out_2524355098589883270[135] = 0.0;
   out_2524355098589883270[136] = 0.0;
   out_2524355098589883270[137] = 0.0;
   out_2524355098589883270[138] = 0.0;
   out_2524355098589883270[139] = 0.0;
   out_2524355098589883270[140] = 0.0;
   out_2524355098589883270[141] = 0.0;
   out_2524355098589883270[142] = 0.0;
   out_2524355098589883270[143] = 0.0;
   out_2524355098589883270[144] = 0.0;
   out_2524355098589883270[145] = 0.0;
   out_2524355098589883270[146] = 0.0;
   out_2524355098589883270[147] = 0.0;
   out_2524355098589883270[148] = 0.0;
   out_2524355098589883270[149] = 0.0;
   out_2524355098589883270[150] = 0.0;
   out_2524355098589883270[151] = 0.0;
   out_2524355098589883270[152] = 1.0;
   out_2524355098589883270[153] = 0.0;
   out_2524355098589883270[154] = 0.0;
   out_2524355098589883270[155] = 0.0;
   out_2524355098589883270[156] = 0.0;
   out_2524355098589883270[157] = 0.0;
   out_2524355098589883270[158] = 0.0;
   out_2524355098589883270[159] = 0.0;
   out_2524355098589883270[160] = 0.0;
   out_2524355098589883270[161] = 0.0;
   out_2524355098589883270[162] = 0.0;
   out_2524355098589883270[163] = 0.0;
   out_2524355098589883270[164] = 0.0;
   out_2524355098589883270[165] = 0.0;
   out_2524355098589883270[166] = 0.0;
   out_2524355098589883270[167] = 0.0;
   out_2524355098589883270[168] = 0.0;
   out_2524355098589883270[169] = 0.0;
   out_2524355098589883270[170] = 0.0;
   out_2524355098589883270[171] = 1.0;
   out_2524355098589883270[172] = 0.0;
   out_2524355098589883270[173] = 0.0;
   out_2524355098589883270[174] = 0.0;
   out_2524355098589883270[175] = 0.0;
   out_2524355098589883270[176] = 0.0;
   out_2524355098589883270[177] = 0.0;
   out_2524355098589883270[178] = 0.0;
   out_2524355098589883270[179] = 0.0;
   out_2524355098589883270[180] = 0.0;
   out_2524355098589883270[181] = 0.0;
   out_2524355098589883270[182] = 0.0;
   out_2524355098589883270[183] = 0.0;
   out_2524355098589883270[184] = 0.0;
   out_2524355098589883270[185] = 0.0;
   out_2524355098589883270[186] = 0.0;
   out_2524355098589883270[187] = 0.0;
   out_2524355098589883270[188] = 0.0;
   out_2524355098589883270[189] = 0.0;
   out_2524355098589883270[190] = 1.0;
   out_2524355098589883270[191] = 0.0;
   out_2524355098589883270[192] = 0.0;
   out_2524355098589883270[193] = 0.0;
   out_2524355098589883270[194] = 0.0;
   out_2524355098589883270[195] = 0.0;
   out_2524355098589883270[196] = 0.0;
   out_2524355098589883270[197] = 0.0;
   out_2524355098589883270[198] = 0.0;
   out_2524355098589883270[199] = 0.0;
   out_2524355098589883270[200] = 0.0;
   out_2524355098589883270[201] = 0.0;
   out_2524355098589883270[202] = 0.0;
   out_2524355098589883270[203] = 0.0;
   out_2524355098589883270[204] = 0.0;
   out_2524355098589883270[205] = 0.0;
   out_2524355098589883270[206] = 0.0;
   out_2524355098589883270[207] = 0.0;
   out_2524355098589883270[208] = 0.0;
   out_2524355098589883270[209] = 1.0;
   out_2524355098589883270[210] = 0.0;
   out_2524355098589883270[211] = 0.0;
   out_2524355098589883270[212] = 0.0;
   out_2524355098589883270[213] = 0.0;
   out_2524355098589883270[214] = 0.0;
   out_2524355098589883270[215] = 0.0;
   out_2524355098589883270[216] = 0.0;
   out_2524355098589883270[217] = 0.0;
   out_2524355098589883270[218] = 0.0;
   out_2524355098589883270[219] = 0.0;
   out_2524355098589883270[220] = 0.0;
   out_2524355098589883270[221] = 0.0;
   out_2524355098589883270[222] = 0.0;
   out_2524355098589883270[223] = 0.0;
   out_2524355098589883270[224] = 0.0;
   out_2524355098589883270[225] = 0.0;
   out_2524355098589883270[226] = 0.0;
   out_2524355098589883270[227] = 0.0;
   out_2524355098589883270[228] = 1.0;
   out_2524355098589883270[229] = 0.0;
   out_2524355098589883270[230] = 0.0;
   out_2524355098589883270[231] = 0.0;
   out_2524355098589883270[232] = 0.0;
   out_2524355098589883270[233] = 0.0;
   out_2524355098589883270[234] = 0.0;
   out_2524355098589883270[235] = 0.0;
   out_2524355098589883270[236] = 0.0;
   out_2524355098589883270[237] = 0.0;
   out_2524355098589883270[238] = 0.0;
   out_2524355098589883270[239] = 0.0;
   out_2524355098589883270[240] = 0.0;
   out_2524355098589883270[241] = 0.0;
   out_2524355098589883270[242] = 0.0;
   out_2524355098589883270[243] = 0.0;
   out_2524355098589883270[244] = 0.0;
   out_2524355098589883270[245] = 0.0;
   out_2524355098589883270[246] = 0.0;
   out_2524355098589883270[247] = 1.0;
   out_2524355098589883270[248] = 0.0;
   out_2524355098589883270[249] = 0.0;
   out_2524355098589883270[250] = 0.0;
   out_2524355098589883270[251] = 0.0;
   out_2524355098589883270[252] = 0.0;
   out_2524355098589883270[253] = 0.0;
   out_2524355098589883270[254] = 0.0;
   out_2524355098589883270[255] = 0.0;
   out_2524355098589883270[256] = 0.0;
   out_2524355098589883270[257] = 0.0;
   out_2524355098589883270[258] = 0.0;
   out_2524355098589883270[259] = 0.0;
   out_2524355098589883270[260] = 0.0;
   out_2524355098589883270[261] = 0.0;
   out_2524355098589883270[262] = 0.0;
   out_2524355098589883270[263] = 0.0;
   out_2524355098589883270[264] = 0.0;
   out_2524355098589883270[265] = 0.0;
   out_2524355098589883270[266] = 1.0;
   out_2524355098589883270[267] = 0.0;
   out_2524355098589883270[268] = 0.0;
   out_2524355098589883270[269] = 0.0;
   out_2524355098589883270[270] = 0.0;
   out_2524355098589883270[271] = 0.0;
   out_2524355098589883270[272] = 0.0;
   out_2524355098589883270[273] = 0.0;
   out_2524355098589883270[274] = 0.0;
   out_2524355098589883270[275] = 0.0;
   out_2524355098589883270[276] = 0.0;
   out_2524355098589883270[277] = 0.0;
   out_2524355098589883270[278] = 0.0;
   out_2524355098589883270[279] = 0.0;
   out_2524355098589883270[280] = 0.0;
   out_2524355098589883270[281] = 0.0;
   out_2524355098589883270[282] = 0.0;
   out_2524355098589883270[283] = 0.0;
   out_2524355098589883270[284] = 0.0;
   out_2524355098589883270[285] = 1.0;
   out_2524355098589883270[286] = 0.0;
   out_2524355098589883270[287] = 0.0;
   out_2524355098589883270[288] = 0.0;
   out_2524355098589883270[289] = 0.0;
   out_2524355098589883270[290] = 0.0;
   out_2524355098589883270[291] = 0.0;
   out_2524355098589883270[292] = 0.0;
   out_2524355098589883270[293] = 0.0;
   out_2524355098589883270[294] = 0.0;
   out_2524355098589883270[295] = 0.0;
   out_2524355098589883270[296] = 0.0;
   out_2524355098589883270[297] = 0.0;
   out_2524355098589883270[298] = 0.0;
   out_2524355098589883270[299] = 0.0;
   out_2524355098589883270[300] = 0.0;
   out_2524355098589883270[301] = 0.0;
   out_2524355098589883270[302] = 0.0;
   out_2524355098589883270[303] = 0.0;
   out_2524355098589883270[304] = 1.0;
   out_2524355098589883270[305] = 0.0;
   out_2524355098589883270[306] = 0.0;
   out_2524355098589883270[307] = 0.0;
   out_2524355098589883270[308] = 0.0;
   out_2524355098589883270[309] = 0.0;
   out_2524355098589883270[310] = 0.0;
   out_2524355098589883270[311] = 0.0;
   out_2524355098589883270[312] = 0.0;
   out_2524355098589883270[313] = 0.0;
   out_2524355098589883270[314] = 0.0;
   out_2524355098589883270[315] = 0.0;
   out_2524355098589883270[316] = 0.0;
   out_2524355098589883270[317] = 0.0;
   out_2524355098589883270[318] = 0.0;
   out_2524355098589883270[319] = 0.0;
   out_2524355098589883270[320] = 0.0;
   out_2524355098589883270[321] = 0.0;
   out_2524355098589883270[322] = 0.0;
   out_2524355098589883270[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2560858857775655991) {
   out_2560858857775655991[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2560858857775655991[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2560858857775655991[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2560858857775655991[3] = dt*state[12] + state[3];
   out_2560858857775655991[4] = dt*state[13] + state[4];
   out_2560858857775655991[5] = dt*state[14] + state[5];
   out_2560858857775655991[6] = state[6];
   out_2560858857775655991[7] = state[7];
   out_2560858857775655991[8] = state[8];
   out_2560858857775655991[9] = state[9];
   out_2560858857775655991[10] = state[10];
   out_2560858857775655991[11] = state[11];
   out_2560858857775655991[12] = state[12];
   out_2560858857775655991[13] = state[13];
   out_2560858857775655991[14] = state[14];
   out_2560858857775655991[15] = state[15];
   out_2560858857775655991[16] = state[16];
   out_2560858857775655991[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6720701162118872845) {
   out_6720701162118872845[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6720701162118872845[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6720701162118872845[2] = 0;
   out_6720701162118872845[3] = 0;
   out_6720701162118872845[4] = 0;
   out_6720701162118872845[5] = 0;
   out_6720701162118872845[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6720701162118872845[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6720701162118872845[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6720701162118872845[9] = 0;
   out_6720701162118872845[10] = 0;
   out_6720701162118872845[11] = 0;
   out_6720701162118872845[12] = 0;
   out_6720701162118872845[13] = 0;
   out_6720701162118872845[14] = 0;
   out_6720701162118872845[15] = 0;
   out_6720701162118872845[16] = 0;
   out_6720701162118872845[17] = 0;
   out_6720701162118872845[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6720701162118872845[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6720701162118872845[20] = 0;
   out_6720701162118872845[21] = 0;
   out_6720701162118872845[22] = 0;
   out_6720701162118872845[23] = 0;
   out_6720701162118872845[24] = 0;
   out_6720701162118872845[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6720701162118872845[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6720701162118872845[27] = 0;
   out_6720701162118872845[28] = 0;
   out_6720701162118872845[29] = 0;
   out_6720701162118872845[30] = 0;
   out_6720701162118872845[31] = 0;
   out_6720701162118872845[32] = 0;
   out_6720701162118872845[33] = 0;
   out_6720701162118872845[34] = 0;
   out_6720701162118872845[35] = 0;
   out_6720701162118872845[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6720701162118872845[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6720701162118872845[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6720701162118872845[39] = 0;
   out_6720701162118872845[40] = 0;
   out_6720701162118872845[41] = 0;
   out_6720701162118872845[42] = 0;
   out_6720701162118872845[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6720701162118872845[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6720701162118872845[45] = 0;
   out_6720701162118872845[46] = 0;
   out_6720701162118872845[47] = 0;
   out_6720701162118872845[48] = 0;
   out_6720701162118872845[49] = 0;
   out_6720701162118872845[50] = 0;
   out_6720701162118872845[51] = 0;
   out_6720701162118872845[52] = 0;
   out_6720701162118872845[53] = 0;
   out_6720701162118872845[54] = 0;
   out_6720701162118872845[55] = 0;
   out_6720701162118872845[56] = 0;
   out_6720701162118872845[57] = 1;
   out_6720701162118872845[58] = 0;
   out_6720701162118872845[59] = 0;
   out_6720701162118872845[60] = 0;
   out_6720701162118872845[61] = 0;
   out_6720701162118872845[62] = 0;
   out_6720701162118872845[63] = 0;
   out_6720701162118872845[64] = 0;
   out_6720701162118872845[65] = 0;
   out_6720701162118872845[66] = dt;
   out_6720701162118872845[67] = 0;
   out_6720701162118872845[68] = 0;
   out_6720701162118872845[69] = 0;
   out_6720701162118872845[70] = 0;
   out_6720701162118872845[71] = 0;
   out_6720701162118872845[72] = 0;
   out_6720701162118872845[73] = 0;
   out_6720701162118872845[74] = 0;
   out_6720701162118872845[75] = 0;
   out_6720701162118872845[76] = 1;
   out_6720701162118872845[77] = 0;
   out_6720701162118872845[78] = 0;
   out_6720701162118872845[79] = 0;
   out_6720701162118872845[80] = 0;
   out_6720701162118872845[81] = 0;
   out_6720701162118872845[82] = 0;
   out_6720701162118872845[83] = 0;
   out_6720701162118872845[84] = 0;
   out_6720701162118872845[85] = dt;
   out_6720701162118872845[86] = 0;
   out_6720701162118872845[87] = 0;
   out_6720701162118872845[88] = 0;
   out_6720701162118872845[89] = 0;
   out_6720701162118872845[90] = 0;
   out_6720701162118872845[91] = 0;
   out_6720701162118872845[92] = 0;
   out_6720701162118872845[93] = 0;
   out_6720701162118872845[94] = 0;
   out_6720701162118872845[95] = 1;
   out_6720701162118872845[96] = 0;
   out_6720701162118872845[97] = 0;
   out_6720701162118872845[98] = 0;
   out_6720701162118872845[99] = 0;
   out_6720701162118872845[100] = 0;
   out_6720701162118872845[101] = 0;
   out_6720701162118872845[102] = 0;
   out_6720701162118872845[103] = 0;
   out_6720701162118872845[104] = dt;
   out_6720701162118872845[105] = 0;
   out_6720701162118872845[106] = 0;
   out_6720701162118872845[107] = 0;
   out_6720701162118872845[108] = 0;
   out_6720701162118872845[109] = 0;
   out_6720701162118872845[110] = 0;
   out_6720701162118872845[111] = 0;
   out_6720701162118872845[112] = 0;
   out_6720701162118872845[113] = 0;
   out_6720701162118872845[114] = 1;
   out_6720701162118872845[115] = 0;
   out_6720701162118872845[116] = 0;
   out_6720701162118872845[117] = 0;
   out_6720701162118872845[118] = 0;
   out_6720701162118872845[119] = 0;
   out_6720701162118872845[120] = 0;
   out_6720701162118872845[121] = 0;
   out_6720701162118872845[122] = 0;
   out_6720701162118872845[123] = 0;
   out_6720701162118872845[124] = 0;
   out_6720701162118872845[125] = 0;
   out_6720701162118872845[126] = 0;
   out_6720701162118872845[127] = 0;
   out_6720701162118872845[128] = 0;
   out_6720701162118872845[129] = 0;
   out_6720701162118872845[130] = 0;
   out_6720701162118872845[131] = 0;
   out_6720701162118872845[132] = 0;
   out_6720701162118872845[133] = 1;
   out_6720701162118872845[134] = 0;
   out_6720701162118872845[135] = 0;
   out_6720701162118872845[136] = 0;
   out_6720701162118872845[137] = 0;
   out_6720701162118872845[138] = 0;
   out_6720701162118872845[139] = 0;
   out_6720701162118872845[140] = 0;
   out_6720701162118872845[141] = 0;
   out_6720701162118872845[142] = 0;
   out_6720701162118872845[143] = 0;
   out_6720701162118872845[144] = 0;
   out_6720701162118872845[145] = 0;
   out_6720701162118872845[146] = 0;
   out_6720701162118872845[147] = 0;
   out_6720701162118872845[148] = 0;
   out_6720701162118872845[149] = 0;
   out_6720701162118872845[150] = 0;
   out_6720701162118872845[151] = 0;
   out_6720701162118872845[152] = 1;
   out_6720701162118872845[153] = 0;
   out_6720701162118872845[154] = 0;
   out_6720701162118872845[155] = 0;
   out_6720701162118872845[156] = 0;
   out_6720701162118872845[157] = 0;
   out_6720701162118872845[158] = 0;
   out_6720701162118872845[159] = 0;
   out_6720701162118872845[160] = 0;
   out_6720701162118872845[161] = 0;
   out_6720701162118872845[162] = 0;
   out_6720701162118872845[163] = 0;
   out_6720701162118872845[164] = 0;
   out_6720701162118872845[165] = 0;
   out_6720701162118872845[166] = 0;
   out_6720701162118872845[167] = 0;
   out_6720701162118872845[168] = 0;
   out_6720701162118872845[169] = 0;
   out_6720701162118872845[170] = 0;
   out_6720701162118872845[171] = 1;
   out_6720701162118872845[172] = 0;
   out_6720701162118872845[173] = 0;
   out_6720701162118872845[174] = 0;
   out_6720701162118872845[175] = 0;
   out_6720701162118872845[176] = 0;
   out_6720701162118872845[177] = 0;
   out_6720701162118872845[178] = 0;
   out_6720701162118872845[179] = 0;
   out_6720701162118872845[180] = 0;
   out_6720701162118872845[181] = 0;
   out_6720701162118872845[182] = 0;
   out_6720701162118872845[183] = 0;
   out_6720701162118872845[184] = 0;
   out_6720701162118872845[185] = 0;
   out_6720701162118872845[186] = 0;
   out_6720701162118872845[187] = 0;
   out_6720701162118872845[188] = 0;
   out_6720701162118872845[189] = 0;
   out_6720701162118872845[190] = 1;
   out_6720701162118872845[191] = 0;
   out_6720701162118872845[192] = 0;
   out_6720701162118872845[193] = 0;
   out_6720701162118872845[194] = 0;
   out_6720701162118872845[195] = 0;
   out_6720701162118872845[196] = 0;
   out_6720701162118872845[197] = 0;
   out_6720701162118872845[198] = 0;
   out_6720701162118872845[199] = 0;
   out_6720701162118872845[200] = 0;
   out_6720701162118872845[201] = 0;
   out_6720701162118872845[202] = 0;
   out_6720701162118872845[203] = 0;
   out_6720701162118872845[204] = 0;
   out_6720701162118872845[205] = 0;
   out_6720701162118872845[206] = 0;
   out_6720701162118872845[207] = 0;
   out_6720701162118872845[208] = 0;
   out_6720701162118872845[209] = 1;
   out_6720701162118872845[210] = 0;
   out_6720701162118872845[211] = 0;
   out_6720701162118872845[212] = 0;
   out_6720701162118872845[213] = 0;
   out_6720701162118872845[214] = 0;
   out_6720701162118872845[215] = 0;
   out_6720701162118872845[216] = 0;
   out_6720701162118872845[217] = 0;
   out_6720701162118872845[218] = 0;
   out_6720701162118872845[219] = 0;
   out_6720701162118872845[220] = 0;
   out_6720701162118872845[221] = 0;
   out_6720701162118872845[222] = 0;
   out_6720701162118872845[223] = 0;
   out_6720701162118872845[224] = 0;
   out_6720701162118872845[225] = 0;
   out_6720701162118872845[226] = 0;
   out_6720701162118872845[227] = 0;
   out_6720701162118872845[228] = 1;
   out_6720701162118872845[229] = 0;
   out_6720701162118872845[230] = 0;
   out_6720701162118872845[231] = 0;
   out_6720701162118872845[232] = 0;
   out_6720701162118872845[233] = 0;
   out_6720701162118872845[234] = 0;
   out_6720701162118872845[235] = 0;
   out_6720701162118872845[236] = 0;
   out_6720701162118872845[237] = 0;
   out_6720701162118872845[238] = 0;
   out_6720701162118872845[239] = 0;
   out_6720701162118872845[240] = 0;
   out_6720701162118872845[241] = 0;
   out_6720701162118872845[242] = 0;
   out_6720701162118872845[243] = 0;
   out_6720701162118872845[244] = 0;
   out_6720701162118872845[245] = 0;
   out_6720701162118872845[246] = 0;
   out_6720701162118872845[247] = 1;
   out_6720701162118872845[248] = 0;
   out_6720701162118872845[249] = 0;
   out_6720701162118872845[250] = 0;
   out_6720701162118872845[251] = 0;
   out_6720701162118872845[252] = 0;
   out_6720701162118872845[253] = 0;
   out_6720701162118872845[254] = 0;
   out_6720701162118872845[255] = 0;
   out_6720701162118872845[256] = 0;
   out_6720701162118872845[257] = 0;
   out_6720701162118872845[258] = 0;
   out_6720701162118872845[259] = 0;
   out_6720701162118872845[260] = 0;
   out_6720701162118872845[261] = 0;
   out_6720701162118872845[262] = 0;
   out_6720701162118872845[263] = 0;
   out_6720701162118872845[264] = 0;
   out_6720701162118872845[265] = 0;
   out_6720701162118872845[266] = 1;
   out_6720701162118872845[267] = 0;
   out_6720701162118872845[268] = 0;
   out_6720701162118872845[269] = 0;
   out_6720701162118872845[270] = 0;
   out_6720701162118872845[271] = 0;
   out_6720701162118872845[272] = 0;
   out_6720701162118872845[273] = 0;
   out_6720701162118872845[274] = 0;
   out_6720701162118872845[275] = 0;
   out_6720701162118872845[276] = 0;
   out_6720701162118872845[277] = 0;
   out_6720701162118872845[278] = 0;
   out_6720701162118872845[279] = 0;
   out_6720701162118872845[280] = 0;
   out_6720701162118872845[281] = 0;
   out_6720701162118872845[282] = 0;
   out_6720701162118872845[283] = 0;
   out_6720701162118872845[284] = 0;
   out_6720701162118872845[285] = 1;
   out_6720701162118872845[286] = 0;
   out_6720701162118872845[287] = 0;
   out_6720701162118872845[288] = 0;
   out_6720701162118872845[289] = 0;
   out_6720701162118872845[290] = 0;
   out_6720701162118872845[291] = 0;
   out_6720701162118872845[292] = 0;
   out_6720701162118872845[293] = 0;
   out_6720701162118872845[294] = 0;
   out_6720701162118872845[295] = 0;
   out_6720701162118872845[296] = 0;
   out_6720701162118872845[297] = 0;
   out_6720701162118872845[298] = 0;
   out_6720701162118872845[299] = 0;
   out_6720701162118872845[300] = 0;
   out_6720701162118872845[301] = 0;
   out_6720701162118872845[302] = 0;
   out_6720701162118872845[303] = 0;
   out_6720701162118872845[304] = 1;
   out_6720701162118872845[305] = 0;
   out_6720701162118872845[306] = 0;
   out_6720701162118872845[307] = 0;
   out_6720701162118872845[308] = 0;
   out_6720701162118872845[309] = 0;
   out_6720701162118872845[310] = 0;
   out_6720701162118872845[311] = 0;
   out_6720701162118872845[312] = 0;
   out_6720701162118872845[313] = 0;
   out_6720701162118872845[314] = 0;
   out_6720701162118872845[315] = 0;
   out_6720701162118872845[316] = 0;
   out_6720701162118872845[317] = 0;
   out_6720701162118872845[318] = 0;
   out_6720701162118872845[319] = 0;
   out_6720701162118872845[320] = 0;
   out_6720701162118872845[321] = 0;
   out_6720701162118872845[322] = 0;
   out_6720701162118872845[323] = 1;
}
void h_4(double *state, double *unused, double *out_4511036304417573681) {
   out_4511036304417573681[0] = state[6] + state[9];
   out_4511036304417573681[1] = state[7] + state[10];
   out_4511036304417573681[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7722084091146443579) {
   out_7722084091146443579[0] = 0;
   out_7722084091146443579[1] = 0;
   out_7722084091146443579[2] = 0;
   out_7722084091146443579[3] = 0;
   out_7722084091146443579[4] = 0;
   out_7722084091146443579[5] = 0;
   out_7722084091146443579[6] = 1;
   out_7722084091146443579[7] = 0;
   out_7722084091146443579[8] = 0;
   out_7722084091146443579[9] = 1;
   out_7722084091146443579[10] = 0;
   out_7722084091146443579[11] = 0;
   out_7722084091146443579[12] = 0;
   out_7722084091146443579[13] = 0;
   out_7722084091146443579[14] = 0;
   out_7722084091146443579[15] = 0;
   out_7722084091146443579[16] = 0;
   out_7722084091146443579[17] = 0;
   out_7722084091146443579[18] = 0;
   out_7722084091146443579[19] = 0;
   out_7722084091146443579[20] = 0;
   out_7722084091146443579[21] = 0;
   out_7722084091146443579[22] = 0;
   out_7722084091146443579[23] = 0;
   out_7722084091146443579[24] = 0;
   out_7722084091146443579[25] = 1;
   out_7722084091146443579[26] = 0;
   out_7722084091146443579[27] = 0;
   out_7722084091146443579[28] = 1;
   out_7722084091146443579[29] = 0;
   out_7722084091146443579[30] = 0;
   out_7722084091146443579[31] = 0;
   out_7722084091146443579[32] = 0;
   out_7722084091146443579[33] = 0;
   out_7722084091146443579[34] = 0;
   out_7722084091146443579[35] = 0;
   out_7722084091146443579[36] = 0;
   out_7722084091146443579[37] = 0;
   out_7722084091146443579[38] = 0;
   out_7722084091146443579[39] = 0;
   out_7722084091146443579[40] = 0;
   out_7722084091146443579[41] = 0;
   out_7722084091146443579[42] = 0;
   out_7722084091146443579[43] = 0;
   out_7722084091146443579[44] = 1;
   out_7722084091146443579[45] = 0;
   out_7722084091146443579[46] = 0;
   out_7722084091146443579[47] = 1;
   out_7722084091146443579[48] = 0;
   out_7722084091146443579[49] = 0;
   out_7722084091146443579[50] = 0;
   out_7722084091146443579[51] = 0;
   out_7722084091146443579[52] = 0;
   out_7722084091146443579[53] = 0;
}
void h_10(double *state, double *unused, double *out_2196926179427311242) {
   out_2196926179427311242[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2196926179427311242[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2196926179427311242[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_93714966278450490) {
   out_93714966278450490[0] = 0;
   out_93714966278450490[1] = 9.8100000000000005*cos(state[1]);
   out_93714966278450490[2] = 0;
   out_93714966278450490[3] = 0;
   out_93714966278450490[4] = -state[8];
   out_93714966278450490[5] = state[7];
   out_93714966278450490[6] = 0;
   out_93714966278450490[7] = state[5];
   out_93714966278450490[8] = -state[4];
   out_93714966278450490[9] = 0;
   out_93714966278450490[10] = 0;
   out_93714966278450490[11] = 0;
   out_93714966278450490[12] = 1;
   out_93714966278450490[13] = 0;
   out_93714966278450490[14] = 0;
   out_93714966278450490[15] = 1;
   out_93714966278450490[16] = 0;
   out_93714966278450490[17] = 0;
   out_93714966278450490[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_93714966278450490[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_93714966278450490[20] = 0;
   out_93714966278450490[21] = state[8];
   out_93714966278450490[22] = 0;
   out_93714966278450490[23] = -state[6];
   out_93714966278450490[24] = -state[5];
   out_93714966278450490[25] = 0;
   out_93714966278450490[26] = state[3];
   out_93714966278450490[27] = 0;
   out_93714966278450490[28] = 0;
   out_93714966278450490[29] = 0;
   out_93714966278450490[30] = 0;
   out_93714966278450490[31] = 1;
   out_93714966278450490[32] = 0;
   out_93714966278450490[33] = 0;
   out_93714966278450490[34] = 1;
   out_93714966278450490[35] = 0;
   out_93714966278450490[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_93714966278450490[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_93714966278450490[38] = 0;
   out_93714966278450490[39] = -state[7];
   out_93714966278450490[40] = state[6];
   out_93714966278450490[41] = 0;
   out_93714966278450490[42] = state[4];
   out_93714966278450490[43] = -state[3];
   out_93714966278450490[44] = 0;
   out_93714966278450490[45] = 0;
   out_93714966278450490[46] = 0;
   out_93714966278450490[47] = 0;
   out_93714966278450490[48] = 0;
   out_93714966278450490[49] = 0;
   out_93714966278450490[50] = 1;
   out_93714966278450490[51] = 0;
   out_93714966278450490[52] = 0;
   out_93714966278450490[53] = 1;
}
void h_13(double *state, double *unused, double *out_6528786097031275213) {
   out_6528786097031275213[0] = state[3];
   out_6528786097031275213[1] = state[4];
   out_6528786097031275213[2] = state[5];
}
void H_13(double *state, double *unused, double *out_111452882829742650) {
   out_111452882829742650[0] = 0;
   out_111452882829742650[1] = 0;
   out_111452882829742650[2] = 0;
   out_111452882829742650[3] = 1;
   out_111452882829742650[4] = 0;
   out_111452882829742650[5] = 0;
   out_111452882829742650[6] = 0;
   out_111452882829742650[7] = 0;
   out_111452882829742650[8] = 0;
   out_111452882829742650[9] = 0;
   out_111452882829742650[10] = 0;
   out_111452882829742650[11] = 0;
   out_111452882829742650[12] = 0;
   out_111452882829742650[13] = 0;
   out_111452882829742650[14] = 0;
   out_111452882829742650[15] = 0;
   out_111452882829742650[16] = 0;
   out_111452882829742650[17] = 0;
   out_111452882829742650[18] = 0;
   out_111452882829742650[19] = 0;
   out_111452882829742650[20] = 0;
   out_111452882829742650[21] = 0;
   out_111452882829742650[22] = 1;
   out_111452882829742650[23] = 0;
   out_111452882829742650[24] = 0;
   out_111452882829742650[25] = 0;
   out_111452882829742650[26] = 0;
   out_111452882829742650[27] = 0;
   out_111452882829742650[28] = 0;
   out_111452882829742650[29] = 0;
   out_111452882829742650[30] = 0;
   out_111452882829742650[31] = 0;
   out_111452882829742650[32] = 0;
   out_111452882829742650[33] = 0;
   out_111452882829742650[34] = 0;
   out_111452882829742650[35] = 0;
   out_111452882829742650[36] = 0;
   out_111452882829742650[37] = 0;
   out_111452882829742650[38] = 0;
   out_111452882829742650[39] = 0;
   out_111452882829742650[40] = 0;
   out_111452882829742650[41] = 1;
   out_111452882829742650[42] = 0;
   out_111452882829742650[43] = 0;
   out_111452882829742650[44] = 0;
   out_111452882829742650[45] = 0;
   out_111452882829742650[46] = 0;
   out_111452882829742650[47] = 0;
   out_111452882829742650[48] = 0;
   out_111452882829742650[49] = 0;
   out_111452882829742650[50] = 0;
   out_111452882829742650[51] = 0;
   out_111452882829742650[52] = 0;
   out_111452882829742650[53] = 0;
}
void h_14(double *state, double *unused, double *out_4619261727091989870) {
   out_4619261727091989870[0] = state[6];
   out_4619261727091989870[1] = state[7];
   out_4619261727091989870[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3758843234806959050) {
   out_3758843234806959050[0] = 0;
   out_3758843234806959050[1] = 0;
   out_3758843234806959050[2] = 0;
   out_3758843234806959050[3] = 0;
   out_3758843234806959050[4] = 0;
   out_3758843234806959050[5] = 0;
   out_3758843234806959050[6] = 1;
   out_3758843234806959050[7] = 0;
   out_3758843234806959050[8] = 0;
   out_3758843234806959050[9] = 0;
   out_3758843234806959050[10] = 0;
   out_3758843234806959050[11] = 0;
   out_3758843234806959050[12] = 0;
   out_3758843234806959050[13] = 0;
   out_3758843234806959050[14] = 0;
   out_3758843234806959050[15] = 0;
   out_3758843234806959050[16] = 0;
   out_3758843234806959050[17] = 0;
   out_3758843234806959050[18] = 0;
   out_3758843234806959050[19] = 0;
   out_3758843234806959050[20] = 0;
   out_3758843234806959050[21] = 0;
   out_3758843234806959050[22] = 0;
   out_3758843234806959050[23] = 0;
   out_3758843234806959050[24] = 0;
   out_3758843234806959050[25] = 1;
   out_3758843234806959050[26] = 0;
   out_3758843234806959050[27] = 0;
   out_3758843234806959050[28] = 0;
   out_3758843234806959050[29] = 0;
   out_3758843234806959050[30] = 0;
   out_3758843234806959050[31] = 0;
   out_3758843234806959050[32] = 0;
   out_3758843234806959050[33] = 0;
   out_3758843234806959050[34] = 0;
   out_3758843234806959050[35] = 0;
   out_3758843234806959050[36] = 0;
   out_3758843234806959050[37] = 0;
   out_3758843234806959050[38] = 0;
   out_3758843234806959050[39] = 0;
   out_3758843234806959050[40] = 0;
   out_3758843234806959050[41] = 0;
   out_3758843234806959050[42] = 0;
   out_3758843234806959050[43] = 0;
   out_3758843234806959050[44] = 1;
   out_3758843234806959050[45] = 0;
   out_3758843234806959050[46] = 0;
   out_3758843234806959050[47] = 0;
   out_3758843234806959050[48] = 0;
   out_3758843234806959050[49] = 0;
   out_3758843234806959050[50] = 0;
   out_3758843234806959050[51] = 0;
   out_3758843234806959050[52] = 0;
   out_3758843234806959050[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_9147823097001100337) {
  err_fun(nom_x, delta_x, out_9147823097001100337);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6976768038486665276) {
  inv_err_fun(nom_x, true_x, out_6976768038486665276);
}
void pose_H_mod_fun(double *state, double *out_2524355098589883270) {
  H_mod_fun(state, out_2524355098589883270);
}
void pose_f_fun(double *state, double dt, double *out_2560858857775655991) {
  f_fun(state,  dt, out_2560858857775655991);
}
void pose_F_fun(double *state, double dt, double *out_6720701162118872845) {
  F_fun(state,  dt, out_6720701162118872845);
}
void pose_h_4(double *state, double *unused, double *out_4511036304417573681) {
  h_4(state, unused, out_4511036304417573681);
}
void pose_H_4(double *state, double *unused, double *out_7722084091146443579) {
  H_4(state, unused, out_7722084091146443579);
}
void pose_h_10(double *state, double *unused, double *out_2196926179427311242) {
  h_10(state, unused, out_2196926179427311242);
}
void pose_H_10(double *state, double *unused, double *out_93714966278450490) {
  H_10(state, unused, out_93714966278450490);
}
void pose_h_13(double *state, double *unused, double *out_6528786097031275213) {
  h_13(state, unused, out_6528786097031275213);
}
void pose_H_13(double *state, double *unused, double *out_111452882829742650) {
  H_13(state, unused, out_111452882829742650);
}
void pose_h_14(double *state, double *unused, double *out_4619261727091989870) {
  h_14(state, unused, out_4619261727091989870);
}
void pose_H_14(double *state, double *unused, double *out_3758843234806959050) {
  H_14(state, unused, out_3758843234806959050);
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
