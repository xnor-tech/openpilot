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
void err_fun(double *nom_x, double *delta_x, double *out_235430937466886576) {
   out_235430937466886576[0] = delta_x[0] + nom_x[0];
   out_235430937466886576[1] = delta_x[1] + nom_x[1];
   out_235430937466886576[2] = delta_x[2] + nom_x[2];
   out_235430937466886576[3] = delta_x[3] + nom_x[3];
   out_235430937466886576[4] = delta_x[4] + nom_x[4];
   out_235430937466886576[5] = delta_x[5] + nom_x[5];
   out_235430937466886576[6] = delta_x[6] + nom_x[6];
   out_235430937466886576[7] = delta_x[7] + nom_x[7];
   out_235430937466886576[8] = delta_x[8] + nom_x[8];
   out_235430937466886576[9] = delta_x[9] + nom_x[9];
   out_235430937466886576[10] = delta_x[10] + nom_x[10];
   out_235430937466886576[11] = delta_x[11] + nom_x[11];
   out_235430937466886576[12] = delta_x[12] + nom_x[12];
   out_235430937466886576[13] = delta_x[13] + nom_x[13];
   out_235430937466886576[14] = delta_x[14] + nom_x[14];
   out_235430937466886576[15] = delta_x[15] + nom_x[15];
   out_235430937466886576[16] = delta_x[16] + nom_x[16];
   out_235430937466886576[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2450675293636270791) {
   out_2450675293636270791[0] = -nom_x[0] + true_x[0];
   out_2450675293636270791[1] = -nom_x[1] + true_x[1];
   out_2450675293636270791[2] = -nom_x[2] + true_x[2];
   out_2450675293636270791[3] = -nom_x[3] + true_x[3];
   out_2450675293636270791[4] = -nom_x[4] + true_x[4];
   out_2450675293636270791[5] = -nom_x[5] + true_x[5];
   out_2450675293636270791[6] = -nom_x[6] + true_x[6];
   out_2450675293636270791[7] = -nom_x[7] + true_x[7];
   out_2450675293636270791[8] = -nom_x[8] + true_x[8];
   out_2450675293636270791[9] = -nom_x[9] + true_x[9];
   out_2450675293636270791[10] = -nom_x[10] + true_x[10];
   out_2450675293636270791[11] = -nom_x[11] + true_x[11];
   out_2450675293636270791[12] = -nom_x[12] + true_x[12];
   out_2450675293636270791[13] = -nom_x[13] + true_x[13];
   out_2450675293636270791[14] = -nom_x[14] + true_x[14];
   out_2450675293636270791[15] = -nom_x[15] + true_x[15];
   out_2450675293636270791[16] = -nom_x[16] + true_x[16];
   out_2450675293636270791[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_530647733167696860) {
   out_530647733167696860[0] = 1.0;
   out_530647733167696860[1] = 0.0;
   out_530647733167696860[2] = 0.0;
   out_530647733167696860[3] = 0.0;
   out_530647733167696860[4] = 0.0;
   out_530647733167696860[5] = 0.0;
   out_530647733167696860[6] = 0.0;
   out_530647733167696860[7] = 0.0;
   out_530647733167696860[8] = 0.0;
   out_530647733167696860[9] = 0.0;
   out_530647733167696860[10] = 0.0;
   out_530647733167696860[11] = 0.0;
   out_530647733167696860[12] = 0.0;
   out_530647733167696860[13] = 0.0;
   out_530647733167696860[14] = 0.0;
   out_530647733167696860[15] = 0.0;
   out_530647733167696860[16] = 0.0;
   out_530647733167696860[17] = 0.0;
   out_530647733167696860[18] = 0.0;
   out_530647733167696860[19] = 1.0;
   out_530647733167696860[20] = 0.0;
   out_530647733167696860[21] = 0.0;
   out_530647733167696860[22] = 0.0;
   out_530647733167696860[23] = 0.0;
   out_530647733167696860[24] = 0.0;
   out_530647733167696860[25] = 0.0;
   out_530647733167696860[26] = 0.0;
   out_530647733167696860[27] = 0.0;
   out_530647733167696860[28] = 0.0;
   out_530647733167696860[29] = 0.0;
   out_530647733167696860[30] = 0.0;
   out_530647733167696860[31] = 0.0;
   out_530647733167696860[32] = 0.0;
   out_530647733167696860[33] = 0.0;
   out_530647733167696860[34] = 0.0;
   out_530647733167696860[35] = 0.0;
   out_530647733167696860[36] = 0.0;
   out_530647733167696860[37] = 0.0;
   out_530647733167696860[38] = 1.0;
   out_530647733167696860[39] = 0.0;
   out_530647733167696860[40] = 0.0;
   out_530647733167696860[41] = 0.0;
   out_530647733167696860[42] = 0.0;
   out_530647733167696860[43] = 0.0;
   out_530647733167696860[44] = 0.0;
   out_530647733167696860[45] = 0.0;
   out_530647733167696860[46] = 0.0;
   out_530647733167696860[47] = 0.0;
   out_530647733167696860[48] = 0.0;
   out_530647733167696860[49] = 0.0;
   out_530647733167696860[50] = 0.0;
   out_530647733167696860[51] = 0.0;
   out_530647733167696860[52] = 0.0;
   out_530647733167696860[53] = 0.0;
   out_530647733167696860[54] = 0.0;
   out_530647733167696860[55] = 0.0;
   out_530647733167696860[56] = 0.0;
   out_530647733167696860[57] = 1.0;
   out_530647733167696860[58] = 0.0;
   out_530647733167696860[59] = 0.0;
   out_530647733167696860[60] = 0.0;
   out_530647733167696860[61] = 0.0;
   out_530647733167696860[62] = 0.0;
   out_530647733167696860[63] = 0.0;
   out_530647733167696860[64] = 0.0;
   out_530647733167696860[65] = 0.0;
   out_530647733167696860[66] = 0.0;
   out_530647733167696860[67] = 0.0;
   out_530647733167696860[68] = 0.0;
   out_530647733167696860[69] = 0.0;
   out_530647733167696860[70] = 0.0;
   out_530647733167696860[71] = 0.0;
   out_530647733167696860[72] = 0.0;
   out_530647733167696860[73] = 0.0;
   out_530647733167696860[74] = 0.0;
   out_530647733167696860[75] = 0.0;
   out_530647733167696860[76] = 1.0;
   out_530647733167696860[77] = 0.0;
   out_530647733167696860[78] = 0.0;
   out_530647733167696860[79] = 0.0;
   out_530647733167696860[80] = 0.0;
   out_530647733167696860[81] = 0.0;
   out_530647733167696860[82] = 0.0;
   out_530647733167696860[83] = 0.0;
   out_530647733167696860[84] = 0.0;
   out_530647733167696860[85] = 0.0;
   out_530647733167696860[86] = 0.0;
   out_530647733167696860[87] = 0.0;
   out_530647733167696860[88] = 0.0;
   out_530647733167696860[89] = 0.0;
   out_530647733167696860[90] = 0.0;
   out_530647733167696860[91] = 0.0;
   out_530647733167696860[92] = 0.0;
   out_530647733167696860[93] = 0.0;
   out_530647733167696860[94] = 0.0;
   out_530647733167696860[95] = 1.0;
   out_530647733167696860[96] = 0.0;
   out_530647733167696860[97] = 0.0;
   out_530647733167696860[98] = 0.0;
   out_530647733167696860[99] = 0.0;
   out_530647733167696860[100] = 0.0;
   out_530647733167696860[101] = 0.0;
   out_530647733167696860[102] = 0.0;
   out_530647733167696860[103] = 0.0;
   out_530647733167696860[104] = 0.0;
   out_530647733167696860[105] = 0.0;
   out_530647733167696860[106] = 0.0;
   out_530647733167696860[107] = 0.0;
   out_530647733167696860[108] = 0.0;
   out_530647733167696860[109] = 0.0;
   out_530647733167696860[110] = 0.0;
   out_530647733167696860[111] = 0.0;
   out_530647733167696860[112] = 0.0;
   out_530647733167696860[113] = 0.0;
   out_530647733167696860[114] = 1.0;
   out_530647733167696860[115] = 0.0;
   out_530647733167696860[116] = 0.0;
   out_530647733167696860[117] = 0.0;
   out_530647733167696860[118] = 0.0;
   out_530647733167696860[119] = 0.0;
   out_530647733167696860[120] = 0.0;
   out_530647733167696860[121] = 0.0;
   out_530647733167696860[122] = 0.0;
   out_530647733167696860[123] = 0.0;
   out_530647733167696860[124] = 0.0;
   out_530647733167696860[125] = 0.0;
   out_530647733167696860[126] = 0.0;
   out_530647733167696860[127] = 0.0;
   out_530647733167696860[128] = 0.0;
   out_530647733167696860[129] = 0.0;
   out_530647733167696860[130] = 0.0;
   out_530647733167696860[131] = 0.0;
   out_530647733167696860[132] = 0.0;
   out_530647733167696860[133] = 1.0;
   out_530647733167696860[134] = 0.0;
   out_530647733167696860[135] = 0.0;
   out_530647733167696860[136] = 0.0;
   out_530647733167696860[137] = 0.0;
   out_530647733167696860[138] = 0.0;
   out_530647733167696860[139] = 0.0;
   out_530647733167696860[140] = 0.0;
   out_530647733167696860[141] = 0.0;
   out_530647733167696860[142] = 0.0;
   out_530647733167696860[143] = 0.0;
   out_530647733167696860[144] = 0.0;
   out_530647733167696860[145] = 0.0;
   out_530647733167696860[146] = 0.0;
   out_530647733167696860[147] = 0.0;
   out_530647733167696860[148] = 0.0;
   out_530647733167696860[149] = 0.0;
   out_530647733167696860[150] = 0.0;
   out_530647733167696860[151] = 0.0;
   out_530647733167696860[152] = 1.0;
   out_530647733167696860[153] = 0.0;
   out_530647733167696860[154] = 0.0;
   out_530647733167696860[155] = 0.0;
   out_530647733167696860[156] = 0.0;
   out_530647733167696860[157] = 0.0;
   out_530647733167696860[158] = 0.0;
   out_530647733167696860[159] = 0.0;
   out_530647733167696860[160] = 0.0;
   out_530647733167696860[161] = 0.0;
   out_530647733167696860[162] = 0.0;
   out_530647733167696860[163] = 0.0;
   out_530647733167696860[164] = 0.0;
   out_530647733167696860[165] = 0.0;
   out_530647733167696860[166] = 0.0;
   out_530647733167696860[167] = 0.0;
   out_530647733167696860[168] = 0.0;
   out_530647733167696860[169] = 0.0;
   out_530647733167696860[170] = 0.0;
   out_530647733167696860[171] = 1.0;
   out_530647733167696860[172] = 0.0;
   out_530647733167696860[173] = 0.0;
   out_530647733167696860[174] = 0.0;
   out_530647733167696860[175] = 0.0;
   out_530647733167696860[176] = 0.0;
   out_530647733167696860[177] = 0.0;
   out_530647733167696860[178] = 0.0;
   out_530647733167696860[179] = 0.0;
   out_530647733167696860[180] = 0.0;
   out_530647733167696860[181] = 0.0;
   out_530647733167696860[182] = 0.0;
   out_530647733167696860[183] = 0.0;
   out_530647733167696860[184] = 0.0;
   out_530647733167696860[185] = 0.0;
   out_530647733167696860[186] = 0.0;
   out_530647733167696860[187] = 0.0;
   out_530647733167696860[188] = 0.0;
   out_530647733167696860[189] = 0.0;
   out_530647733167696860[190] = 1.0;
   out_530647733167696860[191] = 0.0;
   out_530647733167696860[192] = 0.0;
   out_530647733167696860[193] = 0.0;
   out_530647733167696860[194] = 0.0;
   out_530647733167696860[195] = 0.0;
   out_530647733167696860[196] = 0.0;
   out_530647733167696860[197] = 0.0;
   out_530647733167696860[198] = 0.0;
   out_530647733167696860[199] = 0.0;
   out_530647733167696860[200] = 0.0;
   out_530647733167696860[201] = 0.0;
   out_530647733167696860[202] = 0.0;
   out_530647733167696860[203] = 0.0;
   out_530647733167696860[204] = 0.0;
   out_530647733167696860[205] = 0.0;
   out_530647733167696860[206] = 0.0;
   out_530647733167696860[207] = 0.0;
   out_530647733167696860[208] = 0.0;
   out_530647733167696860[209] = 1.0;
   out_530647733167696860[210] = 0.0;
   out_530647733167696860[211] = 0.0;
   out_530647733167696860[212] = 0.0;
   out_530647733167696860[213] = 0.0;
   out_530647733167696860[214] = 0.0;
   out_530647733167696860[215] = 0.0;
   out_530647733167696860[216] = 0.0;
   out_530647733167696860[217] = 0.0;
   out_530647733167696860[218] = 0.0;
   out_530647733167696860[219] = 0.0;
   out_530647733167696860[220] = 0.0;
   out_530647733167696860[221] = 0.0;
   out_530647733167696860[222] = 0.0;
   out_530647733167696860[223] = 0.0;
   out_530647733167696860[224] = 0.0;
   out_530647733167696860[225] = 0.0;
   out_530647733167696860[226] = 0.0;
   out_530647733167696860[227] = 0.0;
   out_530647733167696860[228] = 1.0;
   out_530647733167696860[229] = 0.0;
   out_530647733167696860[230] = 0.0;
   out_530647733167696860[231] = 0.0;
   out_530647733167696860[232] = 0.0;
   out_530647733167696860[233] = 0.0;
   out_530647733167696860[234] = 0.0;
   out_530647733167696860[235] = 0.0;
   out_530647733167696860[236] = 0.0;
   out_530647733167696860[237] = 0.0;
   out_530647733167696860[238] = 0.0;
   out_530647733167696860[239] = 0.0;
   out_530647733167696860[240] = 0.0;
   out_530647733167696860[241] = 0.0;
   out_530647733167696860[242] = 0.0;
   out_530647733167696860[243] = 0.0;
   out_530647733167696860[244] = 0.0;
   out_530647733167696860[245] = 0.0;
   out_530647733167696860[246] = 0.0;
   out_530647733167696860[247] = 1.0;
   out_530647733167696860[248] = 0.0;
   out_530647733167696860[249] = 0.0;
   out_530647733167696860[250] = 0.0;
   out_530647733167696860[251] = 0.0;
   out_530647733167696860[252] = 0.0;
   out_530647733167696860[253] = 0.0;
   out_530647733167696860[254] = 0.0;
   out_530647733167696860[255] = 0.0;
   out_530647733167696860[256] = 0.0;
   out_530647733167696860[257] = 0.0;
   out_530647733167696860[258] = 0.0;
   out_530647733167696860[259] = 0.0;
   out_530647733167696860[260] = 0.0;
   out_530647733167696860[261] = 0.0;
   out_530647733167696860[262] = 0.0;
   out_530647733167696860[263] = 0.0;
   out_530647733167696860[264] = 0.0;
   out_530647733167696860[265] = 0.0;
   out_530647733167696860[266] = 1.0;
   out_530647733167696860[267] = 0.0;
   out_530647733167696860[268] = 0.0;
   out_530647733167696860[269] = 0.0;
   out_530647733167696860[270] = 0.0;
   out_530647733167696860[271] = 0.0;
   out_530647733167696860[272] = 0.0;
   out_530647733167696860[273] = 0.0;
   out_530647733167696860[274] = 0.0;
   out_530647733167696860[275] = 0.0;
   out_530647733167696860[276] = 0.0;
   out_530647733167696860[277] = 0.0;
   out_530647733167696860[278] = 0.0;
   out_530647733167696860[279] = 0.0;
   out_530647733167696860[280] = 0.0;
   out_530647733167696860[281] = 0.0;
   out_530647733167696860[282] = 0.0;
   out_530647733167696860[283] = 0.0;
   out_530647733167696860[284] = 0.0;
   out_530647733167696860[285] = 1.0;
   out_530647733167696860[286] = 0.0;
   out_530647733167696860[287] = 0.0;
   out_530647733167696860[288] = 0.0;
   out_530647733167696860[289] = 0.0;
   out_530647733167696860[290] = 0.0;
   out_530647733167696860[291] = 0.0;
   out_530647733167696860[292] = 0.0;
   out_530647733167696860[293] = 0.0;
   out_530647733167696860[294] = 0.0;
   out_530647733167696860[295] = 0.0;
   out_530647733167696860[296] = 0.0;
   out_530647733167696860[297] = 0.0;
   out_530647733167696860[298] = 0.0;
   out_530647733167696860[299] = 0.0;
   out_530647733167696860[300] = 0.0;
   out_530647733167696860[301] = 0.0;
   out_530647733167696860[302] = 0.0;
   out_530647733167696860[303] = 0.0;
   out_530647733167696860[304] = 1.0;
   out_530647733167696860[305] = 0.0;
   out_530647733167696860[306] = 0.0;
   out_530647733167696860[307] = 0.0;
   out_530647733167696860[308] = 0.0;
   out_530647733167696860[309] = 0.0;
   out_530647733167696860[310] = 0.0;
   out_530647733167696860[311] = 0.0;
   out_530647733167696860[312] = 0.0;
   out_530647733167696860[313] = 0.0;
   out_530647733167696860[314] = 0.0;
   out_530647733167696860[315] = 0.0;
   out_530647733167696860[316] = 0.0;
   out_530647733167696860[317] = 0.0;
   out_530647733167696860[318] = 0.0;
   out_530647733167696860[319] = 0.0;
   out_530647733167696860[320] = 0.0;
   out_530647733167696860[321] = 0.0;
   out_530647733167696860[322] = 0.0;
   out_530647733167696860[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_9203828947486336506) {
   out_9203828947486336506[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_9203828947486336506[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_9203828947486336506[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_9203828947486336506[3] = dt*state[12] + state[3];
   out_9203828947486336506[4] = dt*state[13] + state[4];
   out_9203828947486336506[5] = dt*state[14] + state[5];
   out_9203828947486336506[6] = state[6];
   out_9203828947486336506[7] = state[7];
   out_9203828947486336506[8] = state[8];
   out_9203828947486336506[9] = state[9];
   out_9203828947486336506[10] = state[10];
   out_9203828947486336506[11] = state[11];
   out_9203828947486336506[12] = state[12];
   out_9203828947486336506[13] = state[13];
   out_9203828947486336506[14] = state[14];
   out_9203828947486336506[15] = state[15];
   out_9203828947486336506[16] = state[16];
   out_9203828947486336506[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4423182636226594510) {
   out_4423182636226594510[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4423182636226594510[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4423182636226594510[2] = 0;
   out_4423182636226594510[3] = 0;
   out_4423182636226594510[4] = 0;
   out_4423182636226594510[5] = 0;
   out_4423182636226594510[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4423182636226594510[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4423182636226594510[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4423182636226594510[9] = 0;
   out_4423182636226594510[10] = 0;
   out_4423182636226594510[11] = 0;
   out_4423182636226594510[12] = 0;
   out_4423182636226594510[13] = 0;
   out_4423182636226594510[14] = 0;
   out_4423182636226594510[15] = 0;
   out_4423182636226594510[16] = 0;
   out_4423182636226594510[17] = 0;
   out_4423182636226594510[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4423182636226594510[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4423182636226594510[20] = 0;
   out_4423182636226594510[21] = 0;
   out_4423182636226594510[22] = 0;
   out_4423182636226594510[23] = 0;
   out_4423182636226594510[24] = 0;
   out_4423182636226594510[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4423182636226594510[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4423182636226594510[27] = 0;
   out_4423182636226594510[28] = 0;
   out_4423182636226594510[29] = 0;
   out_4423182636226594510[30] = 0;
   out_4423182636226594510[31] = 0;
   out_4423182636226594510[32] = 0;
   out_4423182636226594510[33] = 0;
   out_4423182636226594510[34] = 0;
   out_4423182636226594510[35] = 0;
   out_4423182636226594510[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4423182636226594510[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4423182636226594510[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4423182636226594510[39] = 0;
   out_4423182636226594510[40] = 0;
   out_4423182636226594510[41] = 0;
   out_4423182636226594510[42] = 0;
   out_4423182636226594510[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4423182636226594510[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4423182636226594510[45] = 0;
   out_4423182636226594510[46] = 0;
   out_4423182636226594510[47] = 0;
   out_4423182636226594510[48] = 0;
   out_4423182636226594510[49] = 0;
   out_4423182636226594510[50] = 0;
   out_4423182636226594510[51] = 0;
   out_4423182636226594510[52] = 0;
   out_4423182636226594510[53] = 0;
   out_4423182636226594510[54] = 0;
   out_4423182636226594510[55] = 0;
   out_4423182636226594510[56] = 0;
   out_4423182636226594510[57] = 1;
   out_4423182636226594510[58] = 0;
   out_4423182636226594510[59] = 0;
   out_4423182636226594510[60] = 0;
   out_4423182636226594510[61] = 0;
   out_4423182636226594510[62] = 0;
   out_4423182636226594510[63] = 0;
   out_4423182636226594510[64] = 0;
   out_4423182636226594510[65] = 0;
   out_4423182636226594510[66] = dt;
   out_4423182636226594510[67] = 0;
   out_4423182636226594510[68] = 0;
   out_4423182636226594510[69] = 0;
   out_4423182636226594510[70] = 0;
   out_4423182636226594510[71] = 0;
   out_4423182636226594510[72] = 0;
   out_4423182636226594510[73] = 0;
   out_4423182636226594510[74] = 0;
   out_4423182636226594510[75] = 0;
   out_4423182636226594510[76] = 1;
   out_4423182636226594510[77] = 0;
   out_4423182636226594510[78] = 0;
   out_4423182636226594510[79] = 0;
   out_4423182636226594510[80] = 0;
   out_4423182636226594510[81] = 0;
   out_4423182636226594510[82] = 0;
   out_4423182636226594510[83] = 0;
   out_4423182636226594510[84] = 0;
   out_4423182636226594510[85] = dt;
   out_4423182636226594510[86] = 0;
   out_4423182636226594510[87] = 0;
   out_4423182636226594510[88] = 0;
   out_4423182636226594510[89] = 0;
   out_4423182636226594510[90] = 0;
   out_4423182636226594510[91] = 0;
   out_4423182636226594510[92] = 0;
   out_4423182636226594510[93] = 0;
   out_4423182636226594510[94] = 0;
   out_4423182636226594510[95] = 1;
   out_4423182636226594510[96] = 0;
   out_4423182636226594510[97] = 0;
   out_4423182636226594510[98] = 0;
   out_4423182636226594510[99] = 0;
   out_4423182636226594510[100] = 0;
   out_4423182636226594510[101] = 0;
   out_4423182636226594510[102] = 0;
   out_4423182636226594510[103] = 0;
   out_4423182636226594510[104] = dt;
   out_4423182636226594510[105] = 0;
   out_4423182636226594510[106] = 0;
   out_4423182636226594510[107] = 0;
   out_4423182636226594510[108] = 0;
   out_4423182636226594510[109] = 0;
   out_4423182636226594510[110] = 0;
   out_4423182636226594510[111] = 0;
   out_4423182636226594510[112] = 0;
   out_4423182636226594510[113] = 0;
   out_4423182636226594510[114] = 1;
   out_4423182636226594510[115] = 0;
   out_4423182636226594510[116] = 0;
   out_4423182636226594510[117] = 0;
   out_4423182636226594510[118] = 0;
   out_4423182636226594510[119] = 0;
   out_4423182636226594510[120] = 0;
   out_4423182636226594510[121] = 0;
   out_4423182636226594510[122] = 0;
   out_4423182636226594510[123] = 0;
   out_4423182636226594510[124] = 0;
   out_4423182636226594510[125] = 0;
   out_4423182636226594510[126] = 0;
   out_4423182636226594510[127] = 0;
   out_4423182636226594510[128] = 0;
   out_4423182636226594510[129] = 0;
   out_4423182636226594510[130] = 0;
   out_4423182636226594510[131] = 0;
   out_4423182636226594510[132] = 0;
   out_4423182636226594510[133] = 1;
   out_4423182636226594510[134] = 0;
   out_4423182636226594510[135] = 0;
   out_4423182636226594510[136] = 0;
   out_4423182636226594510[137] = 0;
   out_4423182636226594510[138] = 0;
   out_4423182636226594510[139] = 0;
   out_4423182636226594510[140] = 0;
   out_4423182636226594510[141] = 0;
   out_4423182636226594510[142] = 0;
   out_4423182636226594510[143] = 0;
   out_4423182636226594510[144] = 0;
   out_4423182636226594510[145] = 0;
   out_4423182636226594510[146] = 0;
   out_4423182636226594510[147] = 0;
   out_4423182636226594510[148] = 0;
   out_4423182636226594510[149] = 0;
   out_4423182636226594510[150] = 0;
   out_4423182636226594510[151] = 0;
   out_4423182636226594510[152] = 1;
   out_4423182636226594510[153] = 0;
   out_4423182636226594510[154] = 0;
   out_4423182636226594510[155] = 0;
   out_4423182636226594510[156] = 0;
   out_4423182636226594510[157] = 0;
   out_4423182636226594510[158] = 0;
   out_4423182636226594510[159] = 0;
   out_4423182636226594510[160] = 0;
   out_4423182636226594510[161] = 0;
   out_4423182636226594510[162] = 0;
   out_4423182636226594510[163] = 0;
   out_4423182636226594510[164] = 0;
   out_4423182636226594510[165] = 0;
   out_4423182636226594510[166] = 0;
   out_4423182636226594510[167] = 0;
   out_4423182636226594510[168] = 0;
   out_4423182636226594510[169] = 0;
   out_4423182636226594510[170] = 0;
   out_4423182636226594510[171] = 1;
   out_4423182636226594510[172] = 0;
   out_4423182636226594510[173] = 0;
   out_4423182636226594510[174] = 0;
   out_4423182636226594510[175] = 0;
   out_4423182636226594510[176] = 0;
   out_4423182636226594510[177] = 0;
   out_4423182636226594510[178] = 0;
   out_4423182636226594510[179] = 0;
   out_4423182636226594510[180] = 0;
   out_4423182636226594510[181] = 0;
   out_4423182636226594510[182] = 0;
   out_4423182636226594510[183] = 0;
   out_4423182636226594510[184] = 0;
   out_4423182636226594510[185] = 0;
   out_4423182636226594510[186] = 0;
   out_4423182636226594510[187] = 0;
   out_4423182636226594510[188] = 0;
   out_4423182636226594510[189] = 0;
   out_4423182636226594510[190] = 1;
   out_4423182636226594510[191] = 0;
   out_4423182636226594510[192] = 0;
   out_4423182636226594510[193] = 0;
   out_4423182636226594510[194] = 0;
   out_4423182636226594510[195] = 0;
   out_4423182636226594510[196] = 0;
   out_4423182636226594510[197] = 0;
   out_4423182636226594510[198] = 0;
   out_4423182636226594510[199] = 0;
   out_4423182636226594510[200] = 0;
   out_4423182636226594510[201] = 0;
   out_4423182636226594510[202] = 0;
   out_4423182636226594510[203] = 0;
   out_4423182636226594510[204] = 0;
   out_4423182636226594510[205] = 0;
   out_4423182636226594510[206] = 0;
   out_4423182636226594510[207] = 0;
   out_4423182636226594510[208] = 0;
   out_4423182636226594510[209] = 1;
   out_4423182636226594510[210] = 0;
   out_4423182636226594510[211] = 0;
   out_4423182636226594510[212] = 0;
   out_4423182636226594510[213] = 0;
   out_4423182636226594510[214] = 0;
   out_4423182636226594510[215] = 0;
   out_4423182636226594510[216] = 0;
   out_4423182636226594510[217] = 0;
   out_4423182636226594510[218] = 0;
   out_4423182636226594510[219] = 0;
   out_4423182636226594510[220] = 0;
   out_4423182636226594510[221] = 0;
   out_4423182636226594510[222] = 0;
   out_4423182636226594510[223] = 0;
   out_4423182636226594510[224] = 0;
   out_4423182636226594510[225] = 0;
   out_4423182636226594510[226] = 0;
   out_4423182636226594510[227] = 0;
   out_4423182636226594510[228] = 1;
   out_4423182636226594510[229] = 0;
   out_4423182636226594510[230] = 0;
   out_4423182636226594510[231] = 0;
   out_4423182636226594510[232] = 0;
   out_4423182636226594510[233] = 0;
   out_4423182636226594510[234] = 0;
   out_4423182636226594510[235] = 0;
   out_4423182636226594510[236] = 0;
   out_4423182636226594510[237] = 0;
   out_4423182636226594510[238] = 0;
   out_4423182636226594510[239] = 0;
   out_4423182636226594510[240] = 0;
   out_4423182636226594510[241] = 0;
   out_4423182636226594510[242] = 0;
   out_4423182636226594510[243] = 0;
   out_4423182636226594510[244] = 0;
   out_4423182636226594510[245] = 0;
   out_4423182636226594510[246] = 0;
   out_4423182636226594510[247] = 1;
   out_4423182636226594510[248] = 0;
   out_4423182636226594510[249] = 0;
   out_4423182636226594510[250] = 0;
   out_4423182636226594510[251] = 0;
   out_4423182636226594510[252] = 0;
   out_4423182636226594510[253] = 0;
   out_4423182636226594510[254] = 0;
   out_4423182636226594510[255] = 0;
   out_4423182636226594510[256] = 0;
   out_4423182636226594510[257] = 0;
   out_4423182636226594510[258] = 0;
   out_4423182636226594510[259] = 0;
   out_4423182636226594510[260] = 0;
   out_4423182636226594510[261] = 0;
   out_4423182636226594510[262] = 0;
   out_4423182636226594510[263] = 0;
   out_4423182636226594510[264] = 0;
   out_4423182636226594510[265] = 0;
   out_4423182636226594510[266] = 1;
   out_4423182636226594510[267] = 0;
   out_4423182636226594510[268] = 0;
   out_4423182636226594510[269] = 0;
   out_4423182636226594510[270] = 0;
   out_4423182636226594510[271] = 0;
   out_4423182636226594510[272] = 0;
   out_4423182636226594510[273] = 0;
   out_4423182636226594510[274] = 0;
   out_4423182636226594510[275] = 0;
   out_4423182636226594510[276] = 0;
   out_4423182636226594510[277] = 0;
   out_4423182636226594510[278] = 0;
   out_4423182636226594510[279] = 0;
   out_4423182636226594510[280] = 0;
   out_4423182636226594510[281] = 0;
   out_4423182636226594510[282] = 0;
   out_4423182636226594510[283] = 0;
   out_4423182636226594510[284] = 0;
   out_4423182636226594510[285] = 1;
   out_4423182636226594510[286] = 0;
   out_4423182636226594510[287] = 0;
   out_4423182636226594510[288] = 0;
   out_4423182636226594510[289] = 0;
   out_4423182636226594510[290] = 0;
   out_4423182636226594510[291] = 0;
   out_4423182636226594510[292] = 0;
   out_4423182636226594510[293] = 0;
   out_4423182636226594510[294] = 0;
   out_4423182636226594510[295] = 0;
   out_4423182636226594510[296] = 0;
   out_4423182636226594510[297] = 0;
   out_4423182636226594510[298] = 0;
   out_4423182636226594510[299] = 0;
   out_4423182636226594510[300] = 0;
   out_4423182636226594510[301] = 0;
   out_4423182636226594510[302] = 0;
   out_4423182636226594510[303] = 0;
   out_4423182636226594510[304] = 1;
   out_4423182636226594510[305] = 0;
   out_4423182636226594510[306] = 0;
   out_4423182636226594510[307] = 0;
   out_4423182636226594510[308] = 0;
   out_4423182636226594510[309] = 0;
   out_4423182636226594510[310] = 0;
   out_4423182636226594510[311] = 0;
   out_4423182636226594510[312] = 0;
   out_4423182636226594510[313] = 0;
   out_4423182636226594510[314] = 0;
   out_4423182636226594510[315] = 0;
   out_4423182636226594510[316] = 0;
   out_4423182636226594510[317] = 0;
   out_4423182636226594510[318] = 0;
   out_4423182636226594510[319] = 0;
   out_4423182636226594510[320] = 0;
   out_4423182636226594510[321] = 0;
   out_4423182636226594510[322] = 0;
   out_4423182636226594510[323] = 1;
}
void h_4(double *state, double *unused, double *out_4884014442507138413) {
   out_4884014442507138413[0] = state[6] + state[9];
   out_4884014442507138413[1] = state[7] + state[10];
   out_4884014442507138413[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2350346459073993721) {
   out_2350346459073993721[0] = 0;
   out_2350346459073993721[1] = 0;
   out_2350346459073993721[2] = 0;
   out_2350346459073993721[3] = 0;
   out_2350346459073993721[4] = 0;
   out_2350346459073993721[5] = 0;
   out_2350346459073993721[6] = 1;
   out_2350346459073993721[7] = 0;
   out_2350346459073993721[8] = 0;
   out_2350346459073993721[9] = 1;
   out_2350346459073993721[10] = 0;
   out_2350346459073993721[11] = 0;
   out_2350346459073993721[12] = 0;
   out_2350346459073993721[13] = 0;
   out_2350346459073993721[14] = 0;
   out_2350346459073993721[15] = 0;
   out_2350346459073993721[16] = 0;
   out_2350346459073993721[17] = 0;
   out_2350346459073993721[18] = 0;
   out_2350346459073993721[19] = 0;
   out_2350346459073993721[20] = 0;
   out_2350346459073993721[21] = 0;
   out_2350346459073993721[22] = 0;
   out_2350346459073993721[23] = 0;
   out_2350346459073993721[24] = 0;
   out_2350346459073993721[25] = 1;
   out_2350346459073993721[26] = 0;
   out_2350346459073993721[27] = 0;
   out_2350346459073993721[28] = 1;
   out_2350346459073993721[29] = 0;
   out_2350346459073993721[30] = 0;
   out_2350346459073993721[31] = 0;
   out_2350346459073993721[32] = 0;
   out_2350346459073993721[33] = 0;
   out_2350346459073993721[34] = 0;
   out_2350346459073993721[35] = 0;
   out_2350346459073993721[36] = 0;
   out_2350346459073993721[37] = 0;
   out_2350346459073993721[38] = 0;
   out_2350346459073993721[39] = 0;
   out_2350346459073993721[40] = 0;
   out_2350346459073993721[41] = 0;
   out_2350346459073993721[42] = 0;
   out_2350346459073993721[43] = 0;
   out_2350346459073993721[44] = 1;
   out_2350346459073993721[45] = 0;
   out_2350346459073993721[46] = 0;
   out_2350346459073993721[47] = 1;
   out_2350346459073993721[48] = 0;
   out_2350346459073993721[49] = 0;
   out_2350346459073993721[50] = 0;
   out_2350346459073993721[51] = 0;
   out_2350346459073993721[52] = 0;
   out_2350346459073993721[53] = 0;
}
void h_10(double *state, double *unused, double *out_6338696543292411763) {
   out_6338696543292411763[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6338696543292411763[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6338696543292411763[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2306543916110426020) {
   out_2306543916110426020[0] = 0;
   out_2306543916110426020[1] = 9.8100000000000005*cos(state[1]);
   out_2306543916110426020[2] = 0;
   out_2306543916110426020[3] = 0;
   out_2306543916110426020[4] = -state[8];
   out_2306543916110426020[5] = state[7];
   out_2306543916110426020[6] = 0;
   out_2306543916110426020[7] = state[5];
   out_2306543916110426020[8] = -state[4];
   out_2306543916110426020[9] = 0;
   out_2306543916110426020[10] = 0;
   out_2306543916110426020[11] = 0;
   out_2306543916110426020[12] = 1;
   out_2306543916110426020[13] = 0;
   out_2306543916110426020[14] = 0;
   out_2306543916110426020[15] = 1;
   out_2306543916110426020[16] = 0;
   out_2306543916110426020[17] = 0;
   out_2306543916110426020[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2306543916110426020[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2306543916110426020[20] = 0;
   out_2306543916110426020[21] = state[8];
   out_2306543916110426020[22] = 0;
   out_2306543916110426020[23] = -state[6];
   out_2306543916110426020[24] = -state[5];
   out_2306543916110426020[25] = 0;
   out_2306543916110426020[26] = state[3];
   out_2306543916110426020[27] = 0;
   out_2306543916110426020[28] = 0;
   out_2306543916110426020[29] = 0;
   out_2306543916110426020[30] = 0;
   out_2306543916110426020[31] = 1;
   out_2306543916110426020[32] = 0;
   out_2306543916110426020[33] = 0;
   out_2306543916110426020[34] = 1;
   out_2306543916110426020[35] = 0;
   out_2306543916110426020[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2306543916110426020[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2306543916110426020[38] = 0;
   out_2306543916110426020[39] = -state[7];
   out_2306543916110426020[40] = state[6];
   out_2306543916110426020[41] = 0;
   out_2306543916110426020[42] = state[4];
   out_2306543916110426020[43] = -state[3];
   out_2306543916110426020[44] = 0;
   out_2306543916110426020[45] = 0;
   out_2306543916110426020[46] = 0;
   out_2306543916110426020[47] = 0;
   out_2306543916110426020[48] = 0;
   out_2306543916110426020[49] = 0;
   out_2306543916110426020[50] = 1;
   out_2306543916110426020[51] = 0;
   out_2306543916110426020[52] = 0;
   out_2306543916110426020[53] = 1;
}
void h_13(double *state, double *unused, double *out_8804592808846089421) {
   out_8804592808846089421[0] = state[3];
   out_8804592808846089421[1] = state[4];
   out_8804592808846089421[2] = state[5];
}
void H_13(double *state, double *unused, double *out_861927366258339080) {
   out_861927366258339080[0] = 0;
   out_861927366258339080[1] = 0;
   out_861927366258339080[2] = 0;
   out_861927366258339080[3] = 1;
   out_861927366258339080[4] = 0;
   out_861927366258339080[5] = 0;
   out_861927366258339080[6] = 0;
   out_861927366258339080[7] = 0;
   out_861927366258339080[8] = 0;
   out_861927366258339080[9] = 0;
   out_861927366258339080[10] = 0;
   out_861927366258339080[11] = 0;
   out_861927366258339080[12] = 0;
   out_861927366258339080[13] = 0;
   out_861927366258339080[14] = 0;
   out_861927366258339080[15] = 0;
   out_861927366258339080[16] = 0;
   out_861927366258339080[17] = 0;
   out_861927366258339080[18] = 0;
   out_861927366258339080[19] = 0;
   out_861927366258339080[20] = 0;
   out_861927366258339080[21] = 0;
   out_861927366258339080[22] = 1;
   out_861927366258339080[23] = 0;
   out_861927366258339080[24] = 0;
   out_861927366258339080[25] = 0;
   out_861927366258339080[26] = 0;
   out_861927366258339080[27] = 0;
   out_861927366258339080[28] = 0;
   out_861927366258339080[29] = 0;
   out_861927366258339080[30] = 0;
   out_861927366258339080[31] = 0;
   out_861927366258339080[32] = 0;
   out_861927366258339080[33] = 0;
   out_861927366258339080[34] = 0;
   out_861927366258339080[35] = 0;
   out_861927366258339080[36] = 0;
   out_861927366258339080[37] = 0;
   out_861927366258339080[38] = 0;
   out_861927366258339080[39] = 0;
   out_861927366258339080[40] = 0;
   out_861927366258339080[41] = 1;
   out_861927366258339080[42] = 0;
   out_861927366258339080[43] = 0;
   out_861927366258339080[44] = 0;
   out_861927366258339080[45] = 0;
   out_861927366258339080[46] = 0;
   out_861927366258339080[47] = 0;
   out_861927366258339080[48] = 0;
   out_861927366258339080[49] = 0;
   out_861927366258339080[50] = 0;
   out_861927366258339080[51] = 0;
   out_861927366258339080[52] = 0;
   out_861927366258339080[53] = 0;
}
void h_14(double *state, double *unused, double *out_5486189001180777325) {
   out_5486189001180777325[0] = state[6];
   out_5486189001180777325[1] = state[7];
   out_5486189001180777325[2] = state[8];
}
void H_14(double *state, double *unused, double *out_1612894397265490808) {
   out_1612894397265490808[0] = 0;
   out_1612894397265490808[1] = 0;
   out_1612894397265490808[2] = 0;
   out_1612894397265490808[3] = 0;
   out_1612894397265490808[4] = 0;
   out_1612894397265490808[5] = 0;
   out_1612894397265490808[6] = 1;
   out_1612894397265490808[7] = 0;
   out_1612894397265490808[8] = 0;
   out_1612894397265490808[9] = 0;
   out_1612894397265490808[10] = 0;
   out_1612894397265490808[11] = 0;
   out_1612894397265490808[12] = 0;
   out_1612894397265490808[13] = 0;
   out_1612894397265490808[14] = 0;
   out_1612894397265490808[15] = 0;
   out_1612894397265490808[16] = 0;
   out_1612894397265490808[17] = 0;
   out_1612894397265490808[18] = 0;
   out_1612894397265490808[19] = 0;
   out_1612894397265490808[20] = 0;
   out_1612894397265490808[21] = 0;
   out_1612894397265490808[22] = 0;
   out_1612894397265490808[23] = 0;
   out_1612894397265490808[24] = 0;
   out_1612894397265490808[25] = 1;
   out_1612894397265490808[26] = 0;
   out_1612894397265490808[27] = 0;
   out_1612894397265490808[28] = 0;
   out_1612894397265490808[29] = 0;
   out_1612894397265490808[30] = 0;
   out_1612894397265490808[31] = 0;
   out_1612894397265490808[32] = 0;
   out_1612894397265490808[33] = 0;
   out_1612894397265490808[34] = 0;
   out_1612894397265490808[35] = 0;
   out_1612894397265490808[36] = 0;
   out_1612894397265490808[37] = 0;
   out_1612894397265490808[38] = 0;
   out_1612894397265490808[39] = 0;
   out_1612894397265490808[40] = 0;
   out_1612894397265490808[41] = 0;
   out_1612894397265490808[42] = 0;
   out_1612894397265490808[43] = 0;
   out_1612894397265490808[44] = 1;
   out_1612894397265490808[45] = 0;
   out_1612894397265490808[46] = 0;
   out_1612894397265490808[47] = 0;
   out_1612894397265490808[48] = 0;
   out_1612894397265490808[49] = 0;
   out_1612894397265490808[50] = 0;
   out_1612894397265490808[51] = 0;
   out_1612894397265490808[52] = 0;
   out_1612894397265490808[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_235430937466886576) {
  err_fun(nom_x, delta_x, out_235430937466886576);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2450675293636270791) {
  inv_err_fun(nom_x, true_x, out_2450675293636270791);
}
void pose_H_mod_fun(double *state, double *out_530647733167696860) {
  H_mod_fun(state, out_530647733167696860);
}
void pose_f_fun(double *state, double dt, double *out_9203828947486336506) {
  f_fun(state,  dt, out_9203828947486336506);
}
void pose_F_fun(double *state, double dt, double *out_4423182636226594510) {
  F_fun(state,  dt, out_4423182636226594510);
}
void pose_h_4(double *state, double *unused, double *out_4884014442507138413) {
  h_4(state, unused, out_4884014442507138413);
}
void pose_H_4(double *state, double *unused, double *out_2350346459073993721) {
  H_4(state, unused, out_2350346459073993721);
}
void pose_h_10(double *state, double *unused, double *out_6338696543292411763) {
  h_10(state, unused, out_6338696543292411763);
}
void pose_H_10(double *state, double *unused, double *out_2306543916110426020) {
  H_10(state, unused, out_2306543916110426020);
}
void pose_h_13(double *state, double *unused, double *out_8804592808846089421) {
  h_13(state, unused, out_8804592808846089421);
}
void pose_H_13(double *state, double *unused, double *out_861927366258339080) {
  H_13(state, unused, out_861927366258339080);
}
void pose_h_14(double *state, double *unused, double *out_5486189001180777325) {
  h_14(state, unused, out_5486189001180777325);
}
void pose_H_14(double *state, double *unused, double *out_1612894397265490808) {
  H_14(state, unused, out_1612894397265490808);
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
