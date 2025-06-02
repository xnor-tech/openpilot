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
void err_fun(double *nom_x, double *delta_x, double *out_945005396424872191) {
   out_945005396424872191[0] = delta_x[0] + nom_x[0];
   out_945005396424872191[1] = delta_x[1] + nom_x[1];
   out_945005396424872191[2] = delta_x[2] + nom_x[2];
   out_945005396424872191[3] = delta_x[3] + nom_x[3];
   out_945005396424872191[4] = delta_x[4] + nom_x[4];
   out_945005396424872191[5] = delta_x[5] + nom_x[5];
   out_945005396424872191[6] = delta_x[6] + nom_x[6];
   out_945005396424872191[7] = delta_x[7] + nom_x[7];
   out_945005396424872191[8] = delta_x[8] + nom_x[8];
   out_945005396424872191[9] = delta_x[9] + nom_x[9];
   out_945005396424872191[10] = delta_x[10] + nom_x[10];
   out_945005396424872191[11] = delta_x[11] + nom_x[11];
   out_945005396424872191[12] = delta_x[12] + nom_x[12];
   out_945005396424872191[13] = delta_x[13] + nom_x[13];
   out_945005396424872191[14] = delta_x[14] + nom_x[14];
   out_945005396424872191[15] = delta_x[15] + nom_x[15];
   out_945005396424872191[16] = delta_x[16] + nom_x[16];
   out_945005396424872191[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4622954603925097103) {
   out_4622954603925097103[0] = -nom_x[0] + true_x[0];
   out_4622954603925097103[1] = -nom_x[1] + true_x[1];
   out_4622954603925097103[2] = -nom_x[2] + true_x[2];
   out_4622954603925097103[3] = -nom_x[3] + true_x[3];
   out_4622954603925097103[4] = -nom_x[4] + true_x[4];
   out_4622954603925097103[5] = -nom_x[5] + true_x[5];
   out_4622954603925097103[6] = -nom_x[6] + true_x[6];
   out_4622954603925097103[7] = -nom_x[7] + true_x[7];
   out_4622954603925097103[8] = -nom_x[8] + true_x[8];
   out_4622954603925097103[9] = -nom_x[9] + true_x[9];
   out_4622954603925097103[10] = -nom_x[10] + true_x[10];
   out_4622954603925097103[11] = -nom_x[11] + true_x[11];
   out_4622954603925097103[12] = -nom_x[12] + true_x[12];
   out_4622954603925097103[13] = -nom_x[13] + true_x[13];
   out_4622954603925097103[14] = -nom_x[14] + true_x[14];
   out_4622954603925097103[15] = -nom_x[15] + true_x[15];
   out_4622954603925097103[16] = -nom_x[16] + true_x[16];
   out_4622954603925097103[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3244724203827213501) {
   out_3244724203827213501[0] = 1.0;
   out_3244724203827213501[1] = 0.0;
   out_3244724203827213501[2] = 0.0;
   out_3244724203827213501[3] = 0.0;
   out_3244724203827213501[4] = 0.0;
   out_3244724203827213501[5] = 0.0;
   out_3244724203827213501[6] = 0.0;
   out_3244724203827213501[7] = 0.0;
   out_3244724203827213501[8] = 0.0;
   out_3244724203827213501[9] = 0.0;
   out_3244724203827213501[10] = 0.0;
   out_3244724203827213501[11] = 0.0;
   out_3244724203827213501[12] = 0.0;
   out_3244724203827213501[13] = 0.0;
   out_3244724203827213501[14] = 0.0;
   out_3244724203827213501[15] = 0.0;
   out_3244724203827213501[16] = 0.0;
   out_3244724203827213501[17] = 0.0;
   out_3244724203827213501[18] = 0.0;
   out_3244724203827213501[19] = 1.0;
   out_3244724203827213501[20] = 0.0;
   out_3244724203827213501[21] = 0.0;
   out_3244724203827213501[22] = 0.0;
   out_3244724203827213501[23] = 0.0;
   out_3244724203827213501[24] = 0.0;
   out_3244724203827213501[25] = 0.0;
   out_3244724203827213501[26] = 0.0;
   out_3244724203827213501[27] = 0.0;
   out_3244724203827213501[28] = 0.0;
   out_3244724203827213501[29] = 0.0;
   out_3244724203827213501[30] = 0.0;
   out_3244724203827213501[31] = 0.0;
   out_3244724203827213501[32] = 0.0;
   out_3244724203827213501[33] = 0.0;
   out_3244724203827213501[34] = 0.0;
   out_3244724203827213501[35] = 0.0;
   out_3244724203827213501[36] = 0.0;
   out_3244724203827213501[37] = 0.0;
   out_3244724203827213501[38] = 1.0;
   out_3244724203827213501[39] = 0.0;
   out_3244724203827213501[40] = 0.0;
   out_3244724203827213501[41] = 0.0;
   out_3244724203827213501[42] = 0.0;
   out_3244724203827213501[43] = 0.0;
   out_3244724203827213501[44] = 0.0;
   out_3244724203827213501[45] = 0.0;
   out_3244724203827213501[46] = 0.0;
   out_3244724203827213501[47] = 0.0;
   out_3244724203827213501[48] = 0.0;
   out_3244724203827213501[49] = 0.0;
   out_3244724203827213501[50] = 0.0;
   out_3244724203827213501[51] = 0.0;
   out_3244724203827213501[52] = 0.0;
   out_3244724203827213501[53] = 0.0;
   out_3244724203827213501[54] = 0.0;
   out_3244724203827213501[55] = 0.0;
   out_3244724203827213501[56] = 0.0;
   out_3244724203827213501[57] = 1.0;
   out_3244724203827213501[58] = 0.0;
   out_3244724203827213501[59] = 0.0;
   out_3244724203827213501[60] = 0.0;
   out_3244724203827213501[61] = 0.0;
   out_3244724203827213501[62] = 0.0;
   out_3244724203827213501[63] = 0.0;
   out_3244724203827213501[64] = 0.0;
   out_3244724203827213501[65] = 0.0;
   out_3244724203827213501[66] = 0.0;
   out_3244724203827213501[67] = 0.0;
   out_3244724203827213501[68] = 0.0;
   out_3244724203827213501[69] = 0.0;
   out_3244724203827213501[70] = 0.0;
   out_3244724203827213501[71] = 0.0;
   out_3244724203827213501[72] = 0.0;
   out_3244724203827213501[73] = 0.0;
   out_3244724203827213501[74] = 0.0;
   out_3244724203827213501[75] = 0.0;
   out_3244724203827213501[76] = 1.0;
   out_3244724203827213501[77] = 0.0;
   out_3244724203827213501[78] = 0.0;
   out_3244724203827213501[79] = 0.0;
   out_3244724203827213501[80] = 0.0;
   out_3244724203827213501[81] = 0.0;
   out_3244724203827213501[82] = 0.0;
   out_3244724203827213501[83] = 0.0;
   out_3244724203827213501[84] = 0.0;
   out_3244724203827213501[85] = 0.0;
   out_3244724203827213501[86] = 0.0;
   out_3244724203827213501[87] = 0.0;
   out_3244724203827213501[88] = 0.0;
   out_3244724203827213501[89] = 0.0;
   out_3244724203827213501[90] = 0.0;
   out_3244724203827213501[91] = 0.0;
   out_3244724203827213501[92] = 0.0;
   out_3244724203827213501[93] = 0.0;
   out_3244724203827213501[94] = 0.0;
   out_3244724203827213501[95] = 1.0;
   out_3244724203827213501[96] = 0.0;
   out_3244724203827213501[97] = 0.0;
   out_3244724203827213501[98] = 0.0;
   out_3244724203827213501[99] = 0.0;
   out_3244724203827213501[100] = 0.0;
   out_3244724203827213501[101] = 0.0;
   out_3244724203827213501[102] = 0.0;
   out_3244724203827213501[103] = 0.0;
   out_3244724203827213501[104] = 0.0;
   out_3244724203827213501[105] = 0.0;
   out_3244724203827213501[106] = 0.0;
   out_3244724203827213501[107] = 0.0;
   out_3244724203827213501[108] = 0.0;
   out_3244724203827213501[109] = 0.0;
   out_3244724203827213501[110] = 0.0;
   out_3244724203827213501[111] = 0.0;
   out_3244724203827213501[112] = 0.0;
   out_3244724203827213501[113] = 0.0;
   out_3244724203827213501[114] = 1.0;
   out_3244724203827213501[115] = 0.0;
   out_3244724203827213501[116] = 0.0;
   out_3244724203827213501[117] = 0.0;
   out_3244724203827213501[118] = 0.0;
   out_3244724203827213501[119] = 0.0;
   out_3244724203827213501[120] = 0.0;
   out_3244724203827213501[121] = 0.0;
   out_3244724203827213501[122] = 0.0;
   out_3244724203827213501[123] = 0.0;
   out_3244724203827213501[124] = 0.0;
   out_3244724203827213501[125] = 0.0;
   out_3244724203827213501[126] = 0.0;
   out_3244724203827213501[127] = 0.0;
   out_3244724203827213501[128] = 0.0;
   out_3244724203827213501[129] = 0.0;
   out_3244724203827213501[130] = 0.0;
   out_3244724203827213501[131] = 0.0;
   out_3244724203827213501[132] = 0.0;
   out_3244724203827213501[133] = 1.0;
   out_3244724203827213501[134] = 0.0;
   out_3244724203827213501[135] = 0.0;
   out_3244724203827213501[136] = 0.0;
   out_3244724203827213501[137] = 0.0;
   out_3244724203827213501[138] = 0.0;
   out_3244724203827213501[139] = 0.0;
   out_3244724203827213501[140] = 0.0;
   out_3244724203827213501[141] = 0.0;
   out_3244724203827213501[142] = 0.0;
   out_3244724203827213501[143] = 0.0;
   out_3244724203827213501[144] = 0.0;
   out_3244724203827213501[145] = 0.0;
   out_3244724203827213501[146] = 0.0;
   out_3244724203827213501[147] = 0.0;
   out_3244724203827213501[148] = 0.0;
   out_3244724203827213501[149] = 0.0;
   out_3244724203827213501[150] = 0.0;
   out_3244724203827213501[151] = 0.0;
   out_3244724203827213501[152] = 1.0;
   out_3244724203827213501[153] = 0.0;
   out_3244724203827213501[154] = 0.0;
   out_3244724203827213501[155] = 0.0;
   out_3244724203827213501[156] = 0.0;
   out_3244724203827213501[157] = 0.0;
   out_3244724203827213501[158] = 0.0;
   out_3244724203827213501[159] = 0.0;
   out_3244724203827213501[160] = 0.0;
   out_3244724203827213501[161] = 0.0;
   out_3244724203827213501[162] = 0.0;
   out_3244724203827213501[163] = 0.0;
   out_3244724203827213501[164] = 0.0;
   out_3244724203827213501[165] = 0.0;
   out_3244724203827213501[166] = 0.0;
   out_3244724203827213501[167] = 0.0;
   out_3244724203827213501[168] = 0.0;
   out_3244724203827213501[169] = 0.0;
   out_3244724203827213501[170] = 0.0;
   out_3244724203827213501[171] = 1.0;
   out_3244724203827213501[172] = 0.0;
   out_3244724203827213501[173] = 0.0;
   out_3244724203827213501[174] = 0.0;
   out_3244724203827213501[175] = 0.0;
   out_3244724203827213501[176] = 0.0;
   out_3244724203827213501[177] = 0.0;
   out_3244724203827213501[178] = 0.0;
   out_3244724203827213501[179] = 0.0;
   out_3244724203827213501[180] = 0.0;
   out_3244724203827213501[181] = 0.0;
   out_3244724203827213501[182] = 0.0;
   out_3244724203827213501[183] = 0.0;
   out_3244724203827213501[184] = 0.0;
   out_3244724203827213501[185] = 0.0;
   out_3244724203827213501[186] = 0.0;
   out_3244724203827213501[187] = 0.0;
   out_3244724203827213501[188] = 0.0;
   out_3244724203827213501[189] = 0.0;
   out_3244724203827213501[190] = 1.0;
   out_3244724203827213501[191] = 0.0;
   out_3244724203827213501[192] = 0.0;
   out_3244724203827213501[193] = 0.0;
   out_3244724203827213501[194] = 0.0;
   out_3244724203827213501[195] = 0.0;
   out_3244724203827213501[196] = 0.0;
   out_3244724203827213501[197] = 0.0;
   out_3244724203827213501[198] = 0.0;
   out_3244724203827213501[199] = 0.0;
   out_3244724203827213501[200] = 0.0;
   out_3244724203827213501[201] = 0.0;
   out_3244724203827213501[202] = 0.0;
   out_3244724203827213501[203] = 0.0;
   out_3244724203827213501[204] = 0.0;
   out_3244724203827213501[205] = 0.0;
   out_3244724203827213501[206] = 0.0;
   out_3244724203827213501[207] = 0.0;
   out_3244724203827213501[208] = 0.0;
   out_3244724203827213501[209] = 1.0;
   out_3244724203827213501[210] = 0.0;
   out_3244724203827213501[211] = 0.0;
   out_3244724203827213501[212] = 0.0;
   out_3244724203827213501[213] = 0.0;
   out_3244724203827213501[214] = 0.0;
   out_3244724203827213501[215] = 0.0;
   out_3244724203827213501[216] = 0.0;
   out_3244724203827213501[217] = 0.0;
   out_3244724203827213501[218] = 0.0;
   out_3244724203827213501[219] = 0.0;
   out_3244724203827213501[220] = 0.0;
   out_3244724203827213501[221] = 0.0;
   out_3244724203827213501[222] = 0.0;
   out_3244724203827213501[223] = 0.0;
   out_3244724203827213501[224] = 0.0;
   out_3244724203827213501[225] = 0.0;
   out_3244724203827213501[226] = 0.0;
   out_3244724203827213501[227] = 0.0;
   out_3244724203827213501[228] = 1.0;
   out_3244724203827213501[229] = 0.0;
   out_3244724203827213501[230] = 0.0;
   out_3244724203827213501[231] = 0.0;
   out_3244724203827213501[232] = 0.0;
   out_3244724203827213501[233] = 0.0;
   out_3244724203827213501[234] = 0.0;
   out_3244724203827213501[235] = 0.0;
   out_3244724203827213501[236] = 0.0;
   out_3244724203827213501[237] = 0.0;
   out_3244724203827213501[238] = 0.0;
   out_3244724203827213501[239] = 0.0;
   out_3244724203827213501[240] = 0.0;
   out_3244724203827213501[241] = 0.0;
   out_3244724203827213501[242] = 0.0;
   out_3244724203827213501[243] = 0.0;
   out_3244724203827213501[244] = 0.0;
   out_3244724203827213501[245] = 0.0;
   out_3244724203827213501[246] = 0.0;
   out_3244724203827213501[247] = 1.0;
   out_3244724203827213501[248] = 0.0;
   out_3244724203827213501[249] = 0.0;
   out_3244724203827213501[250] = 0.0;
   out_3244724203827213501[251] = 0.0;
   out_3244724203827213501[252] = 0.0;
   out_3244724203827213501[253] = 0.0;
   out_3244724203827213501[254] = 0.0;
   out_3244724203827213501[255] = 0.0;
   out_3244724203827213501[256] = 0.0;
   out_3244724203827213501[257] = 0.0;
   out_3244724203827213501[258] = 0.0;
   out_3244724203827213501[259] = 0.0;
   out_3244724203827213501[260] = 0.0;
   out_3244724203827213501[261] = 0.0;
   out_3244724203827213501[262] = 0.0;
   out_3244724203827213501[263] = 0.0;
   out_3244724203827213501[264] = 0.0;
   out_3244724203827213501[265] = 0.0;
   out_3244724203827213501[266] = 1.0;
   out_3244724203827213501[267] = 0.0;
   out_3244724203827213501[268] = 0.0;
   out_3244724203827213501[269] = 0.0;
   out_3244724203827213501[270] = 0.0;
   out_3244724203827213501[271] = 0.0;
   out_3244724203827213501[272] = 0.0;
   out_3244724203827213501[273] = 0.0;
   out_3244724203827213501[274] = 0.0;
   out_3244724203827213501[275] = 0.0;
   out_3244724203827213501[276] = 0.0;
   out_3244724203827213501[277] = 0.0;
   out_3244724203827213501[278] = 0.0;
   out_3244724203827213501[279] = 0.0;
   out_3244724203827213501[280] = 0.0;
   out_3244724203827213501[281] = 0.0;
   out_3244724203827213501[282] = 0.0;
   out_3244724203827213501[283] = 0.0;
   out_3244724203827213501[284] = 0.0;
   out_3244724203827213501[285] = 1.0;
   out_3244724203827213501[286] = 0.0;
   out_3244724203827213501[287] = 0.0;
   out_3244724203827213501[288] = 0.0;
   out_3244724203827213501[289] = 0.0;
   out_3244724203827213501[290] = 0.0;
   out_3244724203827213501[291] = 0.0;
   out_3244724203827213501[292] = 0.0;
   out_3244724203827213501[293] = 0.0;
   out_3244724203827213501[294] = 0.0;
   out_3244724203827213501[295] = 0.0;
   out_3244724203827213501[296] = 0.0;
   out_3244724203827213501[297] = 0.0;
   out_3244724203827213501[298] = 0.0;
   out_3244724203827213501[299] = 0.0;
   out_3244724203827213501[300] = 0.0;
   out_3244724203827213501[301] = 0.0;
   out_3244724203827213501[302] = 0.0;
   out_3244724203827213501[303] = 0.0;
   out_3244724203827213501[304] = 1.0;
   out_3244724203827213501[305] = 0.0;
   out_3244724203827213501[306] = 0.0;
   out_3244724203827213501[307] = 0.0;
   out_3244724203827213501[308] = 0.0;
   out_3244724203827213501[309] = 0.0;
   out_3244724203827213501[310] = 0.0;
   out_3244724203827213501[311] = 0.0;
   out_3244724203827213501[312] = 0.0;
   out_3244724203827213501[313] = 0.0;
   out_3244724203827213501[314] = 0.0;
   out_3244724203827213501[315] = 0.0;
   out_3244724203827213501[316] = 0.0;
   out_3244724203827213501[317] = 0.0;
   out_3244724203827213501[318] = 0.0;
   out_3244724203827213501[319] = 0.0;
   out_3244724203827213501[320] = 0.0;
   out_3244724203827213501[321] = 0.0;
   out_3244724203827213501[322] = 0.0;
   out_3244724203827213501[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5671114693680624888) {
   out_5671114693680624888[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5671114693680624888[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5671114693680624888[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5671114693680624888[3] = dt*state[12] + state[3];
   out_5671114693680624888[4] = dt*state[13] + state[4];
   out_5671114693680624888[5] = dt*state[14] + state[5];
   out_5671114693680624888[6] = state[6];
   out_5671114693680624888[7] = state[7];
   out_5671114693680624888[8] = state[8];
   out_5671114693680624888[9] = state[9];
   out_5671114693680624888[10] = state[10];
   out_5671114693680624888[11] = state[11];
   out_5671114693680624888[12] = state[12];
   out_5671114693680624888[13] = state[13];
   out_5671114693680624888[14] = state[14];
   out_5671114693680624888[15] = state[15];
   out_5671114693680624888[16] = state[16];
   out_5671114693680624888[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8302195571783514857) {
   out_8302195571783514857[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8302195571783514857[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8302195571783514857[2] = 0;
   out_8302195571783514857[3] = 0;
   out_8302195571783514857[4] = 0;
   out_8302195571783514857[5] = 0;
   out_8302195571783514857[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8302195571783514857[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8302195571783514857[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8302195571783514857[9] = 0;
   out_8302195571783514857[10] = 0;
   out_8302195571783514857[11] = 0;
   out_8302195571783514857[12] = 0;
   out_8302195571783514857[13] = 0;
   out_8302195571783514857[14] = 0;
   out_8302195571783514857[15] = 0;
   out_8302195571783514857[16] = 0;
   out_8302195571783514857[17] = 0;
   out_8302195571783514857[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8302195571783514857[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8302195571783514857[20] = 0;
   out_8302195571783514857[21] = 0;
   out_8302195571783514857[22] = 0;
   out_8302195571783514857[23] = 0;
   out_8302195571783514857[24] = 0;
   out_8302195571783514857[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8302195571783514857[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8302195571783514857[27] = 0;
   out_8302195571783514857[28] = 0;
   out_8302195571783514857[29] = 0;
   out_8302195571783514857[30] = 0;
   out_8302195571783514857[31] = 0;
   out_8302195571783514857[32] = 0;
   out_8302195571783514857[33] = 0;
   out_8302195571783514857[34] = 0;
   out_8302195571783514857[35] = 0;
   out_8302195571783514857[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8302195571783514857[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8302195571783514857[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8302195571783514857[39] = 0;
   out_8302195571783514857[40] = 0;
   out_8302195571783514857[41] = 0;
   out_8302195571783514857[42] = 0;
   out_8302195571783514857[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8302195571783514857[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8302195571783514857[45] = 0;
   out_8302195571783514857[46] = 0;
   out_8302195571783514857[47] = 0;
   out_8302195571783514857[48] = 0;
   out_8302195571783514857[49] = 0;
   out_8302195571783514857[50] = 0;
   out_8302195571783514857[51] = 0;
   out_8302195571783514857[52] = 0;
   out_8302195571783514857[53] = 0;
   out_8302195571783514857[54] = 0;
   out_8302195571783514857[55] = 0;
   out_8302195571783514857[56] = 0;
   out_8302195571783514857[57] = 1;
   out_8302195571783514857[58] = 0;
   out_8302195571783514857[59] = 0;
   out_8302195571783514857[60] = 0;
   out_8302195571783514857[61] = 0;
   out_8302195571783514857[62] = 0;
   out_8302195571783514857[63] = 0;
   out_8302195571783514857[64] = 0;
   out_8302195571783514857[65] = 0;
   out_8302195571783514857[66] = dt;
   out_8302195571783514857[67] = 0;
   out_8302195571783514857[68] = 0;
   out_8302195571783514857[69] = 0;
   out_8302195571783514857[70] = 0;
   out_8302195571783514857[71] = 0;
   out_8302195571783514857[72] = 0;
   out_8302195571783514857[73] = 0;
   out_8302195571783514857[74] = 0;
   out_8302195571783514857[75] = 0;
   out_8302195571783514857[76] = 1;
   out_8302195571783514857[77] = 0;
   out_8302195571783514857[78] = 0;
   out_8302195571783514857[79] = 0;
   out_8302195571783514857[80] = 0;
   out_8302195571783514857[81] = 0;
   out_8302195571783514857[82] = 0;
   out_8302195571783514857[83] = 0;
   out_8302195571783514857[84] = 0;
   out_8302195571783514857[85] = dt;
   out_8302195571783514857[86] = 0;
   out_8302195571783514857[87] = 0;
   out_8302195571783514857[88] = 0;
   out_8302195571783514857[89] = 0;
   out_8302195571783514857[90] = 0;
   out_8302195571783514857[91] = 0;
   out_8302195571783514857[92] = 0;
   out_8302195571783514857[93] = 0;
   out_8302195571783514857[94] = 0;
   out_8302195571783514857[95] = 1;
   out_8302195571783514857[96] = 0;
   out_8302195571783514857[97] = 0;
   out_8302195571783514857[98] = 0;
   out_8302195571783514857[99] = 0;
   out_8302195571783514857[100] = 0;
   out_8302195571783514857[101] = 0;
   out_8302195571783514857[102] = 0;
   out_8302195571783514857[103] = 0;
   out_8302195571783514857[104] = dt;
   out_8302195571783514857[105] = 0;
   out_8302195571783514857[106] = 0;
   out_8302195571783514857[107] = 0;
   out_8302195571783514857[108] = 0;
   out_8302195571783514857[109] = 0;
   out_8302195571783514857[110] = 0;
   out_8302195571783514857[111] = 0;
   out_8302195571783514857[112] = 0;
   out_8302195571783514857[113] = 0;
   out_8302195571783514857[114] = 1;
   out_8302195571783514857[115] = 0;
   out_8302195571783514857[116] = 0;
   out_8302195571783514857[117] = 0;
   out_8302195571783514857[118] = 0;
   out_8302195571783514857[119] = 0;
   out_8302195571783514857[120] = 0;
   out_8302195571783514857[121] = 0;
   out_8302195571783514857[122] = 0;
   out_8302195571783514857[123] = 0;
   out_8302195571783514857[124] = 0;
   out_8302195571783514857[125] = 0;
   out_8302195571783514857[126] = 0;
   out_8302195571783514857[127] = 0;
   out_8302195571783514857[128] = 0;
   out_8302195571783514857[129] = 0;
   out_8302195571783514857[130] = 0;
   out_8302195571783514857[131] = 0;
   out_8302195571783514857[132] = 0;
   out_8302195571783514857[133] = 1;
   out_8302195571783514857[134] = 0;
   out_8302195571783514857[135] = 0;
   out_8302195571783514857[136] = 0;
   out_8302195571783514857[137] = 0;
   out_8302195571783514857[138] = 0;
   out_8302195571783514857[139] = 0;
   out_8302195571783514857[140] = 0;
   out_8302195571783514857[141] = 0;
   out_8302195571783514857[142] = 0;
   out_8302195571783514857[143] = 0;
   out_8302195571783514857[144] = 0;
   out_8302195571783514857[145] = 0;
   out_8302195571783514857[146] = 0;
   out_8302195571783514857[147] = 0;
   out_8302195571783514857[148] = 0;
   out_8302195571783514857[149] = 0;
   out_8302195571783514857[150] = 0;
   out_8302195571783514857[151] = 0;
   out_8302195571783514857[152] = 1;
   out_8302195571783514857[153] = 0;
   out_8302195571783514857[154] = 0;
   out_8302195571783514857[155] = 0;
   out_8302195571783514857[156] = 0;
   out_8302195571783514857[157] = 0;
   out_8302195571783514857[158] = 0;
   out_8302195571783514857[159] = 0;
   out_8302195571783514857[160] = 0;
   out_8302195571783514857[161] = 0;
   out_8302195571783514857[162] = 0;
   out_8302195571783514857[163] = 0;
   out_8302195571783514857[164] = 0;
   out_8302195571783514857[165] = 0;
   out_8302195571783514857[166] = 0;
   out_8302195571783514857[167] = 0;
   out_8302195571783514857[168] = 0;
   out_8302195571783514857[169] = 0;
   out_8302195571783514857[170] = 0;
   out_8302195571783514857[171] = 1;
   out_8302195571783514857[172] = 0;
   out_8302195571783514857[173] = 0;
   out_8302195571783514857[174] = 0;
   out_8302195571783514857[175] = 0;
   out_8302195571783514857[176] = 0;
   out_8302195571783514857[177] = 0;
   out_8302195571783514857[178] = 0;
   out_8302195571783514857[179] = 0;
   out_8302195571783514857[180] = 0;
   out_8302195571783514857[181] = 0;
   out_8302195571783514857[182] = 0;
   out_8302195571783514857[183] = 0;
   out_8302195571783514857[184] = 0;
   out_8302195571783514857[185] = 0;
   out_8302195571783514857[186] = 0;
   out_8302195571783514857[187] = 0;
   out_8302195571783514857[188] = 0;
   out_8302195571783514857[189] = 0;
   out_8302195571783514857[190] = 1;
   out_8302195571783514857[191] = 0;
   out_8302195571783514857[192] = 0;
   out_8302195571783514857[193] = 0;
   out_8302195571783514857[194] = 0;
   out_8302195571783514857[195] = 0;
   out_8302195571783514857[196] = 0;
   out_8302195571783514857[197] = 0;
   out_8302195571783514857[198] = 0;
   out_8302195571783514857[199] = 0;
   out_8302195571783514857[200] = 0;
   out_8302195571783514857[201] = 0;
   out_8302195571783514857[202] = 0;
   out_8302195571783514857[203] = 0;
   out_8302195571783514857[204] = 0;
   out_8302195571783514857[205] = 0;
   out_8302195571783514857[206] = 0;
   out_8302195571783514857[207] = 0;
   out_8302195571783514857[208] = 0;
   out_8302195571783514857[209] = 1;
   out_8302195571783514857[210] = 0;
   out_8302195571783514857[211] = 0;
   out_8302195571783514857[212] = 0;
   out_8302195571783514857[213] = 0;
   out_8302195571783514857[214] = 0;
   out_8302195571783514857[215] = 0;
   out_8302195571783514857[216] = 0;
   out_8302195571783514857[217] = 0;
   out_8302195571783514857[218] = 0;
   out_8302195571783514857[219] = 0;
   out_8302195571783514857[220] = 0;
   out_8302195571783514857[221] = 0;
   out_8302195571783514857[222] = 0;
   out_8302195571783514857[223] = 0;
   out_8302195571783514857[224] = 0;
   out_8302195571783514857[225] = 0;
   out_8302195571783514857[226] = 0;
   out_8302195571783514857[227] = 0;
   out_8302195571783514857[228] = 1;
   out_8302195571783514857[229] = 0;
   out_8302195571783514857[230] = 0;
   out_8302195571783514857[231] = 0;
   out_8302195571783514857[232] = 0;
   out_8302195571783514857[233] = 0;
   out_8302195571783514857[234] = 0;
   out_8302195571783514857[235] = 0;
   out_8302195571783514857[236] = 0;
   out_8302195571783514857[237] = 0;
   out_8302195571783514857[238] = 0;
   out_8302195571783514857[239] = 0;
   out_8302195571783514857[240] = 0;
   out_8302195571783514857[241] = 0;
   out_8302195571783514857[242] = 0;
   out_8302195571783514857[243] = 0;
   out_8302195571783514857[244] = 0;
   out_8302195571783514857[245] = 0;
   out_8302195571783514857[246] = 0;
   out_8302195571783514857[247] = 1;
   out_8302195571783514857[248] = 0;
   out_8302195571783514857[249] = 0;
   out_8302195571783514857[250] = 0;
   out_8302195571783514857[251] = 0;
   out_8302195571783514857[252] = 0;
   out_8302195571783514857[253] = 0;
   out_8302195571783514857[254] = 0;
   out_8302195571783514857[255] = 0;
   out_8302195571783514857[256] = 0;
   out_8302195571783514857[257] = 0;
   out_8302195571783514857[258] = 0;
   out_8302195571783514857[259] = 0;
   out_8302195571783514857[260] = 0;
   out_8302195571783514857[261] = 0;
   out_8302195571783514857[262] = 0;
   out_8302195571783514857[263] = 0;
   out_8302195571783514857[264] = 0;
   out_8302195571783514857[265] = 0;
   out_8302195571783514857[266] = 1;
   out_8302195571783514857[267] = 0;
   out_8302195571783514857[268] = 0;
   out_8302195571783514857[269] = 0;
   out_8302195571783514857[270] = 0;
   out_8302195571783514857[271] = 0;
   out_8302195571783514857[272] = 0;
   out_8302195571783514857[273] = 0;
   out_8302195571783514857[274] = 0;
   out_8302195571783514857[275] = 0;
   out_8302195571783514857[276] = 0;
   out_8302195571783514857[277] = 0;
   out_8302195571783514857[278] = 0;
   out_8302195571783514857[279] = 0;
   out_8302195571783514857[280] = 0;
   out_8302195571783514857[281] = 0;
   out_8302195571783514857[282] = 0;
   out_8302195571783514857[283] = 0;
   out_8302195571783514857[284] = 0;
   out_8302195571783514857[285] = 1;
   out_8302195571783514857[286] = 0;
   out_8302195571783514857[287] = 0;
   out_8302195571783514857[288] = 0;
   out_8302195571783514857[289] = 0;
   out_8302195571783514857[290] = 0;
   out_8302195571783514857[291] = 0;
   out_8302195571783514857[292] = 0;
   out_8302195571783514857[293] = 0;
   out_8302195571783514857[294] = 0;
   out_8302195571783514857[295] = 0;
   out_8302195571783514857[296] = 0;
   out_8302195571783514857[297] = 0;
   out_8302195571783514857[298] = 0;
   out_8302195571783514857[299] = 0;
   out_8302195571783514857[300] = 0;
   out_8302195571783514857[301] = 0;
   out_8302195571783514857[302] = 0;
   out_8302195571783514857[303] = 0;
   out_8302195571783514857[304] = 1;
   out_8302195571783514857[305] = 0;
   out_8302195571783514857[306] = 0;
   out_8302195571783514857[307] = 0;
   out_8302195571783514857[308] = 0;
   out_8302195571783514857[309] = 0;
   out_8302195571783514857[310] = 0;
   out_8302195571783514857[311] = 0;
   out_8302195571783514857[312] = 0;
   out_8302195571783514857[313] = 0;
   out_8302195571783514857[314] = 0;
   out_8302195571783514857[315] = 0;
   out_8302195571783514857[316] = 0;
   out_8302195571783514857[317] = 0;
   out_8302195571783514857[318] = 0;
   out_8302195571783514857[319] = 0;
   out_8302195571783514857[320] = 0;
   out_8302195571783514857[321] = 0;
   out_8302195571783514857[322] = 0;
   out_8302195571783514857[323] = 1;
}
void h_4(double *state, double *unused, double *out_1554668460974419477) {
   out_1554668460974419477[0] = state[6] + state[9];
   out_1554668460974419477[1] = state[7] + state[10];
   out_1554668460974419477[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_44314302725743477) {
   out_44314302725743477[0] = 0;
   out_44314302725743477[1] = 0;
   out_44314302725743477[2] = 0;
   out_44314302725743477[3] = 0;
   out_44314302725743477[4] = 0;
   out_44314302725743477[5] = 0;
   out_44314302725743477[6] = 1;
   out_44314302725743477[7] = 0;
   out_44314302725743477[8] = 0;
   out_44314302725743477[9] = 1;
   out_44314302725743477[10] = 0;
   out_44314302725743477[11] = 0;
   out_44314302725743477[12] = 0;
   out_44314302725743477[13] = 0;
   out_44314302725743477[14] = 0;
   out_44314302725743477[15] = 0;
   out_44314302725743477[16] = 0;
   out_44314302725743477[17] = 0;
   out_44314302725743477[18] = 0;
   out_44314302725743477[19] = 0;
   out_44314302725743477[20] = 0;
   out_44314302725743477[21] = 0;
   out_44314302725743477[22] = 0;
   out_44314302725743477[23] = 0;
   out_44314302725743477[24] = 0;
   out_44314302725743477[25] = 1;
   out_44314302725743477[26] = 0;
   out_44314302725743477[27] = 0;
   out_44314302725743477[28] = 1;
   out_44314302725743477[29] = 0;
   out_44314302725743477[30] = 0;
   out_44314302725743477[31] = 0;
   out_44314302725743477[32] = 0;
   out_44314302725743477[33] = 0;
   out_44314302725743477[34] = 0;
   out_44314302725743477[35] = 0;
   out_44314302725743477[36] = 0;
   out_44314302725743477[37] = 0;
   out_44314302725743477[38] = 0;
   out_44314302725743477[39] = 0;
   out_44314302725743477[40] = 0;
   out_44314302725743477[41] = 0;
   out_44314302725743477[42] = 0;
   out_44314302725743477[43] = 0;
   out_44314302725743477[44] = 1;
   out_44314302725743477[45] = 0;
   out_44314302725743477[46] = 0;
   out_44314302725743477[47] = 1;
   out_44314302725743477[48] = 0;
   out_44314302725743477[49] = 0;
   out_44314302725743477[50] = 0;
   out_44314302725743477[51] = 0;
   out_44314302725743477[52] = 0;
   out_44314302725743477[53] = 0;
}
void h_10(double *state, double *unused, double *out_2262676311495278110) {
   out_2262676311495278110[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2262676311495278110[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2262676311495278110[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3242013822092943294) {
   out_3242013822092943294[0] = 0;
   out_3242013822092943294[1] = 9.8100000000000005*cos(state[1]);
   out_3242013822092943294[2] = 0;
   out_3242013822092943294[3] = 0;
   out_3242013822092943294[4] = -state[8];
   out_3242013822092943294[5] = state[7];
   out_3242013822092943294[6] = 0;
   out_3242013822092943294[7] = state[5];
   out_3242013822092943294[8] = -state[4];
   out_3242013822092943294[9] = 0;
   out_3242013822092943294[10] = 0;
   out_3242013822092943294[11] = 0;
   out_3242013822092943294[12] = 1;
   out_3242013822092943294[13] = 0;
   out_3242013822092943294[14] = 0;
   out_3242013822092943294[15] = 1;
   out_3242013822092943294[16] = 0;
   out_3242013822092943294[17] = 0;
   out_3242013822092943294[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3242013822092943294[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3242013822092943294[20] = 0;
   out_3242013822092943294[21] = state[8];
   out_3242013822092943294[22] = 0;
   out_3242013822092943294[23] = -state[6];
   out_3242013822092943294[24] = -state[5];
   out_3242013822092943294[25] = 0;
   out_3242013822092943294[26] = state[3];
   out_3242013822092943294[27] = 0;
   out_3242013822092943294[28] = 0;
   out_3242013822092943294[29] = 0;
   out_3242013822092943294[30] = 0;
   out_3242013822092943294[31] = 1;
   out_3242013822092943294[32] = 0;
   out_3242013822092943294[33] = 0;
   out_3242013822092943294[34] = 1;
   out_3242013822092943294[35] = 0;
   out_3242013822092943294[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3242013822092943294[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3242013822092943294[38] = 0;
   out_3242013822092943294[39] = -state[7];
   out_3242013822092943294[40] = state[6];
   out_3242013822092943294[41] = 0;
   out_3242013822092943294[42] = state[4];
   out_3242013822092943294[43] = -state[3];
   out_3242013822092943294[44] = 0;
   out_3242013822092943294[45] = 0;
   out_3242013822092943294[46] = 0;
   out_3242013822092943294[47] = 0;
   out_3242013822092943294[48] = 0;
   out_3242013822092943294[49] = 0;
   out_3242013822092943294[50] = 1;
   out_3242013822092943294[51] = 0;
   out_3242013822092943294[52] = 0;
   out_3242013822092943294[53] = 1;
}
void h_13(double *state, double *unused, double *out_8592105098224939032) {
   out_8592105098224939032[0] = state[3];
   out_8592105098224939032[1] = state[4];
   out_8592105098224939032[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7654945511042444406) {
   out_7654945511042444406[0] = 0;
   out_7654945511042444406[1] = 0;
   out_7654945511042444406[2] = 0;
   out_7654945511042444406[3] = 1;
   out_7654945511042444406[4] = 0;
   out_7654945511042444406[5] = 0;
   out_7654945511042444406[6] = 0;
   out_7654945511042444406[7] = 0;
   out_7654945511042444406[8] = 0;
   out_7654945511042444406[9] = 0;
   out_7654945511042444406[10] = 0;
   out_7654945511042444406[11] = 0;
   out_7654945511042444406[12] = 0;
   out_7654945511042444406[13] = 0;
   out_7654945511042444406[14] = 0;
   out_7654945511042444406[15] = 0;
   out_7654945511042444406[16] = 0;
   out_7654945511042444406[17] = 0;
   out_7654945511042444406[18] = 0;
   out_7654945511042444406[19] = 0;
   out_7654945511042444406[20] = 0;
   out_7654945511042444406[21] = 0;
   out_7654945511042444406[22] = 1;
   out_7654945511042444406[23] = 0;
   out_7654945511042444406[24] = 0;
   out_7654945511042444406[25] = 0;
   out_7654945511042444406[26] = 0;
   out_7654945511042444406[27] = 0;
   out_7654945511042444406[28] = 0;
   out_7654945511042444406[29] = 0;
   out_7654945511042444406[30] = 0;
   out_7654945511042444406[31] = 0;
   out_7654945511042444406[32] = 0;
   out_7654945511042444406[33] = 0;
   out_7654945511042444406[34] = 0;
   out_7654945511042444406[35] = 0;
   out_7654945511042444406[36] = 0;
   out_7654945511042444406[37] = 0;
   out_7654945511042444406[38] = 0;
   out_7654945511042444406[39] = 0;
   out_7654945511042444406[40] = 0;
   out_7654945511042444406[41] = 1;
   out_7654945511042444406[42] = 0;
   out_7654945511042444406[43] = 0;
   out_7654945511042444406[44] = 0;
   out_7654945511042444406[45] = 0;
   out_7654945511042444406[46] = 0;
   out_7654945511042444406[47] = 0;
   out_7654945511042444406[48] = 0;
   out_7654945511042444406[49] = 0;
   out_7654945511042444406[50] = 0;
   out_7654945511042444406[51] = 0;
   out_7654945511042444406[52] = 0;
   out_7654945511042444406[53] = 0;
}
void h_14(double *state, double *unused, double *out_4909758137732599256) {
   out_4909758137732599256[0] = state[6];
   out_4909758137732599256[1] = state[7];
   out_4909758137732599256[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3038474129569628819) {
   out_3038474129569628819[0] = 0;
   out_3038474129569628819[1] = 0;
   out_3038474129569628819[2] = 0;
   out_3038474129569628819[3] = 0;
   out_3038474129569628819[4] = 0;
   out_3038474129569628819[5] = 0;
   out_3038474129569628819[6] = 1;
   out_3038474129569628819[7] = 0;
   out_3038474129569628819[8] = 0;
   out_3038474129569628819[9] = 0;
   out_3038474129569628819[10] = 0;
   out_3038474129569628819[11] = 0;
   out_3038474129569628819[12] = 0;
   out_3038474129569628819[13] = 0;
   out_3038474129569628819[14] = 0;
   out_3038474129569628819[15] = 0;
   out_3038474129569628819[16] = 0;
   out_3038474129569628819[17] = 0;
   out_3038474129569628819[18] = 0;
   out_3038474129569628819[19] = 0;
   out_3038474129569628819[20] = 0;
   out_3038474129569628819[21] = 0;
   out_3038474129569628819[22] = 0;
   out_3038474129569628819[23] = 0;
   out_3038474129569628819[24] = 0;
   out_3038474129569628819[25] = 1;
   out_3038474129569628819[26] = 0;
   out_3038474129569628819[27] = 0;
   out_3038474129569628819[28] = 0;
   out_3038474129569628819[29] = 0;
   out_3038474129569628819[30] = 0;
   out_3038474129569628819[31] = 0;
   out_3038474129569628819[32] = 0;
   out_3038474129569628819[33] = 0;
   out_3038474129569628819[34] = 0;
   out_3038474129569628819[35] = 0;
   out_3038474129569628819[36] = 0;
   out_3038474129569628819[37] = 0;
   out_3038474129569628819[38] = 0;
   out_3038474129569628819[39] = 0;
   out_3038474129569628819[40] = 0;
   out_3038474129569628819[41] = 0;
   out_3038474129569628819[42] = 0;
   out_3038474129569628819[43] = 0;
   out_3038474129569628819[44] = 1;
   out_3038474129569628819[45] = 0;
   out_3038474129569628819[46] = 0;
   out_3038474129569628819[47] = 0;
   out_3038474129569628819[48] = 0;
   out_3038474129569628819[49] = 0;
   out_3038474129569628819[50] = 0;
   out_3038474129569628819[51] = 0;
   out_3038474129569628819[52] = 0;
   out_3038474129569628819[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_945005396424872191) {
  err_fun(nom_x, delta_x, out_945005396424872191);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4622954603925097103) {
  inv_err_fun(nom_x, true_x, out_4622954603925097103);
}
void pose_H_mod_fun(double *state, double *out_3244724203827213501) {
  H_mod_fun(state, out_3244724203827213501);
}
void pose_f_fun(double *state, double dt, double *out_5671114693680624888) {
  f_fun(state,  dt, out_5671114693680624888);
}
void pose_F_fun(double *state, double dt, double *out_8302195571783514857) {
  F_fun(state,  dt, out_8302195571783514857);
}
void pose_h_4(double *state, double *unused, double *out_1554668460974419477) {
  h_4(state, unused, out_1554668460974419477);
}
void pose_H_4(double *state, double *unused, double *out_44314302725743477) {
  H_4(state, unused, out_44314302725743477);
}
void pose_h_10(double *state, double *unused, double *out_2262676311495278110) {
  h_10(state, unused, out_2262676311495278110);
}
void pose_H_10(double *state, double *unused, double *out_3242013822092943294) {
  H_10(state, unused, out_3242013822092943294);
}
void pose_h_13(double *state, double *unused, double *out_8592105098224939032) {
  h_13(state, unused, out_8592105098224939032);
}
void pose_H_13(double *state, double *unused, double *out_7654945511042444406) {
  H_13(state, unused, out_7654945511042444406);
}
void pose_h_14(double *state, double *unused, double *out_4909758137732599256) {
  h_14(state, unused, out_4909758137732599256);
}
void pose_H_14(double *state, double *unused, double *out_3038474129569628819) {
  H_14(state, unused, out_3038474129569628819);
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
