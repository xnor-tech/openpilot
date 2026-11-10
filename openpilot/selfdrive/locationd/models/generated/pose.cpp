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
void err_fun(double *nom_x, double *delta_x, double *out_2691520434775550490) {
   out_2691520434775550490[0] = delta_x[0] + nom_x[0];
   out_2691520434775550490[1] = delta_x[1] + nom_x[1];
   out_2691520434775550490[2] = delta_x[2] + nom_x[2];
   out_2691520434775550490[3] = delta_x[3] + nom_x[3];
   out_2691520434775550490[4] = delta_x[4] + nom_x[4];
   out_2691520434775550490[5] = delta_x[5] + nom_x[5];
   out_2691520434775550490[6] = delta_x[6] + nom_x[6];
   out_2691520434775550490[7] = delta_x[7] + nom_x[7];
   out_2691520434775550490[8] = delta_x[8] + nom_x[8];
   out_2691520434775550490[9] = delta_x[9] + nom_x[9];
   out_2691520434775550490[10] = delta_x[10] + nom_x[10];
   out_2691520434775550490[11] = delta_x[11] + nom_x[11];
   out_2691520434775550490[12] = delta_x[12] + nom_x[12];
   out_2691520434775550490[13] = delta_x[13] + nom_x[13];
   out_2691520434775550490[14] = delta_x[14] + nom_x[14];
   out_2691520434775550490[15] = delta_x[15] + nom_x[15];
   out_2691520434775550490[16] = delta_x[16] + nom_x[16];
   out_2691520434775550490[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2397415840402016948) {
   out_2397415840402016948[0] = -nom_x[0] + true_x[0];
   out_2397415840402016948[1] = -nom_x[1] + true_x[1];
   out_2397415840402016948[2] = -nom_x[2] + true_x[2];
   out_2397415840402016948[3] = -nom_x[3] + true_x[3];
   out_2397415840402016948[4] = -nom_x[4] + true_x[4];
   out_2397415840402016948[5] = -nom_x[5] + true_x[5];
   out_2397415840402016948[6] = -nom_x[6] + true_x[6];
   out_2397415840402016948[7] = -nom_x[7] + true_x[7];
   out_2397415840402016948[8] = -nom_x[8] + true_x[8];
   out_2397415840402016948[9] = -nom_x[9] + true_x[9];
   out_2397415840402016948[10] = -nom_x[10] + true_x[10];
   out_2397415840402016948[11] = -nom_x[11] + true_x[11];
   out_2397415840402016948[12] = -nom_x[12] + true_x[12];
   out_2397415840402016948[13] = -nom_x[13] + true_x[13];
   out_2397415840402016948[14] = -nom_x[14] + true_x[14];
   out_2397415840402016948[15] = -nom_x[15] + true_x[15];
   out_2397415840402016948[16] = -nom_x[16] + true_x[16];
   out_2397415840402016948[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2601610123313846998) {
   out_2601610123313846998[0] = 1.0;
   out_2601610123313846998[1] = 0.0;
   out_2601610123313846998[2] = 0.0;
   out_2601610123313846998[3] = 0.0;
   out_2601610123313846998[4] = 0.0;
   out_2601610123313846998[5] = 0.0;
   out_2601610123313846998[6] = 0.0;
   out_2601610123313846998[7] = 0.0;
   out_2601610123313846998[8] = 0.0;
   out_2601610123313846998[9] = 0.0;
   out_2601610123313846998[10] = 0.0;
   out_2601610123313846998[11] = 0.0;
   out_2601610123313846998[12] = 0.0;
   out_2601610123313846998[13] = 0.0;
   out_2601610123313846998[14] = 0.0;
   out_2601610123313846998[15] = 0.0;
   out_2601610123313846998[16] = 0.0;
   out_2601610123313846998[17] = 0.0;
   out_2601610123313846998[18] = 0.0;
   out_2601610123313846998[19] = 1.0;
   out_2601610123313846998[20] = 0.0;
   out_2601610123313846998[21] = 0.0;
   out_2601610123313846998[22] = 0.0;
   out_2601610123313846998[23] = 0.0;
   out_2601610123313846998[24] = 0.0;
   out_2601610123313846998[25] = 0.0;
   out_2601610123313846998[26] = 0.0;
   out_2601610123313846998[27] = 0.0;
   out_2601610123313846998[28] = 0.0;
   out_2601610123313846998[29] = 0.0;
   out_2601610123313846998[30] = 0.0;
   out_2601610123313846998[31] = 0.0;
   out_2601610123313846998[32] = 0.0;
   out_2601610123313846998[33] = 0.0;
   out_2601610123313846998[34] = 0.0;
   out_2601610123313846998[35] = 0.0;
   out_2601610123313846998[36] = 0.0;
   out_2601610123313846998[37] = 0.0;
   out_2601610123313846998[38] = 1.0;
   out_2601610123313846998[39] = 0.0;
   out_2601610123313846998[40] = 0.0;
   out_2601610123313846998[41] = 0.0;
   out_2601610123313846998[42] = 0.0;
   out_2601610123313846998[43] = 0.0;
   out_2601610123313846998[44] = 0.0;
   out_2601610123313846998[45] = 0.0;
   out_2601610123313846998[46] = 0.0;
   out_2601610123313846998[47] = 0.0;
   out_2601610123313846998[48] = 0.0;
   out_2601610123313846998[49] = 0.0;
   out_2601610123313846998[50] = 0.0;
   out_2601610123313846998[51] = 0.0;
   out_2601610123313846998[52] = 0.0;
   out_2601610123313846998[53] = 0.0;
   out_2601610123313846998[54] = 0.0;
   out_2601610123313846998[55] = 0.0;
   out_2601610123313846998[56] = 0.0;
   out_2601610123313846998[57] = 1.0;
   out_2601610123313846998[58] = 0.0;
   out_2601610123313846998[59] = 0.0;
   out_2601610123313846998[60] = 0.0;
   out_2601610123313846998[61] = 0.0;
   out_2601610123313846998[62] = 0.0;
   out_2601610123313846998[63] = 0.0;
   out_2601610123313846998[64] = 0.0;
   out_2601610123313846998[65] = 0.0;
   out_2601610123313846998[66] = 0.0;
   out_2601610123313846998[67] = 0.0;
   out_2601610123313846998[68] = 0.0;
   out_2601610123313846998[69] = 0.0;
   out_2601610123313846998[70] = 0.0;
   out_2601610123313846998[71] = 0.0;
   out_2601610123313846998[72] = 0.0;
   out_2601610123313846998[73] = 0.0;
   out_2601610123313846998[74] = 0.0;
   out_2601610123313846998[75] = 0.0;
   out_2601610123313846998[76] = 1.0;
   out_2601610123313846998[77] = 0.0;
   out_2601610123313846998[78] = 0.0;
   out_2601610123313846998[79] = 0.0;
   out_2601610123313846998[80] = 0.0;
   out_2601610123313846998[81] = 0.0;
   out_2601610123313846998[82] = 0.0;
   out_2601610123313846998[83] = 0.0;
   out_2601610123313846998[84] = 0.0;
   out_2601610123313846998[85] = 0.0;
   out_2601610123313846998[86] = 0.0;
   out_2601610123313846998[87] = 0.0;
   out_2601610123313846998[88] = 0.0;
   out_2601610123313846998[89] = 0.0;
   out_2601610123313846998[90] = 0.0;
   out_2601610123313846998[91] = 0.0;
   out_2601610123313846998[92] = 0.0;
   out_2601610123313846998[93] = 0.0;
   out_2601610123313846998[94] = 0.0;
   out_2601610123313846998[95] = 1.0;
   out_2601610123313846998[96] = 0.0;
   out_2601610123313846998[97] = 0.0;
   out_2601610123313846998[98] = 0.0;
   out_2601610123313846998[99] = 0.0;
   out_2601610123313846998[100] = 0.0;
   out_2601610123313846998[101] = 0.0;
   out_2601610123313846998[102] = 0.0;
   out_2601610123313846998[103] = 0.0;
   out_2601610123313846998[104] = 0.0;
   out_2601610123313846998[105] = 0.0;
   out_2601610123313846998[106] = 0.0;
   out_2601610123313846998[107] = 0.0;
   out_2601610123313846998[108] = 0.0;
   out_2601610123313846998[109] = 0.0;
   out_2601610123313846998[110] = 0.0;
   out_2601610123313846998[111] = 0.0;
   out_2601610123313846998[112] = 0.0;
   out_2601610123313846998[113] = 0.0;
   out_2601610123313846998[114] = 1.0;
   out_2601610123313846998[115] = 0.0;
   out_2601610123313846998[116] = 0.0;
   out_2601610123313846998[117] = 0.0;
   out_2601610123313846998[118] = 0.0;
   out_2601610123313846998[119] = 0.0;
   out_2601610123313846998[120] = 0.0;
   out_2601610123313846998[121] = 0.0;
   out_2601610123313846998[122] = 0.0;
   out_2601610123313846998[123] = 0.0;
   out_2601610123313846998[124] = 0.0;
   out_2601610123313846998[125] = 0.0;
   out_2601610123313846998[126] = 0.0;
   out_2601610123313846998[127] = 0.0;
   out_2601610123313846998[128] = 0.0;
   out_2601610123313846998[129] = 0.0;
   out_2601610123313846998[130] = 0.0;
   out_2601610123313846998[131] = 0.0;
   out_2601610123313846998[132] = 0.0;
   out_2601610123313846998[133] = 1.0;
   out_2601610123313846998[134] = 0.0;
   out_2601610123313846998[135] = 0.0;
   out_2601610123313846998[136] = 0.0;
   out_2601610123313846998[137] = 0.0;
   out_2601610123313846998[138] = 0.0;
   out_2601610123313846998[139] = 0.0;
   out_2601610123313846998[140] = 0.0;
   out_2601610123313846998[141] = 0.0;
   out_2601610123313846998[142] = 0.0;
   out_2601610123313846998[143] = 0.0;
   out_2601610123313846998[144] = 0.0;
   out_2601610123313846998[145] = 0.0;
   out_2601610123313846998[146] = 0.0;
   out_2601610123313846998[147] = 0.0;
   out_2601610123313846998[148] = 0.0;
   out_2601610123313846998[149] = 0.0;
   out_2601610123313846998[150] = 0.0;
   out_2601610123313846998[151] = 0.0;
   out_2601610123313846998[152] = 1.0;
   out_2601610123313846998[153] = 0.0;
   out_2601610123313846998[154] = 0.0;
   out_2601610123313846998[155] = 0.0;
   out_2601610123313846998[156] = 0.0;
   out_2601610123313846998[157] = 0.0;
   out_2601610123313846998[158] = 0.0;
   out_2601610123313846998[159] = 0.0;
   out_2601610123313846998[160] = 0.0;
   out_2601610123313846998[161] = 0.0;
   out_2601610123313846998[162] = 0.0;
   out_2601610123313846998[163] = 0.0;
   out_2601610123313846998[164] = 0.0;
   out_2601610123313846998[165] = 0.0;
   out_2601610123313846998[166] = 0.0;
   out_2601610123313846998[167] = 0.0;
   out_2601610123313846998[168] = 0.0;
   out_2601610123313846998[169] = 0.0;
   out_2601610123313846998[170] = 0.0;
   out_2601610123313846998[171] = 1.0;
   out_2601610123313846998[172] = 0.0;
   out_2601610123313846998[173] = 0.0;
   out_2601610123313846998[174] = 0.0;
   out_2601610123313846998[175] = 0.0;
   out_2601610123313846998[176] = 0.0;
   out_2601610123313846998[177] = 0.0;
   out_2601610123313846998[178] = 0.0;
   out_2601610123313846998[179] = 0.0;
   out_2601610123313846998[180] = 0.0;
   out_2601610123313846998[181] = 0.0;
   out_2601610123313846998[182] = 0.0;
   out_2601610123313846998[183] = 0.0;
   out_2601610123313846998[184] = 0.0;
   out_2601610123313846998[185] = 0.0;
   out_2601610123313846998[186] = 0.0;
   out_2601610123313846998[187] = 0.0;
   out_2601610123313846998[188] = 0.0;
   out_2601610123313846998[189] = 0.0;
   out_2601610123313846998[190] = 1.0;
   out_2601610123313846998[191] = 0.0;
   out_2601610123313846998[192] = 0.0;
   out_2601610123313846998[193] = 0.0;
   out_2601610123313846998[194] = 0.0;
   out_2601610123313846998[195] = 0.0;
   out_2601610123313846998[196] = 0.0;
   out_2601610123313846998[197] = 0.0;
   out_2601610123313846998[198] = 0.0;
   out_2601610123313846998[199] = 0.0;
   out_2601610123313846998[200] = 0.0;
   out_2601610123313846998[201] = 0.0;
   out_2601610123313846998[202] = 0.0;
   out_2601610123313846998[203] = 0.0;
   out_2601610123313846998[204] = 0.0;
   out_2601610123313846998[205] = 0.0;
   out_2601610123313846998[206] = 0.0;
   out_2601610123313846998[207] = 0.0;
   out_2601610123313846998[208] = 0.0;
   out_2601610123313846998[209] = 1.0;
   out_2601610123313846998[210] = 0.0;
   out_2601610123313846998[211] = 0.0;
   out_2601610123313846998[212] = 0.0;
   out_2601610123313846998[213] = 0.0;
   out_2601610123313846998[214] = 0.0;
   out_2601610123313846998[215] = 0.0;
   out_2601610123313846998[216] = 0.0;
   out_2601610123313846998[217] = 0.0;
   out_2601610123313846998[218] = 0.0;
   out_2601610123313846998[219] = 0.0;
   out_2601610123313846998[220] = 0.0;
   out_2601610123313846998[221] = 0.0;
   out_2601610123313846998[222] = 0.0;
   out_2601610123313846998[223] = 0.0;
   out_2601610123313846998[224] = 0.0;
   out_2601610123313846998[225] = 0.0;
   out_2601610123313846998[226] = 0.0;
   out_2601610123313846998[227] = 0.0;
   out_2601610123313846998[228] = 1.0;
   out_2601610123313846998[229] = 0.0;
   out_2601610123313846998[230] = 0.0;
   out_2601610123313846998[231] = 0.0;
   out_2601610123313846998[232] = 0.0;
   out_2601610123313846998[233] = 0.0;
   out_2601610123313846998[234] = 0.0;
   out_2601610123313846998[235] = 0.0;
   out_2601610123313846998[236] = 0.0;
   out_2601610123313846998[237] = 0.0;
   out_2601610123313846998[238] = 0.0;
   out_2601610123313846998[239] = 0.0;
   out_2601610123313846998[240] = 0.0;
   out_2601610123313846998[241] = 0.0;
   out_2601610123313846998[242] = 0.0;
   out_2601610123313846998[243] = 0.0;
   out_2601610123313846998[244] = 0.0;
   out_2601610123313846998[245] = 0.0;
   out_2601610123313846998[246] = 0.0;
   out_2601610123313846998[247] = 1.0;
   out_2601610123313846998[248] = 0.0;
   out_2601610123313846998[249] = 0.0;
   out_2601610123313846998[250] = 0.0;
   out_2601610123313846998[251] = 0.0;
   out_2601610123313846998[252] = 0.0;
   out_2601610123313846998[253] = 0.0;
   out_2601610123313846998[254] = 0.0;
   out_2601610123313846998[255] = 0.0;
   out_2601610123313846998[256] = 0.0;
   out_2601610123313846998[257] = 0.0;
   out_2601610123313846998[258] = 0.0;
   out_2601610123313846998[259] = 0.0;
   out_2601610123313846998[260] = 0.0;
   out_2601610123313846998[261] = 0.0;
   out_2601610123313846998[262] = 0.0;
   out_2601610123313846998[263] = 0.0;
   out_2601610123313846998[264] = 0.0;
   out_2601610123313846998[265] = 0.0;
   out_2601610123313846998[266] = 1.0;
   out_2601610123313846998[267] = 0.0;
   out_2601610123313846998[268] = 0.0;
   out_2601610123313846998[269] = 0.0;
   out_2601610123313846998[270] = 0.0;
   out_2601610123313846998[271] = 0.0;
   out_2601610123313846998[272] = 0.0;
   out_2601610123313846998[273] = 0.0;
   out_2601610123313846998[274] = 0.0;
   out_2601610123313846998[275] = 0.0;
   out_2601610123313846998[276] = 0.0;
   out_2601610123313846998[277] = 0.0;
   out_2601610123313846998[278] = 0.0;
   out_2601610123313846998[279] = 0.0;
   out_2601610123313846998[280] = 0.0;
   out_2601610123313846998[281] = 0.0;
   out_2601610123313846998[282] = 0.0;
   out_2601610123313846998[283] = 0.0;
   out_2601610123313846998[284] = 0.0;
   out_2601610123313846998[285] = 1.0;
   out_2601610123313846998[286] = 0.0;
   out_2601610123313846998[287] = 0.0;
   out_2601610123313846998[288] = 0.0;
   out_2601610123313846998[289] = 0.0;
   out_2601610123313846998[290] = 0.0;
   out_2601610123313846998[291] = 0.0;
   out_2601610123313846998[292] = 0.0;
   out_2601610123313846998[293] = 0.0;
   out_2601610123313846998[294] = 0.0;
   out_2601610123313846998[295] = 0.0;
   out_2601610123313846998[296] = 0.0;
   out_2601610123313846998[297] = 0.0;
   out_2601610123313846998[298] = 0.0;
   out_2601610123313846998[299] = 0.0;
   out_2601610123313846998[300] = 0.0;
   out_2601610123313846998[301] = 0.0;
   out_2601610123313846998[302] = 0.0;
   out_2601610123313846998[303] = 0.0;
   out_2601610123313846998[304] = 1.0;
   out_2601610123313846998[305] = 0.0;
   out_2601610123313846998[306] = 0.0;
   out_2601610123313846998[307] = 0.0;
   out_2601610123313846998[308] = 0.0;
   out_2601610123313846998[309] = 0.0;
   out_2601610123313846998[310] = 0.0;
   out_2601610123313846998[311] = 0.0;
   out_2601610123313846998[312] = 0.0;
   out_2601610123313846998[313] = 0.0;
   out_2601610123313846998[314] = 0.0;
   out_2601610123313846998[315] = 0.0;
   out_2601610123313846998[316] = 0.0;
   out_2601610123313846998[317] = 0.0;
   out_2601610123313846998[318] = 0.0;
   out_2601610123313846998[319] = 0.0;
   out_2601610123313846998[320] = 0.0;
   out_2601610123313846998[321] = 0.0;
   out_2601610123313846998[322] = 0.0;
   out_2601610123313846998[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4760475200250568711) {
   out_4760475200250568711[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4760475200250568711[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4760475200250568711[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4760475200250568711[3] = dt*state[12] + state[3];
   out_4760475200250568711[4] = dt*state[13] + state[4];
   out_4760475200250568711[5] = dt*state[14] + state[5];
   out_4760475200250568711[6] = state[6];
   out_4760475200250568711[7] = state[7];
   out_4760475200250568711[8] = state[8];
   out_4760475200250568711[9] = state[9];
   out_4760475200250568711[10] = state[10];
   out_4760475200250568711[11] = state[11];
   out_4760475200250568711[12] = state[12];
   out_4760475200250568711[13] = state[13];
   out_4760475200250568711[14] = state[14];
   out_4760475200250568711[15] = state[15];
   out_4760475200250568711[16] = state[16];
   out_4760475200250568711[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3582442533271693884) {
   out_3582442533271693884[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3582442533271693884[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3582442533271693884[2] = 0;
   out_3582442533271693884[3] = 0;
   out_3582442533271693884[4] = 0;
   out_3582442533271693884[5] = 0;
   out_3582442533271693884[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3582442533271693884[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3582442533271693884[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3582442533271693884[9] = 0;
   out_3582442533271693884[10] = 0;
   out_3582442533271693884[11] = 0;
   out_3582442533271693884[12] = 0;
   out_3582442533271693884[13] = 0;
   out_3582442533271693884[14] = 0;
   out_3582442533271693884[15] = 0;
   out_3582442533271693884[16] = 0;
   out_3582442533271693884[17] = 0;
   out_3582442533271693884[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3582442533271693884[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3582442533271693884[20] = 0;
   out_3582442533271693884[21] = 0;
   out_3582442533271693884[22] = 0;
   out_3582442533271693884[23] = 0;
   out_3582442533271693884[24] = 0;
   out_3582442533271693884[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3582442533271693884[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3582442533271693884[27] = 0;
   out_3582442533271693884[28] = 0;
   out_3582442533271693884[29] = 0;
   out_3582442533271693884[30] = 0;
   out_3582442533271693884[31] = 0;
   out_3582442533271693884[32] = 0;
   out_3582442533271693884[33] = 0;
   out_3582442533271693884[34] = 0;
   out_3582442533271693884[35] = 0;
   out_3582442533271693884[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3582442533271693884[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3582442533271693884[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3582442533271693884[39] = 0;
   out_3582442533271693884[40] = 0;
   out_3582442533271693884[41] = 0;
   out_3582442533271693884[42] = 0;
   out_3582442533271693884[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3582442533271693884[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3582442533271693884[45] = 0;
   out_3582442533271693884[46] = 0;
   out_3582442533271693884[47] = 0;
   out_3582442533271693884[48] = 0;
   out_3582442533271693884[49] = 0;
   out_3582442533271693884[50] = 0;
   out_3582442533271693884[51] = 0;
   out_3582442533271693884[52] = 0;
   out_3582442533271693884[53] = 0;
   out_3582442533271693884[54] = 0;
   out_3582442533271693884[55] = 0;
   out_3582442533271693884[56] = 0;
   out_3582442533271693884[57] = 1;
   out_3582442533271693884[58] = 0;
   out_3582442533271693884[59] = 0;
   out_3582442533271693884[60] = 0;
   out_3582442533271693884[61] = 0;
   out_3582442533271693884[62] = 0;
   out_3582442533271693884[63] = 0;
   out_3582442533271693884[64] = 0;
   out_3582442533271693884[65] = 0;
   out_3582442533271693884[66] = dt;
   out_3582442533271693884[67] = 0;
   out_3582442533271693884[68] = 0;
   out_3582442533271693884[69] = 0;
   out_3582442533271693884[70] = 0;
   out_3582442533271693884[71] = 0;
   out_3582442533271693884[72] = 0;
   out_3582442533271693884[73] = 0;
   out_3582442533271693884[74] = 0;
   out_3582442533271693884[75] = 0;
   out_3582442533271693884[76] = 1;
   out_3582442533271693884[77] = 0;
   out_3582442533271693884[78] = 0;
   out_3582442533271693884[79] = 0;
   out_3582442533271693884[80] = 0;
   out_3582442533271693884[81] = 0;
   out_3582442533271693884[82] = 0;
   out_3582442533271693884[83] = 0;
   out_3582442533271693884[84] = 0;
   out_3582442533271693884[85] = dt;
   out_3582442533271693884[86] = 0;
   out_3582442533271693884[87] = 0;
   out_3582442533271693884[88] = 0;
   out_3582442533271693884[89] = 0;
   out_3582442533271693884[90] = 0;
   out_3582442533271693884[91] = 0;
   out_3582442533271693884[92] = 0;
   out_3582442533271693884[93] = 0;
   out_3582442533271693884[94] = 0;
   out_3582442533271693884[95] = 1;
   out_3582442533271693884[96] = 0;
   out_3582442533271693884[97] = 0;
   out_3582442533271693884[98] = 0;
   out_3582442533271693884[99] = 0;
   out_3582442533271693884[100] = 0;
   out_3582442533271693884[101] = 0;
   out_3582442533271693884[102] = 0;
   out_3582442533271693884[103] = 0;
   out_3582442533271693884[104] = dt;
   out_3582442533271693884[105] = 0;
   out_3582442533271693884[106] = 0;
   out_3582442533271693884[107] = 0;
   out_3582442533271693884[108] = 0;
   out_3582442533271693884[109] = 0;
   out_3582442533271693884[110] = 0;
   out_3582442533271693884[111] = 0;
   out_3582442533271693884[112] = 0;
   out_3582442533271693884[113] = 0;
   out_3582442533271693884[114] = 1;
   out_3582442533271693884[115] = 0;
   out_3582442533271693884[116] = 0;
   out_3582442533271693884[117] = 0;
   out_3582442533271693884[118] = 0;
   out_3582442533271693884[119] = 0;
   out_3582442533271693884[120] = 0;
   out_3582442533271693884[121] = 0;
   out_3582442533271693884[122] = 0;
   out_3582442533271693884[123] = 0;
   out_3582442533271693884[124] = 0;
   out_3582442533271693884[125] = 0;
   out_3582442533271693884[126] = 0;
   out_3582442533271693884[127] = 0;
   out_3582442533271693884[128] = 0;
   out_3582442533271693884[129] = 0;
   out_3582442533271693884[130] = 0;
   out_3582442533271693884[131] = 0;
   out_3582442533271693884[132] = 0;
   out_3582442533271693884[133] = 1;
   out_3582442533271693884[134] = 0;
   out_3582442533271693884[135] = 0;
   out_3582442533271693884[136] = 0;
   out_3582442533271693884[137] = 0;
   out_3582442533271693884[138] = 0;
   out_3582442533271693884[139] = 0;
   out_3582442533271693884[140] = 0;
   out_3582442533271693884[141] = 0;
   out_3582442533271693884[142] = 0;
   out_3582442533271693884[143] = 0;
   out_3582442533271693884[144] = 0;
   out_3582442533271693884[145] = 0;
   out_3582442533271693884[146] = 0;
   out_3582442533271693884[147] = 0;
   out_3582442533271693884[148] = 0;
   out_3582442533271693884[149] = 0;
   out_3582442533271693884[150] = 0;
   out_3582442533271693884[151] = 0;
   out_3582442533271693884[152] = 1;
   out_3582442533271693884[153] = 0;
   out_3582442533271693884[154] = 0;
   out_3582442533271693884[155] = 0;
   out_3582442533271693884[156] = 0;
   out_3582442533271693884[157] = 0;
   out_3582442533271693884[158] = 0;
   out_3582442533271693884[159] = 0;
   out_3582442533271693884[160] = 0;
   out_3582442533271693884[161] = 0;
   out_3582442533271693884[162] = 0;
   out_3582442533271693884[163] = 0;
   out_3582442533271693884[164] = 0;
   out_3582442533271693884[165] = 0;
   out_3582442533271693884[166] = 0;
   out_3582442533271693884[167] = 0;
   out_3582442533271693884[168] = 0;
   out_3582442533271693884[169] = 0;
   out_3582442533271693884[170] = 0;
   out_3582442533271693884[171] = 1;
   out_3582442533271693884[172] = 0;
   out_3582442533271693884[173] = 0;
   out_3582442533271693884[174] = 0;
   out_3582442533271693884[175] = 0;
   out_3582442533271693884[176] = 0;
   out_3582442533271693884[177] = 0;
   out_3582442533271693884[178] = 0;
   out_3582442533271693884[179] = 0;
   out_3582442533271693884[180] = 0;
   out_3582442533271693884[181] = 0;
   out_3582442533271693884[182] = 0;
   out_3582442533271693884[183] = 0;
   out_3582442533271693884[184] = 0;
   out_3582442533271693884[185] = 0;
   out_3582442533271693884[186] = 0;
   out_3582442533271693884[187] = 0;
   out_3582442533271693884[188] = 0;
   out_3582442533271693884[189] = 0;
   out_3582442533271693884[190] = 1;
   out_3582442533271693884[191] = 0;
   out_3582442533271693884[192] = 0;
   out_3582442533271693884[193] = 0;
   out_3582442533271693884[194] = 0;
   out_3582442533271693884[195] = 0;
   out_3582442533271693884[196] = 0;
   out_3582442533271693884[197] = 0;
   out_3582442533271693884[198] = 0;
   out_3582442533271693884[199] = 0;
   out_3582442533271693884[200] = 0;
   out_3582442533271693884[201] = 0;
   out_3582442533271693884[202] = 0;
   out_3582442533271693884[203] = 0;
   out_3582442533271693884[204] = 0;
   out_3582442533271693884[205] = 0;
   out_3582442533271693884[206] = 0;
   out_3582442533271693884[207] = 0;
   out_3582442533271693884[208] = 0;
   out_3582442533271693884[209] = 1;
   out_3582442533271693884[210] = 0;
   out_3582442533271693884[211] = 0;
   out_3582442533271693884[212] = 0;
   out_3582442533271693884[213] = 0;
   out_3582442533271693884[214] = 0;
   out_3582442533271693884[215] = 0;
   out_3582442533271693884[216] = 0;
   out_3582442533271693884[217] = 0;
   out_3582442533271693884[218] = 0;
   out_3582442533271693884[219] = 0;
   out_3582442533271693884[220] = 0;
   out_3582442533271693884[221] = 0;
   out_3582442533271693884[222] = 0;
   out_3582442533271693884[223] = 0;
   out_3582442533271693884[224] = 0;
   out_3582442533271693884[225] = 0;
   out_3582442533271693884[226] = 0;
   out_3582442533271693884[227] = 0;
   out_3582442533271693884[228] = 1;
   out_3582442533271693884[229] = 0;
   out_3582442533271693884[230] = 0;
   out_3582442533271693884[231] = 0;
   out_3582442533271693884[232] = 0;
   out_3582442533271693884[233] = 0;
   out_3582442533271693884[234] = 0;
   out_3582442533271693884[235] = 0;
   out_3582442533271693884[236] = 0;
   out_3582442533271693884[237] = 0;
   out_3582442533271693884[238] = 0;
   out_3582442533271693884[239] = 0;
   out_3582442533271693884[240] = 0;
   out_3582442533271693884[241] = 0;
   out_3582442533271693884[242] = 0;
   out_3582442533271693884[243] = 0;
   out_3582442533271693884[244] = 0;
   out_3582442533271693884[245] = 0;
   out_3582442533271693884[246] = 0;
   out_3582442533271693884[247] = 1;
   out_3582442533271693884[248] = 0;
   out_3582442533271693884[249] = 0;
   out_3582442533271693884[250] = 0;
   out_3582442533271693884[251] = 0;
   out_3582442533271693884[252] = 0;
   out_3582442533271693884[253] = 0;
   out_3582442533271693884[254] = 0;
   out_3582442533271693884[255] = 0;
   out_3582442533271693884[256] = 0;
   out_3582442533271693884[257] = 0;
   out_3582442533271693884[258] = 0;
   out_3582442533271693884[259] = 0;
   out_3582442533271693884[260] = 0;
   out_3582442533271693884[261] = 0;
   out_3582442533271693884[262] = 0;
   out_3582442533271693884[263] = 0;
   out_3582442533271693884[264] = 0;
   out_3582442533271693884[265] = 0;
   out_3582442533271693884[266] = 1;
   out_3582442533271693884[267] = 0;
   out_3582442533271693884[268] = 0;
   out_3582442533271693884[269] = 0;
   out_3582442533271693884[270] = 0;
   out_3582442533271693884[271] = 0;
   out_3582442533271693884[272] = 0;
   out_3582442533271693884[273] = 0;
   out_3582442533271693884[274] = 0;
   out_3582442533271693884[275] = 0;
   out_3582442533271693884[276] = 0;
   out_3582442533271693884[277] = 0;
   out_3582442533271693884[278] = 0;
   out_3582442533271693884[279] = 0;
   out_3582442533271693884[280] = 0;
   out_3582442533271693884[281] = 0;
   out_3582442533271693884[282] = 0;
   out_3582442533271693884[283] = 0;
   out_3582442533271693884[284] = 0;
   out_3582442533271693884[285] = 1;
   out_3582442533271693884[286] = 0;
   out_3582442533271693884[287] = 0;
   out_3582442533271693884[288] = 0;
   out_3582442533271693884[289] = 0;
   out_3582442533271693884[290] = 0;
   out_3582442533271693884[291] = 0;
   out_3582442533271693884[292] = 0;
   out_3582442533271693884[293] = 0;
   out_3582442533271693884[294] = 0;
   out_3582442533271693884[295] = 0;
   out_3582442533271693884[296] = 0;
   out_3582442533271693884[297] = 0;
   out_3582442533271693884[298] = 0;
   out_3582442533271693884[299] = 0;
   out_3582442533271693884[300] = 0;
   out_3582442533271693884[301] = 0;
   out_3582442533271693884[302] = 0;
   out_3582442533271693884[303] = 0;
   out_3582442533271693884[304] = 1;
   out_3582442533271693884[305] = 0;
   out_3582442533271693884[306] = 0;
   out_3582442533271693884[307] = 0;
   out_3582442533271693884[308] = 0;
   out_3582442533271693884[309] = 0;
   out_3582442533271693884[310] = 0;
   out_3582442533271693884[311] = 0;
   out_3582442533271693884[312] = 0;
   out_3582442533271693884[313] = 0;
   out_3582442533271693884[314] = 0;
   out_3582442533271693884[315] = 0;
   out_3582442533271693884[316] = 0;
   out_3582442533271693884[317] = 0;
   out_3582442533271693884[318] = 0;
   out_3582442533271693884[319] = 0;
   out_3582442533271693884[320] = 0;
   out_3582442533271693884[321] = 0;
   out_3582442533271693884[322] = 0;
   out_3582442533271693884[323] = 1;
}
void h_4(double *state, double *unused, double *out_2509617360494600185) {
   out_2509617360494600185[0] = state[6] + state[9];
   out_2509617360494600185[1] = state[7] + state[10];
   out_2509617360494600185[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7754128508352927733) {
   out_7754128508352927733[0] = 0;
   out_7754128508352927733[1] = 0;
   out_7754128508352927733[2] = 0;
   out_7754128508352927733[3] = 0;
   out_7754128508352927733[4] = 0;
   out_7754128508352927733[5] = 0;
   out_7754128508352927733[6] = 1;
   out_7754128508352927733[7] = 0;
   out_7754128508352927733[8] = 0;
   out_7754128508352927733[9] = 1;
   out_7754128508352927733[10] = 0;
   out_7754128508352927733[11] = 0;
   out_7754128508352927733[12] = 0;
   out_7754128508352927733[13] = 0;
   out_7754128508352927733[14] = 0;
   out_7754128508352927733[15] = 0;
   out_7754128508352927733[16] = 0;
   out_7754128508352927733[17] = 0;
   out_7754128508352927733[18] = 0;
   out_7754128508352927733[19] = 0;
   out_7754128508352927733[20] = 0;
   out_7754128508352927733[21] = 0;
   out_7754128508352927733[22] = 0;
   out_7754128508352927733[23] = 0;
   out_7754128508352927733[24] = 0;
   out_7754128508352927733[25] = 1;
   out_7754128508352927733[26] = 0;
   out_7754128508352927733[27] = 0;
   out_7754128508352927733[28] = 1;
   out_7754128508352927733[29] = 0;
   out_7754128508352927733[30] = 0;
   out_7754128508352927733[31] = 0;
   out_7754128508352927733[32] = 0;
   out_7754128508352927733[33] = 0;
   out_7754128508352927733[34] = 0;
   out_7754128508352927733[35] = 0;
   out_7754128508352927733[36] = 0;
   out_7754128508352927733[37] = 0;
   out_7754128508352927733[38] = 0;
   out_7754128508352927733[39] = 0;
   out_7754128508352927733[40] = 0;
   out_7754128508352927733[41] = 0;
   out_7754128508352927733[42] = 0;
   out_7754128508352927733[43] = 0;
   out_7754128508352927733[44] = 1;
   out_7754128508352927733[45] = 0;
   out_7754128508352927733[46] = 0;
   out_7754128508352927733[47] = 1;
   out_7754128508352927733[48] = 0;
   out_7754128508352927733[49] = 0;
   out_7754128508352927733[50] = 0;
   out_7754128508352927733[51] = 0;
   out_7754128508352927733[52] = 0;
   out_7754128508352927733[53] = 0;
}
void h_10(double *state, double *unused, double *out_8277461064165376406) {
   out_8277461064165376406[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8277461064165376406[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8277461064165376406[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1041335713495420488) {
   out_1041335713495420488[0] = 0;
   out_1041335713495420488[1] = 9.8100000000000005*cos(state[1]);
   out_1041335713495420488[2] = 0;
   out_1041335713495420488[3] = 0;
   out_1041335713495420488[4] = -state[8];
   out_1041335713495420488[5] = state[7];
   out_1041335713495420488[6] = 0;
   out_1041335713495420488[7] = state[5];
   out_1041335713495420488[8] = -state[4];
   out_1041335713495420488[9] = 0;
   out_1041335713495420488[10] = 0;
   out_1041335713495420488[11] = 0;
   out_1041335713495420488[12] = 1;
   out_1041335713495420488[13] = 0;
   out_1041335713495420488[14] = 0;
   out_1041335713495420488[15] = 1;
   out_1041335713495420488[16] = 0;
   out_1041335713495420488[17] = 0;
   out_1041335713495420488[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1041335713495420488[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1041335713495420488[20] = 0;
   out_1041335713495420488[21] = state[8];
   out_1041335713495420488[22] = 0;
   out_1041335713495420488[23] = -state[6];
   out_1041335713495420488[24] = -state[5];
   out_1041335713495420488[25] = 0;
   out_1041335713495420488[26] = state[3];
   out_1041335713495420488[27] = 0;
   out_1041335713495420488[28] = 0;
   out_1041335713495420488[29] = 0;
   out_1041335713495420488[30] = 0;
   out_1041335713495420488[31] = 1;
   out_1041335713495420488[32] = 0;
   out_1041335713495420488[33] = 0;
   out_1041335713495420488[34] = 1;
   out_1041335713495420488[35] = 0;
   out_1041335713495420488[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1041335713495420488[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1041335713495420488[38] = 0;
   out_1041335713495420488[39] = -state[7];
   out_1041335713495420488[40] = state[6];
   out_1041335713495420488[41] = 0;
   out_1041335713495420488[42] = state[4];
   out_1041335713495420488[43] = -state[3];
   out_1041335713495420488[44] = 0;
   out_1041335713495420488[45] = 0;
   out_1041335713495420488[46] = 0;
   out_1041335713495420488[47] = 0;
   out_1041335713495420488[48] = 0;
   out_1041335713495420488[49] = 0;
   out_1041335713495420488[50] = 1;
   out_1041335713495420488[51] = 0;
   out_1041335713495420488[52] = 0;
   out_1041335713495420488[53] = 1;
}
void h_13(double *state, double *unused, double *out_5178744377390213500) {
   out_5178744377390213500[0] = state[3];
   out_5178744377390213500[1] = state[4];
   out_5178744377390213500[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3920373045050403709) {
   out_3920373045050403709[0] = 0;
   out_3920373045050403709[1] = 0;
   out_3920373045050403709[2] = 0;
   out_3920373045050403709[3] = 1;
   out_3920373045050403709[4] = 0;
   out_3920373045050403709[5] = 0;
   out_3920373045050403709[6] = 0;
   out_3920373045050403709[7] = 0;
   out_3920373045050403709[8] = 0;
   out_3920373045050403709[9] = 0;
   out_3920373045050403709[10] = 0;
   out_3920373045050403709[11] = 0;
   out_3920373045050403709[12] = 0;
   out_3920373045050403709[13] = 0;
   out_3920373045050403709[14] = 0;
   out_3920373045050403709[15] = 0;
   out_3920373045050403709[16] = 0;
   out_3920373045050403709[17] = 0;
   out_3920373045050403709[18] = 0;
   out_3920373045050403709[19] = 0;
   out_3920373045050403709[20] = 0;
   out_3920373045050403709[21] = 0;
   out_3920373045050403709[22] = 1;
   out_3920373045050403709[23] = 0;
   out_3920373045050403709[24] = 0;
   out_3920373045050403709[25] = 0;
   out_3920373045050403709[26] = 0;
   out_3920373045050403709[27] = 0;
   out_3920373045050403709[28] = 0;
   out_3920373045050403709[29] = 0;
   out_3920373045050403709[30] = 0;
   out_3920373045050403709[31] = 0;
   out_3920373045050403709[32] = 0;
   out_3920373045050403709[33] = 0;
   out_3920373045050403709[34] = 0;
   out_3920373045050403709[35] = 0;
   out_3920373045050403709[36] = 0;
   out_3920373045050403709[37] = 0;
   out_3920373045050403709[38] = 0;
   out_3920373045050403709[39] = 0;
   out_3920373045050403709[40] = 0;
   out_3920373045050403709[41] = 1;
   out_3920373045050403709[42] = 0;
   out_3920373045050403709[43] = 0;
   out_3920373045050403709[44] = 0;
   out_3920373045050403709[45] = 0;
   out_3920373045050403709[46] = 0;
   out_3920373045050403709[47] = 0;
   out_3920373045050403709[48] = 0;
   out_3920373045050403709[49] = 0;
   out_3920373045050403709[50] = 0;
   out_3920373045050403709[51] = 0;
   out_3920373045050403709[52] = 0;
   out_3920373045050403709[53] = 0;
}
void h_14(double *state, double *unused, double *out_7113706599292876475) {
   out_7113706599292876475[0] = state[6];
   out_7113706599292876475[1] = state[7];
   out_7113706599292876475[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4671340076057555437) {
   out_4671340076057555437[0] = 0;
   out_4671340076057555437[1] = 0;
   out_4671340076057555437[2] = 0;
   out_4671340076057555437[3] = 0;
   out_4671340076057555437[4] = 0;
   out_4671340076057555437[5] = 0;
   out_4671340076057555437[6] = 1;
   out_4671340076057555437[7] = 0;
   out_4671340076057555437[8] = 0;
   out_4671340076057555437[9] = 0;
   out_4671340076057555437[10] = 0;
   out_4671340076057555437[11] = 0;
   out_4671340076057555437[12] = 0;
   out_4671340076057555437[13] = 0;
   out_4671340076057555437[14] = 0;
   out_4671340076057555437[15] = 0;
   out_4671340076057555437[16] = 0;
   out_4671340076057555437[17] = 0;
   out_4671340076057555437[18] = 0;
   out_4671340076057555437[19] = 0;
   out_4671340076057555437[20] = 0;
   out_4671340076057555437[21] = 0;
   out_4671340076057555437[22] = 0;
   out_4671340076057555437[23] = 0;
   out_4671340076057555437[24] = 0;
   out_4671340076057555437[25] = 1;
   out_4671340076057555437[26] = 0;
   out_4671340076057555437[27] = 0;
   out_4671340076057555437[28] = 0;
   out_4671340076057555437[29] = 0;
   out_4671340076057555437[30] = 0;
   out_4671340076057555437[31] = 0;
   out_4671340076057555437[32] = 0;
   out_4671340076057555437[33] = 0;
   out_4671340076057555437[34] = 0;
   out_4671340076057555437[35] = 0;
   out_4671340076057555437[36] = 0;
   out_4671340076057555437[37] = 0;
   out_4671340076057555437[38] = 0;
   out_4671340076057555437[39] = 0;
   out_4671340076057555437[40] = 0;
   out_4671340076057555437[41] = 0;
   out_4671340076057555437[42] = 0;
   out_4671340076057555437[43] = 0;
   out_4671340076057555437[44] = 1;
   out_4671340076057555437[45] = 0;
   out_4671340076057555437[46] = 0;
   out_4671340076057555437[47] = 0;
   out_4671340076057555437[48] = 0;
   out_4671340076057555437[49] = 0;
   out_4671340076057555437[50] = 0;
   out_4671340076057555437[51] = 0;
   out_4671340076057555437[52] = 0;
   out_4671340076057555437[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2691520434775550490) {
  err_fun(nom_x, delta_x, out_2691520434775550490);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2397415840402016948) {
  inv_err_fun(nom_x, true_x, out_2397415840402016948);
}
void pose_H_mod_fun(double *state, double *out_2601610123313846998) {
  H_mod_fun(state, out_2601610123313846998);
}
void pose_f_fun(double *state, double dt, double *out_4760475200250568711) {
  f_fun(state,  dt, out_4760475200250568711);
}
void pose_F_fun(double *state, double dt, double *out_3582442533271693884) {
  F_fun(state,  dt, out_3582442533271693884);
}
void pose_h_4(double *state, double *unused, double *out_2509617360494600185) {
  h_4(state, unused, out_2509617360494600185);
}
void pose_H_4(double *state, double *unused, double *out_7754128508352927733) {
  H_4(state, unused, out_7754128508352927733);
}
void pose_h_10(double *state, double *unused, double *out_8277461064165376406) {
  h_10(state, unused, out_8277461064165376406);
}
void pose_H_10(double *state, double *unused, double *out_1041335713495420488) {
  H_10(state, unused, out_1041335713495420488);
}
void pose_h_13(double *state, double *unused, double *out_5178744377390213500) {
  h_13(state, unused, out_5178744377390213500);
}
void pose_H_13(double *state, double *unused, double *out_3920373045050403709) {
  H_13(state, unused, out_3920373045050403709);
}
void pose_h_14(double *state, double *unused, double *out_7113706599292876475) {
  h_14(state, unused, out_7113706599292876475);
}
void pose_H_14(double *state, double *unused, double *out_4671340076057555437) {
  H_14(state, unused, out_4671340076057555437);
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
