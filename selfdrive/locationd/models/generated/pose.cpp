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
void err_fun(double *nom_x, double *delta_x, double *out_4864162633936306843) {
   out_4864162633936306843[0] = delta_x[0] + nom_x[0];
   out_4864162633936306843[1] = delta_x[1] + nom_x[1];
   out_4864162633936306843[2] = delta_x[2] + nom_x[2];
   out_4864162633936306843[3] = delta_x[3] + nom_x[3];
   out_4864162633936306843[4] = delta_x[4] + nom_x[4];
   out_4864162633936306843[5] = delta_x[5] + nom_x[5];
   out_4864162633936306843[6] = delta_x[6] + nom_x[6];
   out_4864162633936306843[7] = delta_x[7] + nom_x[7];
   out_4864162633936306843[8] = delta_x[8] + nom_x[8];
   out_4864162633936306843[9] = delta_x[9] + nom_x[9];
   out_4864162633936306843[10] = delta_x[10] + nom_x[10];
   out_4864162633936306843[11] = delta_x[11] + nom_x[11];
   out_4864162633936306843[12] = delta_x[12] + nom_x[12];
   out_4864162633936306843[13] = delta_x[13] + nom_x[13];
   out_4864162633936306843[14] = delta_x[14] + nom_x[14];
   out_4864162633936306843[15] = delta_x[15] + nom_x[15];
   out_4864162633936306843[16] = delta_x[16] + nom_x[16];
   out_4864162633936306843[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9066787371279446197) {
   out_9066787371279446197[0] = -nom_x[0] + true_x[0];
   out_9066787371279446197[1] = -nom_x[1] + true_x[1];
   out_9066787371279446197[2] = -nom_x[2] + true_x[2];
   out_9066787371279446197[3] = -nom_x[3] + true_x[3];
   out_9066787371279446197[4] = -nom_x[4] + true_x[4];
   out_9066787371279446197[5] = -nom_x[5] + true_x[5];
   out_9066787371279446197[6] = -nom_x[6] + true_x[6];
   out_9066787371279446197[7] = -nom_x[7] + true_x[7];
   out_9066787371279446197[8] = -nom_x[8] + true_x[8];
   out_9066787371279446197[9] = -nom_x[9] + true_x[9];
   out_9066787371279446197[10] = -nom_x[10] + true_x[10];
   out_9066787371279446197[11] = -nom_x[11] + true_x[11];
   out_9066787371279446197[12] = -nom_x[12] + true_x[12];
   out_9066787371279446197[13] = -nom_x[13] + true_x[13];
   out_9066787371279446197[14] = -nom_x[14] + true_x[14];
   out_9066787371279446197[15] = -nom_x[15] + true_x[15];
   out_9066787371279446197[16] = -nom_x[16] + true_x[16];
   out_9066787371279446197[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8494019208454287783) {
   out_8494019208454287783[0] = 1.0;
   out_8494019208454287783[1] = 0.0;
   out_8494019208454287783[2] = 0.0;
   out_8494019208454287783[3] = 0.0;
   out_8494019208454287783[4] = 0.0;
   out_8494019208454287783[5] = 0.0;
   out_8494019208454287783[6] = 0.0;
   out_8494019208454287783[7] = 0.0;
   out_8494019208454287783[8] = 0.0;
   out_8494019208454287783[9] = 0.0;
   out_8494019208454287783[10] = 0.0;
   out_8494019208454287783[11] = 0.0;
   out_8494019208454287783[12] = 0.0;
   out_8494019208454287783[13] = 0.0;
   out_8494019208454287783[14] = 0.0;
   out_8494019208454287783[15] = 0.0;
   out_8494019208454287783[16] = 0.0;
   out_8494019208454287783[17] = 0.0;
   out_8494019208454287783[18] = 0.0;
   out_8494019208454287783[19] = 1.0;
   out_8494019208454287783[20] = 0.0;
   out_8494019208454287783[21] = 0.0;
   out_8494019208454287783[22] = 0.0;
   out_8494019208454287783[23] = 0.0;
   out_8494019208454287783[24] = 0.0;
   out_8494019208454287783[25] = 0.0;
   out_8494019208454287783[26] = 0.0;
   out_8494019208454287783[27] = 0.0;
   out_8494019208454287783[28] = 0.0;
   out_8494019208454287783[29] = 0.0;
   out_8494019208454287783[30] = 0.0;
   out_8494019208454287783[31] = 0.0;
   out_8494019208454287783[32] = 0.0;
   out_8494019208454287783[33] = 0.0;
   out_8494019208454287783[34] = 0.0;
   out_8494019208454287783[35] = 0.0;
   out_8494019208454287783[36] = 0.0;
   out_8494019208454287783[37] = 0.0;
   out_8494019208454287783[38] = 1.0;
   out_8494019208454287783[39] = 0.0;
   out_8494019208454287783[40] = 0.0;
   out_8494019208454287783[41] = 0.0;
   out_8494019208454287783[42] = 0.0;
   out_8494019208454287783[43] = 0.0;
   out_8494019208454287783[44] = 0.0;
   out_8494019208454287783[45] = 0.0;
   out_8494019208454287783[46] = 0.0;
   out_8494019208454287783[47] = 0.0;
   out_8494019208454287783[48] = 0.0;
   out_8494019208454287783[49] = 0.0;
   out_8494019208454287783[50] = 0.0;
   out_8494019208454287783[51] = 0.0;
   out_8494019208454287783[52] = 0.0;
   out_8494019208454287783[53] = 0.0;
   out_8494019208454287783[54] = 0.0;
   out_8494019208454287783[55] = 0.0;
   out_8494019208454287783[56] = 0.0;
   out_8494019208454287783[57] = 1.0;
   out_8494019208454287783[58] = 0.0;
   out_8494019208454287783[59] = 0.0;
   out_8494019208454287783[60] = 0.0;
   out_8494019208454287783[61] = 0.0;
   out_8494019208454287783[62] = 0.0;
   out_8494019208454287783[63] = 0.0;
   out_8494019208454287783[64] = 0.0;
   out_8494019208454287783[65] = 0.0;
   out_8494019208454287783[66] = 0.0;
   out_8494019208454287783[67] = 0.0;
   out_8494019208454287783[68] = 0.0;
   out_8494019208454287783[69] = 0.0;
   out_8494019208454287783[70] = 0.0;
   out_8494019208454287783[71] = 0.0;
   out_8494019208454287783[72] = 0.0;
   out_8494019208454287783[73] = 0.0;
   out_8494019208454287783[74] = 0.0;
   out_8494019208454287783[75] = 0.0;
   out_8494019208454287783[76] = 1.0;
   out_8494019208454287783[77] = 0.0;
   out_8494019208454287783[78] = 0.0;
   out_8494019208454287783[79] = 0.0;
   out_8494019208454287783[80] = 0.0;
   out_8494019208454287783[81] = 0.0;
   out_8494019208454287783[82] = 0.0;
   out_8494019208454287783[83] = 0.0;
   out_8494019208454287783[84] = 0.0;
   out_8494019208454287783[85] = 0.0;
   out_8494019208454287783[86] = 0.0;
   out_8494019208454287783[87] = 0.0;
   out_8494019208454287783[88] = 0.0;
   out_8494019208454287783[89] = 0.0;
   out_8494019208454287783[90] = 0.0;
   out_8494019208454287783[91] = 0.0;
   out_8494019208454287783[92] = 0.0;
   out_8494019208454287783[93] = 0.0;
   out_8494019208454287783[94] = 0.0;
   out_8494019208454287783[95] = 1.0;
   out_8494019208454287783[96] = 0.0;
   out_8494019208454287783[97] = 0.0;
   out_8494019208454287783[98] = 0.0;
   out_8494019208454287783[99] = 0.0;
   out_8494019208454287783[100] = 0.0;
   out_8494019208454287783[101] = 0.0;
   out_8494019208454287783[102] = 0.0;
   out_8494019208454287783[103] = 0.0;
   out_8494019208454287783[104] = 0.0;
   out_8494019208454287783[105] = 0.0;
   out_8494019208454287783[106] = 0.0;
   out_8494019208454287783[107] = 0.0;
   out_8494019208454287783[108] = 0.0;
   out_8494019208454287783[109] = 0.0;
   out_8494019208454287783[110] = 0.0;
   out_8494019208454287783[111] = 0.0;
   out_8494019208454287783[112] = 0.0;
   out_8494019208454287783[113] = 0.0;
   out_8494019208454287783[114] = 1.0;
   out_8494019208454287783[115] = 0.0;
   out_8494019208454287783[116] = 0.0;
   out_8494019208454287783[117] = 0.0;
   out_8494019208454287783[118] = 0.0;
   out_8494019208454287783[119] = 0.0;
   out_8494019208454287783[120] = 0.0;
   out_8494019208454287783[121] = 0.0;
   out_8494019208454287783[122] = 0.0;
   out_8494019208454287783[123] = 0.0;
   out_8494019208454287783[124] = 0.0;
   out_8494019208454287783[125] = 0.0;
   out_8494019208454287783[126] = 0.0;
   out_8494019208454287783[127] = 0.0;
   out_8494019208454287783[128] = 0.0;
   out_8494019208454287783[129] = 0.0;
   out_8494019208454287783[130] = 0.0;
   out_8494019208454287783[131] = 0.0;
   out_8494019208454287783[132] = 0.0;
   out_8494019208454287783[133] = 1.0;
   out_8494019208454287783[134] = 0.0;
   out_8494019208454287783[135] = 0.0;
   out_8494019208454287783[136] = 0.0;
   out_8494019208454287783[137] = 0.0;
   out_8494019208454287783[138] = 0.0;
   out_8494019208454287783[139] = 0.0;
   out_8494019208454287783[140] = 0.0;
   out_8494019208454287783[141] = 0.0;
   out_8494019208454287783[142] = 0.0;
   out_8494019208454287783[143] = 0.0;
   out_8494019208454287783[144] = 0.0;
   out_8494019208454287783[145] = 0.0;
   out_8494019208454287783[146] = 0.0;
   out_8494019208454287783[147] = 0.0;
   out_8494019208454287783[148] = 0.0;
   out_8494019208454287783[149] = 0.0;
   out_8494019208454287783[150] = 0.0;
   out_8494019208454287783[151] = 0.0;
   out_8494019208454287783[152] = 1.0;
   out_8494019208454287783[153] = 0.0;
   out_8494019208454287783[154] = 0.0;
   out_8494019208454287783[155] = 0.0;
   out_8494019208454287783[156] = 0.0;
   out_8494019208454287783[157] = 0.0;
   out_8494019208454287783[158] = 0.0;
   out_8494019208454287783[159] = 0.0;
   out_8494019208454287783[160] = 0.0;
   out_8494019208454287783[161] = 0.0;
   out_8494019208454287783[162] = 0.0;
   out_8494019208454287783[163] = 0.0;
   out_8494019208454287783[164] = 0.0;
   out_8494019208454287783[165] = 0.0;
   out_8494019208454287783[166] = 0.0;
   out_8494019208454287783[167] = 0.0;
   out_8494019208454287783[168] = 0.0;
   out_8494019208454287783[169] = 0.0;
   out_8494019208454287783[170] = 0.0;
   out_8494019208454287783[171] = 1.0;
   out_8494019208454287783[172] = 0.0;
   out_8494019208454287783[173] = 0.0;
   out_8494019208454287783[174] = 0.0;
   out_8494019208454287783[175] = 0.0;
   out_8494019208454287783[176] = 0.0;
   out_8494019208454287783[177] = 0.0;
   out_8494019208454287783[178] = 0.0;
   out_8494019208454287783[179] = 0.0;
   out_8494019208454287783[180] = 0.0;
   out_8494019208454287783[181] = 0.0;
   out_8494019208454287783[182] = 0.0;
   out_8494019208454287783[183] = 0.0;
   out_8494019208454287783[184] = 0.0;
   out_8494019208454287783[185] = 0.0;
   out_8494019208454287783[186] = 0.0;
   out_8494019208454287783[187] = 0.0;
   out_8494019208454287783[188] = 0.0;
   out_8494019208454287783[189] = 0.0;
   out_8494019208454287783[190] = 1.0;
   out_8494019208454287783[191] = 0.0;
   out_8494019208454287783[192] = 0.0;
   out_8494019208454287783[193] = 0.0;
   out_8494019208454287783[194] = 0.0;
   out_8494019208454287783[195] = 0.0;
   out_8494019208454287783[196] = 0.0;
   out_8494019208454287783[197] = 0.0;
   out_8494019208454287783[198] = 0.0;
   out_8494019208454287783[199] = 0.0;
   out_8494019208454287783[200] = 0.0;
   out_8494019208454287783[201] = 0.0;
   out_8494019208454287783[202] = 0.0;
   out_8494019208454287783[203] = 0.0;
   out_8494019208454287783[204] = 0.0;
   out_8494019208454287783[205] = 0.0;
   out_8494019208454287783[206] = 0.0;
   out_8494019208454287783[207] = 0.0;
   out_8494019208454287783[208] = 0.0;
   out_8494019208454287783[209] = 1.0;
   out_8494019208454287783[210] = 0.0;
   out_8494019208454287783[211] = 0.0;
   out_8494019208454287783[212] = 0.0;
   out_8494019208454287783[213] = 0.0;
   out_8494019208454287783[214] = 0.0;
   out_8494019208454287783[215] = 0.0;
   out_8494019208454287783[216] = 0.0;
   out_8494019208454287783[217] = 0.0;
   out_8494019208454287783[218] = 0.0;
   out_8494019208454287783[219] = 0.0;
   out_8494019208454287783[220] = 0.0;
   out_8494019208454287783[221] = 0.0;
   out_8494019208454287783[222] = 0.0;
   out_8494019208454287783[223] = 0.0;
   out_8494019208454287783[224] = 0.0;
   out_8494019208454287783[225] = 0.0;
   out_8494019208454287783[226] = 0.0;
   out_8494019208454287783[227] = 0.0;
   out_8494019208454287783[228] = 1.0;
   out_8494019208454287783[229] = 0.0;
   out_8494019208454287783[230] = 0.0;
   out_8494019208454287783[231] = 0.0;
   out_8494019208454287783[232] = 0.0;
   out_8494019208454287783[233] = 0.0;
   out_8494019208454287783[234] = 0.0;
   out_8494019208454287783[235] = 0.0;
   out_8494019208454287783[236] = 0.0;
   out_8494019208454287783[237] = 0.0;
   out_8494019208454287783[238] = 0.0;
   out_8494019208454287783[239] = 0.0;
   out_8494019208454287783[240] = 0.0;
   out_8494019208454287783[241] = 0.0;
   out_8494019208454287783[242] = 0.0;
   out_8494019208454287783[243] = 0.0;
   out_8494019208454287783[244] = 0.0;
   out_8494019208454287783[245] = 0.0;
   out_8494019208454287783[246] = 0.0;
   out_8494019208454287783[247] = 1.0;
   out_8494019208454287783[248] = 0.0;
   out_8494019208454287783[249] = 0.0;
   out_8494019208454287783[250] = 0.0;
   out_8494019208454287783[251] = 0.0;
   out_8494019208454287783[252] = 0.0;
   out_8494019208454287783[253] = 0.0;
   out_8494019208454287783[254] = 0.0;
   out_8494019208454287783[255] = 0.0;
   out_8494019208454287783[256] = 0.0;
   out_8494019208454287783[257] = 0.0;
   out_8494019208454287783[258] = 0.0;
   out_8494019208454287783[259] = 0.0;
   out_8494019208454287783[260] = 0.0;
   out_8494019208454287783[261] = 0.0;
   out_8494019208454287783[262] = 0.0;
   out_8494019208454287783[263] = 0.0;
   out_8494019208454287783[264] = 0.0;
   out_8494019208454287783[265] = 0.0;
   out_8494019208454287783[266] = 1.0;
   out_8494019208454287783[267] = 0.0;
   out_8494019208454287783[268] = 0.0;
   out_8494019208454287783[269] = 0.0;
   out_8494019208454287783[270] = 0.0;
   out_8494019208454287783[271] = 0.0;
   out_8494019208454287783[272] = 0.0;
   out_8494019208454287783[273] = 0.0;
   out_8494019208454287783[274] = 0.0;
   out_8494019208454287783[275] = 0.0;
   out_8494019208454287783[276] = 0.0;
   out_8494019208454287783[277] = 0.0;
   out_8494019208454287783[278] = 0.0;
   out_8494019208454287783[279] = 0.0;
   out_8494019208454287783[280] = 0.0;
   out_8494019208454287783[281] = 0.0;
   out_8494019208454287783[282] = 0.0;
   out_8494019208454287783[283] = 0.0;
   out_8494019208454287783[284] = 0.0;
   out_8494019208454287783[285] = 1.0;
   out_8494019208454287783[286] = 0.0;
   out_8494019208454287783[287] = 0.0;
   out_8494019208454287783[288] = 0.0;
   out_8494019208454287783[289] = 0.0;
   out_8494019208454287783[290] = 0.0;
   out_8494019208454287783[291] = 0.0;
   out_8494019208454287783[292] = 0.0;
   out_8494019208454287783[293] = 0.0;
   out_8494019208454287783[294] = 0.0;
   out_8494019208454287783[295] = 0.0;
   out_8494019208454287783[296] = 0.0;
   out_8494019208454287783[297] = 0.0;
   out_8494019208454287783[298] = 0.0;
   out_8494019208454287783[299] = 0.0;
   out_8494019208454287783[300] = 0.0;
   out_8494019208454287783[301] = 0.0;
   out_8494019208454287783[302] = 0.0;
   out_8494019208454287783[303] = 0.0;
   out_8494019208454287783[304] = 1.0;
   out_8494019208454287783[305] = 0.0;
   out_8494019208454287783[306] = 0.0;
   out_8494019208454287783[307] = 0.0;
   out_8494019208454287783[308] = 0.0;
   out_8494019208454287783[309] = 0.0;
   out_8494019208454287783[310] = 0.0;
   out_8494019208454287783[311] = 0.0;
   out_8494019208454287783[312] = 0.0;
   out_8494019208454287783[313] = 0.0;
   out_8494019208454287783[314] = 0.0;
   out_8494019208454287783[315] = 0.0;
   out_8494019208454287783[316] = 0.0;
   out_8494019208454287783[317] = 0.0;
   out_8494019208454287783[318] = 0.0;
   out_8494019208454287783[319] = 0.0;
   out_8494019208454287783[320] = 0.0;
   out_8494019208454287783[321] = 0.0;
   out_8494019208454287783[322] = 0.0;
   out_8494019208454287783[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7093516037157220813) {
   out_7093516037157220813[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7093516037157220813[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7093516037157220813[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7093516037157220813[3] = dt*state[12] + state[3];
   out_7093516037157220813[4] = dt*state[13] + state[4];
   out_7093516037157220813[5] = dt*state[14] + state[5];
   out_7093516037157220813[6] = state[6];
   out_7093516037157220813[7] = state[7];
   out_7093516037157220813[8] = state[8];
   out_7093516037157220813[9] = state[9];
   out_7093516037157220813[10] = state[10];
   out_7093516037157220813[11] = state[11];
   out_7093516037157220813[12] = state[12];
   out_7093516037157220813[13] = state[13];
   out_7093516037157220813[14] = state[14];
   out_7093516037157220813[15] = state[15];
   out_7093516037157220813[16] = state[16];
   out_7093516037157220813[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3635578863113740513) {
   out_3635578863113740513[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3635578863113740513[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3635578863113740513[2] = 0;
   out_3635578863113740513[3] = 0;
   out_3635578863113740513[4] = 0;
   out_3635578863113740513[5] = 0;
   out_3635578863113740513[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3635578863113740513[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3635578863113740513[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3635578863113740513[9] = 0;
   out_3635578863113740513[10] = 0;
   out_3635578863113740513[11] = 0;
   out_3635578863113740513[12] = 0;
   out_3635578863113740513[13] = 0;
   out_3635578863113740513[14] = 0;
   out_3635578863113740513[15] = 0;
   out_3635578863113740513[16] = 0;
   out_3635578863113740513[17] = 0;
   out_3635578863113740513[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3635578863113740513[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3635578863113740513[20] = 0;
   out_3635578863113740513[21] = 0;
   out_3635578863113740513[22] = 0;
   out_3635578863113740513[23] = 0;
   out_3635578863113740513[24] = 0;
   out_3635578863113740513[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3635578863113740513[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3635578863113740513[27] = 0;
   out_3635578863113740513[28] = 0;
   out_3635578863113740513[29] = 0;
   out_3635578863113740513[30] = 0;
   out_3635578863113740513[31] = 0;
   out_3635578863113740513[32] = 0;
   out_3635578863113740513[33] = 0;
   out_3635578863113740513[34] = 0;
   out_3635578863113740513[35] = 0;
   out_3635578863113740513[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3635578863113740513[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3635578863113740513[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3635578863113740513[39] = 0;
   out_3635578863113740513[40] = 0;
   out_3635578863113740513[41] = 0;
   out_3635578863113740513[42] = 0;
   out_3635578863113740513[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3635578863113740513[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3635578863113740513[45] = 0;
   out_3635578863113740513[46] = 0;
   out_3635578863113740513[47] = 0;
   out_3635578863113740513[48] = 0;
   out_3635578863113740513[49] = 0;
   out_3635578863113740513[50] = 0;
   out_3635578863113740513[51] = 0;
   out_3635578863113740513[52] = 0;
   out_3635578863113740513[53] = 0;
   out_3635578863113740513[54] = 0;
   out_3635578863113740513[55] = 0;
   out_3635578863113740513[56] = 0;
   out_3635578863113740513[57] = 1;
   out_3635578863113740513[58] = 0;
   out_3635578863113740513[59] = 0;
   out_3635578863113740513[60] = 0;
   out_3635578863113740513[61] = 0;
   out_3635578863113740513[62] = 0;
   out_3635578863113740513[63] = 0;
   out_3635578863113740513[64] = 0;
   out_3635578863113740513[65] = 0;
   out_3635578863113740513[66] = dt;
   out_3635578863113740513[67] = 0;
   out_3635578863113740513[68] = 0;
   out_3635578863113740513[69] = 0;
   out_3635578863113740513[70] = 0;
   out_3635578863113740513[71] = 0;
   out_3635578863113740513[72] = 0;
   out_3635578863113740513[73] = 0;
   out_3635578863113740513[74] = 0;
   out_3635578863113740513[75] = 0;
   out_3635578863113740513[76] = 1;
   out_3635578863113740513[77] = 0;
   out_3635578863113740513[78] = 0;
   out_3635578863113740513[79] = 0;
   out_3635578863113740513[80] = 0;
   out_3635578863113740513[81] = 0;
   out_3635578863113740513[82] = 0;
   out_3635578863113740513[83] = 0;
   out_3635578863113740513[84] = 0;
   out_3635578863113740513[85] = dt;
   out_3635578863113740513[86] = 0;
   out_3635578863113740513[87] = 0;
   out_3635578863113740513[88] = 0;
   out_3635578863113740513[89] = 0;
   out_3635578863113740513[90] = 0;
   out_3635578863113740513[91] = 0;
   out_3635578863113740513[92] = 0;
   out_3635578863113740513[93] = 0;
   out_3635578863113740513[94] = 0;
   out_3635578863113740513[95] = 1;
   out_3635578863113740513[96] = 0;
   out_3635578863113740513[97] = 0;
   out_3635578863113740513[98] = 0;
   out_3635578863113740513[99] = 0;
   out_3635578863113740513[100] = 0;
   out_3635578863113740513[101] = 0;
   out_3635578863113740513[102] = 0;
   out_3635578863113740513[103] = 0;
   out_3635578863113740513[104] = dt;
   out_3635578863113740513[105] = 0;
   out_3635578863113740513[106] = 0;
   out_3635578863113740513[107] = 0;
   out_3635578863113740513[108] = 0;
   out_3635578863113740513[109] = 0;
   out_3635578863113740513[110] = 0;
   out_3635578863113740513[111] = 0;
   out_3635578863113740513[112] = 0;
   out_3635578863113740513[113] = 0;
   out_3635578863113740513[114] = 1;
   out_3635578863113740513[115] = 0;
   out_3635578863113740513[116] = 0;
   out_3635578863113740513[117] = 0;
   out_3635578863113740513[118] = 0;
   out_3635578863113740513[119] = 0;
   out_3635578863113740513[120] = 0;
   out_3635578863113740513[121] = 0;
   out_3635578863113740513[122] = 0;
   out_3635578863113740513[123] = 0;
   out_3635578863113740513[124] = 0;
   out_3635578863113740513[125] = 0;
   out_3635578863113740513[126] = 0;
   out_3635578863113740513[127] = 0;
   out_3635578863113740513[128] = 0;
   out_3635578863113740513[129] = 0;
   out_3635578863113740513[130] = 0;
   out_3635578863113740513[131] = 0;
   out_3635578863113740513[132] = 0;
   out_3635578863113740513[133] = 1;
   out_3635578863113740513[134] = 0;
   out_3635578863113740513[135] = 0;
   out_3635578863113740513[136] = 0;
   out_3635578863113740513[137] = 0;
   out_3635578863113740513[138] = 0;
   out_3635578863113740513[139] = 0;
   out_3635578863113740513[140] = 0;
   out_3635578863113740513[141] = 0;
   out_3635578863113740513[142] = 0;
   out_3635578863113740513[143] = 0;
   out_3635578863113740513[144] = 0;
   out_3635578863113740513[145] = 0;
   out_3635578863113740513[146] = 0;
   out_3635578863113740513[147] = 0;
   out_3635578863113740513[148] = 0;
   out_3635578863113740513[149] = 0;
   out_3635578863113740513[150] = 0;
   out_3635578863113740513[151] = 0;
   out_3635578863113740513[152] = 1;
   out_3635578863113740513[153] = 0;
   out_3635578863113740513[154] = 0;
   out_3635578863113740513[155] = 0;
   out_3635578863113740513[156] = 0;
   out_3635578863113740513[157] = 0;
   out_3635578863113740513[158] = 0;
   out_3635578863113740513[159] = 0;
   out_3635578863113740513[160] = 0;
   out_3635578863113740513[161] = 0;
   out_3635578863113740513[162] = 0;
   out_3635578863113740513[163] = 0;
   out_3635578863113740513[164] = 0;
   out_3635578863113740513[165] = 0;
   out_3635578863113740513[166] = 0;
   out_3635578863113740513[167] = 0;
   out_3635578863113740513[168] = 0;
   out_3635578863113740513[169] = 0;
   out_3635578863113740513[170] = 0;
   out_3635578863113740513[171] = 1;
   out_3635578863113740513[172] = 0;
   out_3635578863113740513[173] = 0;
   out_3635578863113740513[174] = 0;
   out_3635578863113740513[175] = 0;
   out_3635578863113740513[176] = 0;
   out_3635578863113740513[177] = 0;
   out_3635578863113740513[178] = 0;
   out_3635578863113740513[179] = 0;
   out_3635578863113740513[180] = 0;
   out_3635578863113740513[181] = 0;
   out_3635578863113740513[182] = 0;
   out_3635578863113740513[183] = 0;
   out_3635578863113740513[184] = 0;
   out_3635578863113740513[185] = 0;
   out_3635578863113740513[186] = 0;
   out_3635578863113740513[187] = 0;
   out_3635578863113740513[188] = 0;
   out_3635578863113740513[189] = 0;
   out_3635578863113740513[190] = 1;
   out_3635578863113740513[191] = 0;
   out_3635578863113740513[192] = 0;
   out_3635578863113740513[193] = 0;
   out_3635578863113740513[194] = 0;
   out_3635578863113740513[195] = 0;
   out_3635578863113740513[196] = 0;
   out_3635578863113740513[197] = 0;
   out_3635578863113740513[198] = 0;
   out_3635578863113740513[199] = 0;
   out_3635578863113740513[200] = 0;
   out_3635578863113740513[201] = 0;
   out_3635578863113740513[202] = 0;
   out_3635578863113740513[203] = 0;
   out_3635578863113740513[204] = 0;
   out_3635578863113740513[205] = 0;
   out_3635578863113740513[206] = 0;
   out_3635578863113740513[207] = 0;
   out_3635578863113740513[208] = 0;
   out_3635578863113740513[209] = 1;
   out_3635578863113740513[210] = 0;
   out_3635578863113740513[211] = 0;
   out_3635578863113740513[212] = 0;
   out_3635578863113740513[213] = 0;
   out_3635578863113740513[214] = 0;
   out_3635578863113740513[215] = 0;
   out_3635578863113740513[216] = 0;
   out_3635578863113740513[217] = 0;
   out_3635578863113740513[218] = 0;
   out_3635578863113740513[219] = 0;
   out_3635578863113740513[220] = 0;
   out_3635578863113740513[221] = 0;
   out_3635578863113740513[222] = 0;
   out_3635578863113740513[223] = 0;
   out_3635578863113740513[224] = 0;
   out_3635578863113740513[225] = 0;
   out_3635578863113740513[226] = 0;
   out_3635578863113740513[227] = 0;
   out_3635578863113740513[228] = 1;
   out_3635578863113740513[229] = 0;
   out_3635578863113740513[230] = 0;
   out_3635578863113740513[231] = 0;
   out_3635578863113740513[232] = 0;
   out_3635578863113740513[233] = 0;
   out_3635578863113740513[234] = 0;
   out_3635578863113740513[235] = 0;
   out_3635578863113740513[236] = 0;
   out_3635578863113740513[237] = 0;
   out_3635578863113740513[238] = 0;
   out_3635578863113740513[239] = 0;
   out_3635578863113740513[240] = 0;
   out_3635578863113740513[241] = 0;
   out_3635578863113740513[242] = 0;
   out_3635578863113740513[243] = 0;
   out_3635578863113740513[244] = 0;
   out_3635578863113740513[245] = 0;
   out_3635578863113740513[246] = 0;
   out_3635578863113740513[247] = 1;
   out_3635578863113740513[248] = 0;
   out_3635578863113740513[249] = 0;
   out_3635578863113740513[250] = 0;
   out_3635578863113740513[251] = 0;
   out_3635578863113740513[252] = 0;
   out_3635578863113740513[253] = 0;
   out_3635578863113740513[254] = 0;
   out_3635578863113740513[255] = 0;
   out_3635578863113740513[256] = 0;
   out_3635578863113740513[257] = 0;
   out_3635578863113740513[258] = 0;
   out_3635578863113740513[259] = 0;
   out_3635578863113740513[260] = 0;
   out_3635578863113740513[261] = 0;
   out_3635578863113740513[262] = 0;
   out_3635578863113740513[263] = 0;
   out_3635578863113740513[264] = 0;
   out_3635578863113740513[265] = 0;
   out_3635578863113740513[266] = 1;
   out_3635578863113740513[267] = 0;
   out_3635578863113740513[268] = 0;
   out_3635578863113740513[269] = 0;
   out_3635578863113740513[270] = 0;
   out_3635578863113740513[271] = 0;
   out_3635578863113740513[272] = 0;
   out_3635578863113740513[273] = 0;
   out_3635578863113740513[274] = 0;
   out_3635578863113740513[275] = 0;
   out_3635578863113740513[276] = 0;
   out_3635578863113740513[277] = 0;
   out_3635578863113740513[278] = 0;
   out_3635578863113740513[279] = 0;
   out_3635578863113740513[280] = 0;
   out_3635578863113740513[281] = 0;
   out_3635578863113740513[282] = 0;
   out_3635578863113740513[283] = 0;
   out_3635578863113740513[284] = 0;
   out_3635578863113740513[285] = 1;
   out_3635578863113740513[286] = 0;
   out_3635578863113740513[287] = 0;
   out_3635578863113740513[288] = 0;
   out_3635578863113740513[289] = 0;
   out_3635578863113740513[290] = 0;
   out_3635578863113740513[291] = 0;
   out_3635578863113740513[292] = 0;
   out_3635578863113740513[293] = 0;
   out_3635578863113740513[294] = 0;
   out_3635578863113740513[295] = 0;
   out_3635578863113740513[296] = 0;
   out_3635578863113740513[297] = 0;
   out_3635578863113740513[298] = 0;
   out_3635578863113740513[299] = 0;
   out_3635578863113740513[300] = 0;
   out_3635578863113740513[301] = 0;
   out_3635578863113740513[302] = 0;
   out_3635578863113740513[303] = 0;
   out_3635578863113740513[304] = 1;
   out_3635578863113740513[305] = 0;
   out_3635578863113740513[306] = 0;
   out_3635578863113740513[307] = 0;
   out_3635578863113740513[308] = 0;
   out_3635578863113740513[309] = 0;
   out_3635578863113740513[310] = 0;
   out_3635578863113740513[311] = 0;
   out_3635578863113740513[312] = 0;
   out_3635578863113740513[313] = 0;
   out_3635578863113740513[314] = 0;
   out_3635578863113740513[315] = 0;
   out_3635578863113740513[316] = 0;
   out_3635578863113740513[317] = 0;
   out_3635578863113740513[318] = 0;
   out_3635578863113740513[319] = 0;
   out_3635578863113740513[320] = 0;
   out_3635578863113740513[321] = 0;
   out_3635578863113740513[322] = 0;
   out_3635578863113740513[323] = 1;
}
void h_4(double *state, double *unused, double *out_8924931421822357051) {
   out_8924931421822357051[0] = state[6] + state[9];
   out_8924931421822357051[1] = state[7] + state[10];
   out_8924931421822357051[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_9198563863200551226) {
   out_9198563863200551226[0] = 0;
   out_9198563863200551226[1] = 0;
   out_9198563863200551226[2] = 0;
   out_9198563863200551226[3] = 0;
   out_9198563863200551226[4] = 0;
   out_9198563863200551226[5] = 0;
   out_9198563863200551226[6] = 1;
   out_9198563863200551226[7] = 0;
   out_9198563863200551226[8] = 0;
   out_9198563863200551226[9] = 1;
   out_9198563863200551226[10] = 0;
   out_9198563863200551226[11] = 0;
   out_9198563863200551226[12] = 0;
   out_9198563863200551226[13] = 0;
   out_9198563863200551226[14] = 0;
   out_9198563863200551226[15] = 0;
   out_9198563863200551226[16] = 0;
   out_9198563863200551226[17] = 0;
   out_9198563863200551226[18] = 0;
   out_9198563863200551226[19] = 0;
   out_9198563863200551226[20] = 0;
   out_9198563863200551226[21] = 0;
   out_9198563863200551226[22] = 0;
   out_9198563863200551226[23] = 0;
   out_9198563863200551226[24] = 0;
   out_9198563863200551226[25] = 1;
   out_9198563863200551226[26] = 0;
   out_9198563863200551226[27] = 0;
   out_9198563863200551226[28] = 1;
   out_9198563863200551226[29] = 0;
   out_9198563863200551226[30] = 0;
   out_9198563863200551226[31] = 0;
   out_9198563863200551226[32] = 0;
   out_9198563863200551226[33] = 0;
   out_9198563863200551226[34] = 0;
   out_9198563863200551226[35] = 0;
   out_9198563863200551226[36] = 0;
   out_9198563863200551226[37] = 0;
   out_9198563863200551226[38] = 0;
   out_9198563863200551226[39] = 0;
   out_9198563863200551226[40] = 0;
   out_9198563863200551226[41] = 0;
   out_9198563863200551226[42] = 0;
   out_9198563863200551226[43] = 0;
   out_9198563863200551226[44] = 1;
   out_9198563863200551226[45] = 0;
   out_9198563863200551226[46] = 0;
   out_9198563863200551226[47] = 1;
   out_9198563863200551226[48] = 0;
   out_9198563863200551226[49] = 0;
   out_9198563863200551226[50] = 0;
   out_9198563863200551226[51] = 0;
   out_9198563863200551226[52] = 0;
   out_9198563863200551226[53] = 0;
}
void h_10(double *state, double *unused, double *out_457631932626616900) {
   out_457631932626616900[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_457631932626616900[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_457631932626616900[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_234197665491246895) {
   out_234197665491246895[0] = 0;
   out_234197665491246895[1] = 9.8100000000000005*cos(state[1]);
   out_234197665491246895[2] = 0;
   out_234197665491246895[3] = 0;
   out_234197665491246895[4] = -state[8];
   out_234197665491246895[5] = state[7];
   out_234197665491246895[6] = 0;
   out_234197665491246895[7] = state[5];
   out_234197665491246895[8] = -state[4];
   out_234197665491246895[9] = 0;
   out_234197665491246895[10] = 0;
   out_234197665491246895[11] = 0;
   out_234197665491246895[12] = 1;
   out_234197665491246895[13] = 0;
   out_234197665491246895[14] = 0;
   out_234197665491246895[15] = 1;
   out_234197665491246895[16] = 0;
   out_234197665491246895[17] = 0;
   out_234197665491246895[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_234197665491246895[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_234197665491246895[20] = 0;
   out_234197665491246895[21] = state[8];
   out_234197665491246895[22] = 0;
   out_234197665491246895[23] = -state[6];
   out_234197665491246895[24] = -state[5];
   out_234197665491246895[25] = 0;
   out_234197665491246895[26] = state[3];
   out_234197665491246895[27] = 0;
   out_234197665491246895[28] = 0;
   out_234197665491246895[29] = 0;
   out_234197665491246895[30] = 0;
   out_234197665491246895[31] = 1;
   out_234197665491246895[32] = 0;
   out_234197665491246895[33] = 0;
   out_234197665491246895[34] = 1;
   out_234197665491246895[35] = 0;
   out_234197665491246895[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_234197665491246895[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_234197665491246895[38] = 0;
   out_234197665491246895[39] = -state[7];
   out_234197665491246895[40] = state[6];
   out_234197665491246895[41] = 0;
   out_234197665491246895[42] = state[4];
   out_234197665491246895[43] = -state[3];
   out_234197665491246895[44] = 0;
   out_234197665491246895[45] = 0;
   out_234197665491246895[46] = 0;
   out_234197665491246895[47] = 0;
   out_234197665491246895[48] = 0;
   out_234197665491246895[49] = 0;
   out_234197665491246895[50] = 1;
   out_234197665491246895[51] = 0;
   out_234197665491246895[52] = 0;
   out_234197665491246895[53] = 1;
}
void h_13(double *state, double *unused, double *out_5982858872629872588) {
   out_5982858872629872588[0] = state[3];
   out_5982858872629872588[1] = state[4];
   out_5982858872629872588[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5986290037868218425) {
   out_5986290037868218425[0] = 0;
   out_5986290037868218425[1] = 0;
   out_5986290037868218425[2] = 0;
   out_5986290037868218425[3] = 1;
   out_5986290037868218425[4] = 0;
   out_5986290037868218425[5] = 0;
   out_5986290037868218425[6] = 0;
   out_5986290037868218425[7] = 0;
   out_5986290037868218425[8] = 0;
   out_5986290037868218425[9] = 0;
   out_5986290037868218425[10] = 0;
   out_5986290037868218425[11] = 0;
   out_5986290037868218425[12] = 0;
   out_5986290037868218425[13] = 0;
   out_5986290037868218425[14] = 0;
   out_5986290037868218425[15] = 0;
   out_5986290037868218425[16] = 0;
   out_5986290037868218425[17] = 0;
   out_5986290037868218425[18] = 0;
   out_5986290037868218425[19] = 0;
   out_5986290037868218425[20] = 0;
   out_5986290037868218425[21] = 0;
   out_5986290037868218425[22] = 1;
   out_5986290037868218425[23] = 0;
   out_5986290037868218425[24] = 0;
   out_5986290037868218425[25] = 0;
   out_5986290037868218425[26] = 0;
   out_5986290037868218425[27] = 0;
   out_5986290037868218425[28] = 0;
   out_5986290037868218425[29] = 0;
   out_5986290037868218425[30] = 0;
   out_5986290037868218425[31] = 0;
   out_5986290037868218425[32] = 0;
   out_5986290037868218425[33] = 0;
   out_5986290037868218425[34] = 0;
   out_5986290037868218425[35] = 0;
   out_5986290037868218425[36] = 0;
   out_5986290037868218425[37] = 0;
   out_5986290037868218425[38] = 0;
   out_5986290037868218425[39] = 0;
   out_5986290037868218425[40] = 0;
   out_5986290037868218425[41] = 1;
   out_5986290037868218425[42] = 0;
   out_5986290037868218425[43] = 0;
   out_5986290037868218425[44] = 0;
   out_5986290037868218425[45] = 0;
   out_5986290037868218425[46] = 0;
   out_5986290037868218425[47] = 0;
   out_5986290037868218425[48] = 0;
   out_5986290037868218425[49] = 0;
   out_5986290037868218425[50] = 0;
   out_5986290037868218425[51] = 0;
   out_5986290037868218425[52] = 0;
   out_5986290037868218425[53] = 0;
}
void h_14(double *state, double *unused, double *out_8441296205878297134) {
   out_8441296205878297134[0] = state[6];
   out_8441296205878297134[1] = state[7];
   out_8441296205878297134[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5235323006861066697) {
   out_5235323006861066697[0] = 0;
   out_5235323006861066697[1] = 0;
   out_5235323006861066697[2] = 0;
   out_5235323006861066697[3] = 0;
   out_5235323006861066697[4] = 0;
   out_5235323006861066697[5] = 0;
   out_5235323006861066697[6] = 1;
   out_5235323006861066697[7] = 0;
   out_5235323006861066697[8] = 0;
   out_5235323006861066697[9] = 0;
   out_5235323006861066697[10] = 0;
   out_5235323006861066697[11] = 0;
   out_5235323006861066697[12] = 0;
   out_5235323006861066697[13] = 0;
   out_5235323006861066697[14] = 0;
   out_5235323006861066697[15] = 0;
   out_5235323006861066697[16] = 0;
   out_5235323006861066697[17] = 0;
   out_5235323006861066697[18] = 0;
   out_5235323006861066697[19] = 0;
   out_5235323006861066697[20] = 0;
   out_5235323006861066697[21] = 0;
   out_5235323006861066697[22] = 0;
   out_5235323006861066697[23] = 0;
   out_5235323006861066697[24] = 0;
   out_5235323006861066697[25] = 1;
   out_5235323006861066697[26] = 0;
   out_5235323006861066697[27] = 0;
   out_5235323006861066697[28] = 0;
   out_5235323006861066697[29] = 0;
   out_5235323006861066697[30] = 0;
   out_5235323006861066697[31] = 0;
   out_5235323006861066697[32] = 0;
   out_5235323006861066697[33] = 0;
   out_5235323006861066697[34] = 0;
   out_5235323006861066697[35] = 0;
   out_5235323006861066697[36] = 0;
   out_5235323006861066697[37] = 0;
   out_5235323006861066697[38] = 0;
   out_5235323006861066697[39] = 0;
   out_5235323006861066697[40] = 0;
   out_5235323006861066697[41] = 0;
   out_5235323006861066697[42] = 0;
   out_5235323006861066697[43] = 0;
   out_5235323006861066697[44] = 1;
   out_5235323006861066697[45] = 0;
   out_5235323006861066697[46] = 0;
   out_5235323006861066697[47] = 0;
   out_5235323006861066697[48] = 0;
   out_5235323006861066697[49] = 0;
   out_5235323006861066697[50] = 0;
   out_5235323006861066697[51] = 0;
   out_5235323006861066697[52] = 0;
   out_5235323006861066697[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4864162633936306843) {
  err_fun(nom_x, delta_x, out_4864162633936306843);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9066787371279446197) {
  inv_err_fun(nom_x, true_x, out_9066787371279446197);
}
void pose_H_mod_fun(double *state, double *out_8494019208454287783) {
  H_mod_fun(state, out_8494019208454287783);
}
void pose_f_fun(double *state, double dt, double *out_7093516037157220813) {
  f_fun(state,  dt, out_7093516037157220813);
}
void pose_F_fun(double *state, double dt, double *out_3635578863113740513) {
  F_fun(state,  dt, out_3635578863113740513);
}
void pose_h_4(double *state, double *unused, double *out_8924931421822357051) {
  h_4(state, unused, out_8924931421822357051);
}
void pose_H_4(double *state, double *unused, double *out_9198563863200551226) {
  H_4(state, unused, out_9198563863200551226);
}
void pose_h_10(double *state, double *unused, double *out_457631932626616900) {
  h_10(state, unused, out_457631932626616900);
}
void pose_H_10(double *state, double *unused, double *out_234197665491246895) {
  H_10(state, unused, out_234197665491246895);
}
void pose_h_13(double *state, double *unused, double *out_5982858872629872588) {
  h_13(state, unused, out_5982858872629872588);
}
void pose_H_13(double *state, double *unused, double *out_5986290037868218425) {
  H_13(state, unused, out_5986290037868218425);
}
void pose_h_14(double *state, double *unused, double *out_8441296205878297134) {
  h_14(state, unused, out_8441296205878297134);
}
void pose_H_14(double *state, double *unused, double *out_5235323006861066697) {
  H_14(state, unused, out_5235323006861066697);
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
