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
void err_fun(double *nom_x, double *delta_x, double *out_4833332473031117184) {
   out_4833332473031117184[0] = delta_x[0] + nom_x[0];
   out_4833332473031117184[1] = delta_x[1] + nom_x[1];
   out_4833332473031117184[2] = delta_x[2] + nom_x[2];
   out_4833332473031117184[3] = delta_x[3] + nom_x[3];
   out_4833332473031117184[4] = delta_x[4] + nom_x[4];
   out_4833332473031117184[5] = delta_x[5] + nom_x[5];
   out_4833332473031117184[6] = delta_x[6] + nom_x[6];
   out_4833332473031117184[7] = delta_x[7] + nom_x[7];
   out_4833332473031117184[8] = delta_x[8] + nom_x[8];
   out_4833332473031117184[9] = delta_x[9] + nom_x[9];
   out_4833332473031117184[10] = delta_x[10] + nom_x[10];
   out_4833332473031117184[11] = delta_x[11] + nom_x[11];
   out_4833332473031117184[12] = delta_x[12] + nom_x[12];
   out_4833332473031117184[13] = delta_x[13] + nom_x[13];
   out_4833332473031117184[14] = delta_x[14] + nom_x[14];
   out_4833332473031117184[15] = delta_x[15] + nom_x[15];
   out_4833332473031117184[16] = delta_x[16] + nom_x[16];
   out_4833332473031117184[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3480076499285872836) {
   out_3480076499285872836[0] = -nom_x[0] + true_x[0];
   out_3480076499285872836[1] = -nom_x[1] + true_x[1];
   out_3480076499285872836[2] = -nom_x[2] + true_x[2];
   out_3480076499285872836[3] = -nom_x[3] + true_x[3];
   out_3480076499285872836[4] = -nom_x[4] + true_x[4];
   out_3480076499285872836[5] = -nom_x[5] + true_x[5];
   out_3480076499285872836[6] = -nom_x[6] + true_x[6];
   out_3480076499285872836[7] = -nom_x[7] + true_x[7];
   out_3480076499285872836[8] = -nom_x[8] + true_x[8];
   out_3480076499285872836[9] = -nom_x[9] + true_x[9];
   out_3480076499285872836[10] = -nom_x[10] + true_x[10];
   out_3480076499285872836[11] = -nom_x[11] + true_x[11];
   out_3480076499285872836[12] = -nom_x[12] + true_x[12];
   out_3480076499285872836[13] = -nom_x[13] + true_x[13];
   out_3480076499285872836[14] = -nom_x[14] + true_x[14];
   out_3480076499285872836[15] = -nom_x[15] + true_x[15];
   out_3480076499285872836[16] = -nom_x[16] + true_x[16];
   out_3480076499285872836[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1421822751067225790) {
   out_1421822751067225790[0] = 1.0;
   out_1421822751067225790[1] = 0.0;
   out_1421822751067225790[2] = 0.0;
   out_1421822751067225790[3] = 0.0;
   out_1421822751067225790[4] = 0.0;
   out_1421822751067225790[5] = 0.0;
   out_1421822751067225790[6] = 0.0;
   out_1421822751067225790[7] = 0.0;
   out_1421822751067225790[8] = 0.0;
   out_1421822751067225790[9] = 0.0;
   out_1421822751067225790[10] = 0.0;
   out_1421822751067225790[11] = 0.0;
   out_1421822751067225790[12] = 0.0;
   out_1421822751067225790[13] = 0.0;
   out_1421822751067225790[14] = 0.0;
   out_1421822751067225790[15] = 0.0;
   out_1421822751067225790[16] = 0.0;
   out_1421822751067225790[17] = 0.0;
   out_1421822751067225790[18] = 0.0;
   out_1421822751067225790[19] = 1.0;
   out_1421822751067225790[20] = 0.0;
   out_1421822751067225790[21] = 0.0;
   out_1421822751067225790[22] = 0.0;
   out_1421822751067225790[23] = 0.0;
   out_1421822751067225790[24] = 0.0;
   out_1421822751067225790[25] = 0.0;
   out_1421822751067225790[26] = 0.0;
   out_1421822751067225790[27] = 0.0;
   out_1421822751067225790[28] = 0.0;
   out_1421822751067225790[29] = 0.0;
   out_1421822751067225790[30] = 0.0;
   out_1421822751067225790[31] = 0.0;
   out_1421822751067225790[32] = 0.0;
   out_1421822751067225790[33] = 0.0;
   out_1421822751067225790[34] = 0.0;
   out_1421822751067225790[35] = 0.0;
   out_1421822751067225790[36] = 0.0;
   out_1421822751067225790[37] = 0.0;
   out_1421822751067225790[38] = 1.0;
   out_1421822751067225790[39] = 0.0;
   out_1421822751067225790[40] = 0.0;
   out_1421822751067225790[41] = 0.0;
   out_1421822751067225790[42] = 0.0;
   out_1421822751067225790[43] = 0.0;
   out_1421822751067225790[44] = 0.0;
   out_1421822751067225790[45] = 0.0;
   out_1421822751067225790[46] = 0.0;
   out_1421822751067225790[47] = 0.0;
   out_1421822751067225790[48] = 0.0;
   out_1421822751067225790[49] = 0.0;
   out_1421822751067225790[50] = 0.0;
   out_1421822751067225790[51] = 0.0;
   out_1421822751067225790[52] = 0.0;
   out_1421822751067225790[53] = 0.0;
   out_1421822751067225790[54] = 0.0;
   out_1421822751067225790[55] = 0.0;
   out_1421822751067225790[56] = 0.0;
   out_1421822751067225790[57] = 1.0;
   out_1421822751067225790[58] = 0.0;
   out_1421822751067225790[59] = 0.0;
   out_1421822751067225790[60] = 0.0;
   out_1421822751067225790[61] = 0.0;
   out_1421822751067225790[62] = 0.0;
   out_1421822751067225790[63] = 0.0;
   out_1421822751067225790[64] = 0.0;
   out_1421822751067225790[65] = 0.0;
   out_1421822751067225790[66] = 0.0;
   out_1421822751067225790[67] = 0.0;
   out_1421822751067225790[68] = 0.0;
   out_1421822751067225790[69] = 0.0;
   out_1421822751067225790[70] = 0.0;
   out_1421822751067225790[71] = 0.0;
   out_1421822751067225790[72] = 0.0;
   out_1421822751067225790[73] = 0.0;
   out_1421822751067225790[74] = 0.0;
   out_1421822751067225790[75] = 0.0;
   out_1421822751067225790[76] = 1.0;
   out_1421822751067225790[77] = 0.0;
   out_1421822751067225790[78] = 0.0;
   out_1421822751067225790[79] = 0.0;
   out_1421822751067225790[80] = 0.0;
   out_1421822751067225790[81] = 0.0;
   out_1421822751067225790[82] = 0.0;
   out_1421822751067225790[83] = 0.0;
   out_1421822751067225790[84] = 0.0;
   out_1421822751067225790[85] = 0.0;
   out_1421822751067225790[86] = 0.0;
   out_1421822751067225790[87] = 0.0;
   out_1421822751067225790[88] = 0.0;
   out_1421822751067225790[89] = 0.0;
   out_1421822751067225790[90] = 0.0;
   out_1421822751067225790[91] = 0.0;
   out_1421822751067225790[92] = 0.0;
   out_1421822751067225790[93] = 0.0;
   out_1421822751067225790[94] = 0.0;
   out_1421822751067225790[95] = 1.0;
   out_1421822751067225790[96] = 0.0;
   out_1421822751067225790[97] = 0.0;
   out_1421822751067225790[98] = 0.0;
   out_1421822751067225790[99] = 0.0;
   out_1421822751067225790[100] = 0.0;
   out_1421822751067225790[101] = 0.0;
   out_1421822751067225790[102] = 0.0;
   out_1421822751067225790[103] = 0.0;
   out_1421822751067225790[104] = 0.0;
   out_1421822751067225790[105] = 0.0;
   out_1421822751067225790[106] = 0.0;
   out_1421822751067225790[107] = 0.0;
   out_1421822751067225790[108] = 0.0;
   out_1421822751067225790[109] = 0.0;
   out_1421822751067225790[110] = 0.0;
   out_1421822751067225790[111] = 0.0;
   out_1421822751067225790[112] = 0.0;
   out_1421822751067225790[113] = 0.0;
   out_1421822751067225790[114] = 1.0;
   out_1421822751067225790[115] = 0.0;
   out_1421822751067225790[116] = 0.0;
   out_1421822751067225790[117] = 0.0;
   out_1421822751067225790[118] = 0.0;
   out_1421822751067225790[119] = 0.0;
   out_1421822751067225790[120] = 0.0;
   out_1421822751067225790[121] = 0.0;
   out_1421822751067225790[122] = 0.0;
   out_1421822751067225790[123] = 0.0;
   out_1421822751067225790[124] = 0.0;
   out_1421822751067225790[125] = 0.0;
   out_1421822751067225790[126] = 0.0;
   out_1421822751067225790[127] = 0.0;
   out_1421822751067225790[128] = 0.0;
   out_1421822751067225790[129] = 0.0;
   out_1421822751067225790[130] = 0.0;
   out_1421822751067225790[131] = 0.0;
   out_1421822751067225790[132] = 0.0;
   out_1421822751067225790[133] = 1.0;
   out_1421822751067225790[134] = 0.0;
   out_1421822751067225790[135] = 0.0;
   out_1421822751067225790[136] = 0.0;
   out_1421822751067225790[137] = 0.0;
   out_1421822751067225790[138] = 0.0;
   out_1421822751067225790[139] = 0.0;
   out_1421822751067225790[140] = 0.0;
   out_1421822751067225790[141] = 0.0;
   out_1421822751067225790[142] = 0.0;
   out_1421822751067225790[143] = 0.0;
   out_1421822751067225790[144] = 0.0;
   out_1421822751067225790[145] = 0.0;
   out_1421822751067225790[146] = 0.0;
   out_1421822751067225790[147] = 0.0;
   out_1421822751067225790[148] = 0.0;
   out_1421822751067225790[149] = 0.0;
   out_1421822751067225790[150] = 0.0;
   out_1421822751067225790[151] = 0.0;
   out_1421822751067225790[152] = 1.0;
   out_1421822751067225790[153] = 0.0;
   out_1421822751067225790[154] = 0.0;
   out_1421822751067225790[155] = 0.0;
   out_1421822751067225790[156] = 0.0;
   out_1421822751067225790[157] = 0.0;
   out_1421822751067225790[158] = 0.0;
   out_1421822751067225790[159] = 0.0;
   out_1421822751067225790[160] = 0.0;
   out_1421822751067225790[161] = 0.0;
   out_1421822751067225790[162] = 0.0;
   out_1421822751067225790[163] = 0.0;
   out_1421822751067225790[164] = 0.0;
   out_1421822751067225790[165] = 0.0;
   out_1421822751067225790[166] = 0.0;
   out_1421822751067225790[167] = 0.0;
   out_1421822751067225790[168] = 0.0;
   out_1421822751067225790[169] = 0.0;
   out_1421822751067225790[170] = 0.0;
   out_1421822751067225790[171] = 1.0;
   out_1421822751067225790[172] = 0.0;
   out_1421822751067225790[173] = 0.0;
   out_1421822751067225790[174] = 0.0;
   out_1421822751067225790[175] = 0.0;
   out_1421822751067225790[176] = 0.0;
   out_1421822751067225790[177] = 0.0;
   out_1421822751067225790[178] = 0.0;
   out_1421822751067225790[179] = 0.0;
   out_1421822751067225790[180] = 0.0;
   out_1421822751067225790[181] = 0.0;
   out_1421822751067225790[182] = 0.0;
   out_1421822751067225790[183] = 0.0;
   out_1421822751067225790[184] = 0.0;
   out_1421822751067225790[185] = 0.0;
   out_1421822751067225790[186] = 0.0;
   out_1421822751067225790[187] = 0.0;
   out_1421822751067225790[188] = 0.0;
   out_1421822751067225790[189] = 0.0;
   out_1421822751067225790[190] = 1.0;
   out_1421822751067225790[191] = 0.0;
   out_1421822751067225790[192] = 0.0;
   out_1421822751067225790[193] = 0.0;
   out_1421822751067225790[194] = 0.0;
   out_1421822751067225790[195] = 0.0;
   out_1421822751067225790[196] = 0.0;
   out_1421822751067225790[197] = 0.0;
   out_1421822751067225790[198] = 0.0;
   out_1421822751067225790[199] = 0.0;
   out_1421822751067225790[200] = 0.0;
   out_1421822751067225790[201] = 0.0;
   out_1421822751067225790[202] = 0.0;
   out_1421822751067225790[203] = 0.0;
   out_1421822751067225790[204] = 0.0;
   out_1421822751067225790[205] = 0.0;
   out_1421822751067225790[206] = 0.0;
   out_1421822751067225790[207] = 0.0;
   out_1421822751067225790[208] = 0.0;
   out_1421822751067225790[209] = 1.0;
   out_1421822751067225790[210] = 0.0;
   out_1421822751067225790[211] = 0.0;
   out_1421822751067225790[212] = 0.0;
   out_1421822751067225790[213] = 0.0;
   out_1421822751067225790[214] = 0.0;
   out_1421822751067225790[215] = 0.0;
   out_1421822751067225790[216] = 0.0;
   out_1421822751067225790[217] = 0.0;
   out_1421822751067225790[218] = 0.0;
   out_1421822751067225790[219] = 0.0;
   out_1421822751067225790[220] = 0.0;
   out_1421822751067225790[221] = 0.0;
   out_1421822751067225790[222] = 0.0;
   out_1421822751067225790[223] = 0.0;
   out_1421822751067225790[224] = 0.0;
   out_1421822751067225790[225] = 0.0;
   out_1421822751067225790[226] = 0.0;
   out_1421822751067225790[227] = 0.0;
   out_1421822751067225790[228] = 1.0;
   out_1421822751067225790[229] = 0.0;
   out_1421822751067225790[230] = 0.0;
   out_1421822751067225790[231] = 0.0;
   out_1421822751067225790[232] = 0.0;
   out_1421822751067225790[233] = 0.0;
   out_1421822751067225790[234] = 0.0;
   out_1421822751067225790[235] = 0.0;
   out_1421822751067225790[236] = 0.0;
   out_1421822751067225790[237] = 0.0;
   out_1421822751067225790[238] = 0.0;
   out_1421822751067225790[239] = 0.0;
   out_1421822751067225790[240] = 0.0;
   out_1421822751067225790[241] = 0.0;
   out_1421822751067225790[242] = 0.0;
   out_1421822751067225790[243] = 0.0;
   out_1421822751067225790[244] = 0.0;
   out_1421822751067225790[245] = 0.0;
   out_1421822751067225790[246] = 0.0;
   out_1421822751067225790[247] = 1.0;
   out_1421822751067225790[248] = 0.0;
   out_1421822751067225790[249] = 0.0;
   out_1421822751067225790[250] = 0.0;
   out_1421822751067225790[251] = 0.0;
   out_1421822751067225790[252] = 0.0;
   out_1421822751067225790[253] = 0.0;
   out_1421822751067225790[254] = 0.0;
   out_1421822751067225790[255] = 0.0;
   out_1421822751067225790[256] = 0.0;
   out_1421822751067225790[257] = 0.0;
   out_1421822751067225790[258] = 0.0;
   out_1421822751067225790[259] = 0.0;
   out_1421822751067225790[260] = 0.0;
   out_1421822751067225790[261] = 0.0;
   out_1421822751067225790[262] = 0.0;
   out_1421822751067225790[263] = 0.0;
   out_1421822751067225790[264] = 0.0;
   out_1421822751067225790[265] = 0.0;
   out_1421822751067225790[266] = 1.0;
   out_1421822751067225790[267] = 0.0;
   out_1421822751067225790[268] = 0.0;
   out_1421822751067225790[269] = 0.0;
   out_1421822751067225790[270] = 0.0;
   out_1421822751067225790[271] = 0.0;
   out_1421822751067225790[272] = 0.0;
   out_1421822751067225790[273] = 0.0;
   out_1421822751067225790[274] = 0.0;
   out_1421822751067225790[275] = 0.0;
   out_1421822751067225790[276] = 0.0;
   out_1421822751067225790[277] = 0.0;
   out_1421822751067225790[278] = 0.0;
   out_1421822751067225790[279] = 0.0;
   out_1421822751067225790[280] = 0.0;
   out_1421822751067225790[281] = 0.0;
   out_1421822751067225790[282] = 0.0;
   out_1421822751067225790[283] = 0.0;
   out_1421822751067225790[284] = 0.0;
   out_1421822751067225790[285] = 1.0;
   out_1421822751067225790[286] = 0.0;
   out_1421822751067225790[287] = 0.0;
   out_1421822751067225790[288] = 0.0;
   out_1421822751067225790[289] = 0.0;
   out_1421822751067225790[290] = 0.0;
   out_1421822751067225790[291] = 0.0;
   out_1421822751067225790[292] = 0.0;
   out_1421822751067225790[293] = 0.0;
   out_1421822751067225790[294] = 0.0;
   out_1421822751067225790[295] = 0.0;
   out_1421822751067225790[296] = 0.0;
   out_1421822751067225790[297] = 0.0;
   out_1421822751067225790[298] = 0.0;
   out_1421822751067225790[299] = 0.0;
   out_1421822751067225790[300] = 0.0;
   out_1421822751067225790[301] = 0.0;
   out_1421822751067225790[302] = 0.0;
   out_1421822751067225790[303] = 0.0;
   out_1421822751067225790[304] = 1.0;
   out_1421822751067225790[305] = 0.0;
   out_1421822751067225790[306] = 0.0;
   out_1421822751067225790[307] = 0.0;
   out_1421822751067225790[308] = 0.0;
   out_1421822751067225790[309] = 0.0;
   out_1421822751067225790[310] = 0.0;
   out_1421822751067225790[311] = 0.0;
   out_1421822751067225790[312] = 0.0;
   out_1421822751067225790[313] = 0.0;
   out_1421822751067225790[314] = 0.0;
   out_1421822751067225790[315] = 0.0;
   out_1421822751067225790[316] = 0.0;
   out_1421822751067225790[317] = 0.0;
   out_1421822751067225790[318] = 0.0;
   out_1421822751067225790[319] = 0.0;
   out_1421822751067225790[320] = 0.0;
   out_1421822751067225790[321] = 0.0;
   out_1421822751067225790[322] = 0.0;
   out_1421822751067225790[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3631502472880784260) {
   out_3631502472880784260[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3631502472880784260[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3631502472880784260[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3631502472880784260[3] = dt*state[12] + state[3];
   out_3631502472880784260[4] = dt*state[13] + state[4];
   out_3631502472880784260[5] = dt*state[14] + state[5];
   out_3631502472880784260[6] = state[6];
   out_3631502472880784260[7] = state[7];
   out_3631502472880784260[8] = state[8];
   out_3631502472880784260[9] = state[9];
   out_3631502472880784260[10] = state[10];
   out_3631502472880784260[11] = state[11];
   out_3631502472880784260[12] = state[12];
   out_3631502472880784260[13] = state[13];
   out_3631502472880784260[14] = state[14];
   out_3631502472880784260[15] = state[15];
   out_3631502472880784260[16] = state[16];
   out_3631502472880784260[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1790462606867554083) {
   out_1790462606867554083[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1790462606867554083[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1790462606867554083[2] = 0;
   out_1790462606867554083[3] = 0;
   out_1790462606867554083[4] = 0;
   out_1790462606867554083[5] = 0;
   out_1790462606867554083[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1790462606867554083[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1790462606867554083[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1790462606867554083[9] = 0;
   out_1790462606867554083[10] = 0;
   out_1790462606867554083[11] = 0;
   out_1790462606867554083[12] = 0;
   out_1790462606867554083[13] = 0;
   out_1790462606867554083[14] = 0;
   out_1790462606867554083[15] = 0;
   out_1790462606867554083[16] = 0;
   out_1790462606867554083[17] = 0;
   out_1790462606867554083[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1790462606867554083[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1790462606867554083[20] = 0;
   out_1790462606867554083[21] = 0;
   out_1790462606867554083[22] = 0;
   out_1790462606867554083[23] = 0;
   out_1790462606867554083[24] = 0;
   out_1790462606867554083[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1790462606867554083[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1790462606867554083[27] = 0;
   out_1790462606867554083[28] = 0;
   out_1790462606867554083[29] = 0;
   out_1790462606867554083[30] = 0;
   out_1790462606867554083[31] = 0;
   out_1790462606867554083[32] = 0;
   out_1790462606867554083[33] = 0;
   out_1790462606867554083[34] = 0;
   out_1790462606867554083[35] = 0;
   out_1790462606867554083[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1790462606867554083[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1790462606867554083[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1790462606867554083[39] = 0;
   out_1790462606867554083[40] = 0;
   out_1790462606867554083[41] = 0;
   out_1790462606867554083[42] = 0;
   out_1790462606867554083[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1790462606867554083[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1790462606867554083[45] = 0;
   out_1790462606867554083[46] = 0;
   out_1790462606867554083[47] = 0;
   out_1790462606867554083[48] = 0;
   out_1790462606867554083[49] = 0;
   out_1790462606867554083[50] = 0;
   out_1790462606867554083[51] = 0;
   out_1790462606867554083[52] = 0;
   out_1790462606867554083[53] = 0;
   out_1790462606867554083[54] = 0;
   out_1790462606867554083[55] = 0;
   out_1790462606867554083[56] = 0;
   out_1790462606867554083[57] = 1;
   out_1790462606867554083[58] = 0;
   out_1790462606867554083[59] = 0;
   out_1790462606867554083[60] = 0;
   out_1790462606867554083[61] = 0;
   out_1790462606867554083[62] = 0;
   out_1790462606867554083[63] = 0;
   out_1790462606867554083[64] = 0;
   out_1790462606867554083[65] = 0;
   out_1790462606867554083[66] = dt;
   out_1790462606867554083[67] = 0;
   out_1790462606867554083[68] = 0;
   out_1790462606867554083[69] = 0;
   out_1790462606867554083[70] = 0;
   out_1790462606867554083[71] = 0;
   out_1790462606867554083[72] = 0;
   out_1790462606867554083[73] = 0;
   out_1790462606867554083[74] = 0;
   out_1790462606867554083[75] = 0;
   out_1790462606867554083[76] = 1;
   out_1790462606867554083[77] = 0;
   out_1790462606867554083[78] = 0;
   out_1790462606867554083[79] = 0;
   out_1790462606867554083[80] = 0;
   out_1790462606867554083[81] = 0;
   out_1790462606867554083[82] = 0;
   out_1790462606867554083[83] = 0;
   out_1790462606867554083[84] = 0;
   out_1790462606867554083[85] = dt;
   out_1790462606867554083[86] = 0;
   out_1790462606867554083[87] = 0;
   out_1790462606867554083[88] = 0;
   out_1790462606867554083[89] = 0;
   out_1790462606867554083[90] = 0;
   out_1790462606867554083[91] = 0;
   out_1790462606867554083[92] = 0;
   out_1790462606867554083[93] = 0;
   out_1790462606867554083[94] = 0;
   out_1790462606867554083[95] = 1;
   out_1790462606867554083[96] = 0;
   out_1790462606867554083[97] = 0;
   out_1790462606867554083[98] = 0;
   out_1790462606867554083[99] = 0;
   out_1790462606867554083[100] = 0;
   out_1790462606867554083[101] = 0;
   out_1790462606867554083[102] = 0;
   out_1790462606867554083[103] = 0;
   out_1790462606867554083[104] = dt;
   out_1790462606867554083[105] = 0;
   out_1790462606867554083[106] = 0;
   out_1790462606867554083[107] = 0;
   out_1790462606867554083[108] = 0;
   out_1790462606867554083[109] = 0;
   out_1790462606867554083[110] = 0;
   out_1790462606867554083[111] = 0;
   out_1790462606867554083[112] = 0;
   out_1790462606867554083[113] = 0;
   out_1790462606867554083[114] = 1;
   out_1790462606867554083[115] = 0;
   out_1790462606867554083[116] = 0;
   out_1790462606867554083[117] = 0;
   out_1790462606867554083[118] = 0;
   out_1790462606867554083[119] = 0;
   out_1790462606867554083[120] = 0;
   out_1790462606867554083[121] = 0;
   out_1790462606867554083[122] = 0;
   out_1790462606867554083[123] = 0;
   out_1790462606867554083[124] = 0;
   out_1790462606867554083[125] = 0;
   out_1790462606867554083[126] = 0;
   out_1790462606867554083[127] = 0;
   out_1790462606867554083[128] = 0;
   out_1790462606867554083[129] = 0;
   out_1790462606867554083[130] = 0;
   out_1790462606867554083[131] = 0;
   out_1790462606867554083[132] = 0;
   out_1790462606867554083[133] = 1;
   out_1790462606867554083[134] = 0;
   out_1790462606867554083[135] = 0;
   out_1790462606867554083[136] = 0;
   out_1790462606867554083[137] = 0;
   out_1790462606867554083[138] = 0;
   out_1790462606867554083[139] = 0;
   out_1790462606867554083[140] = 0;
   out_1790462606867554083[141] = 0;
   out_1790462606867554083[142] = 0;
   out_1790462606867554083[143] = 0;
   out_1790462606867554083[144] = 0;
   out_1790462606867554083[145] = 0;
   out_1790462606867554083[146] = 0;
   out_1790462606867554083[147] = 0;
   out_1790462606867554083[148] = 0;
   out_1790462606867554083[149] = 0;
   out_1790462606867554083[150] = 0;
   out_1790462606867554083[151] = 0;
   out_1790462606867554083[152] = 1;
   out_1790462606867554083[153] = 0;
   out_1790462606867554083[154] = 0;
   out_1790462606867554083[155] = 0;
   out_1790462606867554083[156] = 0;
   out_1790462606867554083[157] = 0;
   out_1790462606867554083[158] = 0;
   out_1790462606867554083[159] = 0;
   out_1790462606867554083[160] = 0;
   out_1790462606867554083[161] = 0;
   out_1790462606867554083[162] = 0;
   out_1790462606867554083[163] = 0;
   out_1790462606867554083[164] = 0;
   out_1790462606867554083[165] = 0;
   out_1790462606867554083[166] = 0;
   out_1790462606867554083[167] = 0;
   out_1790462606867554083[168] = 0;
   out_1790462606867554083[169] = 0;
   out_1790462606867554083[170] = 0;
   out_1790462606867554083[171] = 1;
   out_1790462606867554083[172] = 0;
   out_1790462606867554083[173] = 0;
   out_1790462606867554083[174] = 0;
   out_1790462606867554083[175] = 0;
   out_1790462606867554083[176] = 0;
   out_1790462606867554083[177] = 0;
   out_1790462606867554083[178] = 0;
   out_1790462606867554083[179] = 0;
   out_1790462606867554083[180] = 0;
   out_1790462606867554083[181] = 0;
   out_1790462606867554083[182] = 0;
   out_1790462606867554083[183] = 0;
   out_1790462606867554083[184] = 0;
   out_1790462606867554083[185] = 0;
   out_1790462606867554083[186] = 0;
   out_1790462606867554083[187] = 0;
   out_1790462606867554083[188] = 0;
   out_1790462606867554083[189] = 0;
   out_1790462606867554083[190] = 1;
   out_1790462606867554083[191] = 0;
   out_1790462606867554083[192] = 0;
   out_1790462606867554083[193] = 0;
   out_1790462606867554083[194] = 0;
   out_1790462606867554083[195] = 0;
   out_1790462606867554083[196] = 0;
   out_1790462606867554083[197] = 0;
   out_1790462606867554083[198] = 0;
   out_1790462606867554083[199] = 0;
   out_1790462606867554083[200] = 0;
   out_1790462606867554083[201] = 0;
   out_1790462606867554083[202] = 0;
   out_1790462606867554083[203] = 0;
   out_1790462606867554083[204] = 0;
   out_1790462606867554083[205] = 0;
   out_1790462606867554083[206] = 0;
   out_1790462606867554083[207] = 0;
   out_1790462606867554083[208] = 0;
   out_1790462606867554083[209] = 1;
   out_1790462606867554083[210] = 0;
   out_1790462606867554083[211] = 0;
   out_1790462606867554083[212] = 0;
   out_1790462606867554083[213] = 0;
   out_1790462606867554083[214] = 0;
   out_1790462606867554083[215] = 0;
   out_1790462606867554083[216] = 0;
   out_1790462606867554083[217] = 0;
   out_1790462606867554083[218] = 0;
   out_1790462606867554083[219] = 0;
   out_1790462606867554083[220] = 0;
   out_1790462606867554083[221] = 0;
   out_1790462606867554083[222] = 0;
   out_1790462606867554083[223] = 0;
   out_1790462606867554083[224] = 0;
   out_1790462606867554083[225] = 0;
   out_1790462606867554083[226] = 0;
   out_1790462606867554083[227] = 0;
   out_1790462606867554083[228] = 1;
   out_1790462606867554083[229] = 0;
   out_1790462606867554083[230] = 0;
   out_1790462606867554083[231] = 0;
   out_1790462606867554083[232] = 0;
   out_1790462606867554083[233] = 0;
   out_1790462606867554083[234] = 0;
   out_1790462606867554083[235] = 0;
   out_1790462606867554083[236] = 0;
   out_1790462606867554083[237] = 0;
   out_1790462606867554083[238] = 0;
   out_1790462606867554083[239] = 0;
   out_1790462606867554083[240] = 0;
   out_1790462606867554083[241] = 0;
   out_1790462606867554083[242] = 0;
   out_1790462606867554083[243] = 0;
   out_1790462606867554083[244] = 0;
   out_1790462606867554083[245] = 0;
   out_1790462606867554083[246] = 0;
   out_1790462606867554083[247] = 1;
   out_1790462606867554083[248] = 0;
   out_1790462606867554083[249] = 0;
   out_1790462606867554083[250] = 0;
   out_1790462606867554083[251] = 0;
   out_1790462606867554083[252] = 0;
   out_1790462606867554083[253] = 0;
   out_1790462606867554083[254] = 0;
   out_1790462606867554083[255] = 0;
   out_1790462606867554083[256] = 0;
   out_1790462606867554083[257] = 0;
   out_1790462606867554083[258] = 0;
   out_1790462606867554083[259] = 0;
   out_1790462606867554083[260] = 0;
   out_1790462606867554083[261] = 0;
   out_1790462606867554083[262] = 0;
   out_1790462606867554083[263] = 0;
   out_1790462606867554083[264] = 0;
   out_1790462606867554083[265] = 0;
   out_1790462606867554083[266] = 1;
   out_1790462606867554083[267] = 0;
   out_1790462606867554083[268] = 0;
   out_1790462606867554083[269] = 0;
   out_1790462606867554083[270] = 0;
   out_1790462606867554083[271] = 0;
   out_1790462606867554083[272] = 0;
   out_1790462606867554083[273] = 0;
   out_1790462606867554083[274] = 0;
   out_1790462606867554083[275] = 0;
   out_1790462606867554083[276] = 0;
   out_1790462606867554083[277] = 0;
   out_1790462606867554083[278] = 0;
   out_1790462606867554083[279] = 0;
   out_1790462606867554083[280] = 0;
   out_1790462606867554083[281] = 0;
   out_1790462606867554083[282] = 0;
   out_1790462606867554083[283] = 0;
   out_1790462606867554083[284] = 0;
   out_1790462606867554083[285] = 1;
   out_1790462606867554083[286] = 0;
   out_1790462606867554083[287] = 0;
   out_1790462606867554083[288] = 0;
   out_1790462606867554083[289] = 0;
   out_1790462606867554083[290] = 0;
   out_1790462606867554083[291] = 0;
   out_1790462606867554083[292] = 0;
   out_1790462606867554083[293] = 0;
   out_1790462606867554083[294] = 0;
   out_1790462606867554083[295] = 0;
   out_1790462606867554083[296] = 0;
   out_1790462606867554083[297] = 0;
   out_1790462606867554083[298] = 0;
   out_1790462606867554083[299] = 0;
   out_1790462606867554083[300] = 0;
   out_1790462606867554083[301] = 0;
   out_1790462606867554083[302] = 0;
   out_1790462606867554083[303] = 0;
   out_1790462606867554083[304] = 1;
   out_1790462606867554083[305] = 0;
   out_1790462606867554083[306] = 0;
   out_1790462606867554083[307] = 0;
   out_1790462606867554083[308] = 0;
   out_1790462606867554083[309] = 0;
   out_1790462606867554083[310] = 0;
   out_1790462606867554083[311] = 0;
   out_1790462606867554083[312] = 0;
   out_1790462606867554083[313] = 0;
   out_1790462606867554083[314] = 0;
   out_1790462606867554083[315] = 0;
   out_1790462606867554083[316] = 0;
   out_1790462606867554083[317] = 0;
   out_1790462606867554083[318] = 0;
   out_1790462606867554083[319] = 0;
   out_1790462606867554083[320] = 0;
   out_1790462606867554083[321] = 0;
   out_1790462606867554083[322] = 0;
   out_1790462606867554083[323] = 1;
}
void h_4(double *state, double *unused, double *out_8428208003072760504) {
   out_8428208003072760504[0] = state[6] + state[9];
   out_8428208003072760504[1] = state[7] + state[10];
   out_8428208003072760504[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7522538914990595552) {
   out_7522538914990595552[0] = 0;
   out_7522538914990595552[1] = 0;
   out_7522538914990595552[2] = 0;
   out_7522538914990595552[3] = 0;
   out_7522538914990595552[4] = 0;
   out_7522538914990595552[5] = 0;
   out_7522538914990595552[6] = 1;
   out_7522538914990595552[7] = 0;
   out_7522538914990595552[8] = 0;
   out_7522538914990595552[9] = 1;
   out_7522538914990595552[10] = 0;
   out_7522538914990595552[11] = 0;
   out_7522538914990595552[12] = 0;
   out_7522538914990595552[13] = 0;
   out_7522538914990595552[14] = 0;
   out_7522538914990595552[15] = 0;
   out_7522538914990595552[16] = 0;
   out_7522538914990595552[17] = 0;
   out_7522538914990595552[18] = 0;
   out_7522538914990595552[19] = 0;
   out_7522538914990595552[20] = 0;
   out_7522538914990595552[21] = 0;
   out_7522538914990595552[22] = 0;
   out_7522538914990595552[23] = 0;
   out_7522538914990595552[24] = 0;
   out_7522538914990595552[25] = 1;
   out_7522538914990595552[26] = 0;
   out_7522538914990595552[27] = 0;
   out_7522538914990595552[28] = 1;
   out_7522538914990595552[29] = 0;
   out_7522538914990595552[30] = 0;
   out_7522538914990595552[31] = 0;
   out_7522538914990595552[32] = 0;
   out_7522538914990595552[33] = 0;
   out_7522538914990595552[34] = 0;
   out_7522538914990595552[35] = 0;
   out_7522538914990595552[36] = 0;
   out_7522538914990595552[37] = 0;
   out_7522538914990595552[38] = 0;
   out_7522538914990595552[39] = 0;
   out_7522538914990595552[40] = 0;
   out_7522538914990595552[41] = 0;
   out_7522538914990595552[42] = 0;
   out_7522538914990595552[43] = 0;
   out_7522538914990595552[44] = 1;
   out_7522538914990595552[45] = 0;
   out_7522538914990595552[46] = 0;
   out_7522538914990595552[47] = 1;
   out_7522538914990595552[48] = 0;
   out_7522538914990595552[49] = 0;
   out_7522538914990595552[50] = 0;
   out_7522538914990595552[51] = 0;
   out_7522538914990595552[52] = 0;
   out_7522538914990595552[53] = 0;
}
void h_10(double *state, double *unused, double *out_5381702994242187932) {
   out_5381702994242187932[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5381702994242187932[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5381702994242187932[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4110512254011380820) {
   out_4110512254011380820[0] = 0;
   out_4110512254011380820[1] = 9.8100000000000005*cos(state[1]);
   out_4110512254011380820[2] = 0;
   out_4110512254011380820[3] = 0;
   out_4110512254011380820[4] = -state[8];
   out_4110512254011380820[5] = state[7];
   out_4110512254011380820[6] = 0;
   out_4110512254011380820[7] = state[5];
   out_4110512254011380820[8] = -state[4];
   out_4110512254011380820[9] = 0;
   out_4110512254011380820[10] = 0;
   out_4110512254011380820[11] = 0;
   out_4110512254011380820[12] = 1;
   out_4110512254011380820[13] = 0;
   out_4110512254011380820[14] = 0;
   out_4110512254011380820[15] = 1;
   out_4110512254011380820[16] = 0;
   out_4110512254011380820[17] = 0;
   out_4110512254011380820[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4110512254011380820[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4110512254011380820[20] = 0;
   out_4110512254011380820[21] = state[8];
   out_4110512254011380820[22] = 0;
   out_4110512254011380820[23] = -state[6];
   out_4110512254011380820[24] = -state[5];
   out_4110512254011380820[25] = 0;
   out_4110512254011380820[26] = state[3];
   out_4110512254011380820[27] = 0;
   out_4110512254011380820[28] = 0;
   out_4110512254011380820[29] = 0;
   out_4110512254011380820[30] = 0;
   out_4110512254011380820[31] = 1;
   out_4110512254011380820[32] = 0;
   out_4110512254011380820[33] = 0;
   out_4110512254011380820[34] = 1;
   out_4110512254011380820[35] = 0;
   out_4110512254011380820[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4110512254011380820[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4110512254011380820[38] = 0;
   out_4110512254011380820[39] = -state[7];
   out_4110512254011380820[40] = state[6];
   out_4110512254011380820[41] = 0;
   out_4110512254011380820[42] = state[4];
   out_4110512254011380820[43] = -state[3];
   out_4110512254011380820[44] = 0;
   out_4110512254011380820[45] = 0;
   out_4110512254011380820[46] = 0;
   out_4110512254011380820[47] = 0;
   out_4110512254011380820[48] = 0;
   out_4110512254011380820[49] = 0;
   out_4110512254011380820[50] = 1;
   out_4110512254011380820[51] = 0;
   out_4110512254011380820[52] = 0;
   out_4110512254011380820[53] = 1;
}
void h_13(double *state, double *unused, double *out_1537117227709853335) {
   out_1537117227709853335[0] = state[3];
   out_1537117227709853335[1] = state[4];
   out_1537117227709853335[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4310265089658262751) {
   out_4310265089658262751[0] = 0;
   out_4310265089658262751[1] = 0;
   out_4310265089658262751[2] = 0;
   out_4310265089658262751[3] = 1;
   out_4310265089658262751[4] = 0;
   out_4310265089658262751[5] = 0;
   out_4310265089658262751[6] = 0;
   out_4310265089658262751[7] = 0;
   out_4310265089658262751[8] = 0;
   out_4310265089658262751[9] = 0;
   out_4310265089658262751[10] = 0;
   out_4310265089658262751[11] = 0;
   out_4310265089658262751[12] = 0;
   out_4310265089658262751[13] = 0;
   out_4310265089658262751[14] = 0;
   out_4310265089658262751[15] = 0;
   out_4310265089658262751[16] = 0;
   out_4310265089658262751[17] = 0;
   out_4310265089658262751[18] = 0;
   out_4310265089658262751[19] = 0;
   out_4310265089658262751[20] = 0;
   out_4310265089658262751[21] = 0;
   out_4310265089658262751[22] = 1;
   out_4310265089658262751[23] = 0;
   out_4310265089658262751[24] = 0;
   out_4310265089658262751[25] = 0;
   out_4310265089658262751[26] = 0;
   out_4310265089658262751[27] = 0;
   out_4310265089658262751[28] = 0;
   out_4310265089658262751[29] = 0;
   out_4310265089658262751[30] = 0;
   out_4310265089658262751[31] = 0;
   out_4310265089658262751[32] = 0;
   out_4310265089658262751[33] = 0;
   out_4310265089658262751[34] = 0;
   out_4310265089658262751[35] = 0;
   out_4310265089658262751[36] = 0;
   out_4310265089658262751[37] = 0;
   out_4310265089658262751[38] = 0;
   out_4310265089658262751[39] = 0;
   out_4310265089658262751[40] = 0;
   out_4310265089658262751[41] = 1;
   out_4310265089658262751[42] = 0;
   out_4310265089658262751[43] = 0;
   out_4310265089658262751[44] = 0;
   out_4310265089658262751[45] = 0;
   out_4310265089658262751[46] = 0;
   out_4310265089658262751[47] = 0;
   out_4310265089658262751[48] = 0;
   out_4310265089658262751[49] = 0;
   out_4310265089658262751[50] = 0;
   out_4310265089658262751[51] = 0;
   out_4310265089658262751[52] = 0;
   out_4310265089658262751[53] = 0;
}
void h_14(double *state, double *unused, double *out_4391412999592827342) {
   out_4391412999592827342[0] = state[6];
   out_4391412999592827342[1] = state[7];
   out_4391412999592827342[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7841416726423583768) {
   out_7841416726423583768[0] = 0;
   out_7841416726423583768[1] = 0;
   out_7841416726423583768[2] = 0;
   out_7841416726423583768[3] = 0;
   out_7841416726423583768[4] = 0;
   out_7841416726423583768[5] = 0;
   out_7841416726423583768[6] = 1;
   out_7841416726423583768[7] = 0;
   out_7841416726423583768[8] = 0;
   out_7841416726423583768[9] = 0;
   out_7841416726423583768[10] = 0;
   out_7841416726423583768[11] = 0;
   out_7841416726423583768[12] = 0;
   out_7841416726423583768[13] = 0;
   out_7841416726423583768[14] = 0;
   out_7841416726423583768[15] = 0;
   out_7841416726423583768[16] = 0;
   out_7841416726423583768[17] = 0;
   out_7841416726423583768[18] = 0;
   out_7841416726423583768[19] = 0;
   out_7841416726423583768[20] = 0;
   out_7841416726423583768[21] = 0;
   out_7841416726423583768[22] = 0;
   out_7841416726423583768[23] = 0;
   out_7841416726423583768[24] = 0;
   out_7841416726423583768[25] = 1;
   out_7841416726423583768[26] = 0;
   out_7841416726423583768[27] = 0;
   out_7841416726423583768[28] = 0;
   out_7841416726423583768[29] = 0;
   out_7841416726423583768[30] = 0;
   out_7841416726423583768[31] = 0;
   out_7841416726423583768[32] = 0;
   out_7841416726423583768[33] = 0;
   out_7841416726423583768[34] = 0;
   out_7841416726423583768[35] = 0;
   out_7841416726423583768[36] = 0;
   out_7841416726423583768[37] = 0;
   out_7841416726423583768[38] = 0;
   out_7841416726423583768[39] = 0;
   out_7841416726423583768[40] = 0;
   out_7841416726423583768[41] = 0;
   out_7841416726423583768[42] = 0;
   out_7841416726423583768[43] = 0;
   out_7841416726423583768[44] = 1;
   out_7841416726423583768[45] = 0;
   out_7841416726423583768[46] = 0;
   out_7841416726423583768[47] = 0;
   out_7841416726423583768[48] = 0;
   out_7841416726423583768[49] = 0;
   out_7841416726423583768[50] = 0;
   out_7841416726423583768[51] = 0;
   out_7841416726423583768[52] = 0;
   out_7841416726423583768[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4833332473031117184) {
  err_fun(nom_x, delta_x, out_4833332473031117184);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3480076499285872836) {
  inv_err_fun(nom_x, true_x, out_3480076499285872836);
}
void pose_H_mod_fun(double *state, double *out_1421822751067225790) {
  H_mod_fun(state, out_1421822751067225790);
}
void pose_f_fun(double *state, double dt, double *out_3631502472880784260) {
  f_fun(state,  dt, out_3631502472880784260);
}
void pose_F_fun(double *state, double dt, double *out_1790462606867554083) {
  F_fun(state,  dt, out_1790462606867554083);
}
void pose_h_4(double *state, double *unused, double *out_8428208003072760504) {
  h_4(state, unused, out_8428208003072760504);
}
void pose_H_4(double *state, double *unused, double *out_7522538914990595552) {
  H_4(state, unused, out_7522538914990595552);
}
void pose_h_10(double *state, double *unused, double *out_5381702994242187932) {
  h_10(state, unused, out_5381702994242187932);
}
void pose_H_10(double *state, double *unused, double *out_4110512254011380820) {
  H_10(state, unused, out_4110512254011380820);
}
void pose_h_13(double *state, double *unused, double *out_1537117227709853335) {
  h_13(state, unused, out_1537117227709853335);
}
void pose_H_13(double *state, double *unused, double *out_4310265089658262751) {
  H_13(state, unused, out_4310265089658262751);
}
void pose_h_14(double *state, double *unused, double *out_4391412999592827342) {
  h_14(state, unused, out_4391412999592827342);
}
void pose_H_14(double *state, double *unused, double *out_7841416726423583768) {
  H_14(state, unused, out_7841416726423583768);
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
