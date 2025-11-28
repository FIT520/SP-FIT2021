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
void err_fun(double *nom_x, double *delta_x, double *out_2799864307103698151) {
   out_2799864307103698151[0] = delta_x[0] + nom_x[0];
   out_2799864307103698151[1] = delta_x[1] + nom_x[1];
   out_2799864307103698151[2] = delta_x[2] + nom_x[2];
   out_2799864307103698151[3] = delta_x[3] + nom_x[3];
   out_2799864307103698151[4] = delta_x[4] + nom_x[4];
   out_2799864307103698151[5] = delta_x[5] + nom_x[5];
   out_2799864307103698151[6] = delta_x[6] + nom_x[6];
   out_2799864307103698151[7] = delta_x[7] + nom_x[7];
   out_2799864307103698151[8] = delta_x[8] + nom_x[8];
   out_2799864307103698151[9] = delta_x[9] + nom_x[9];
   out_2799864307103698151[10] = delta_x[10] + nom_x[10];
   out_2799864307103698151[11] = delta_x[11] + nom_x[11];
   out_2799864307103698151[12] = delta_x[12] + nom_x[12];
   out_2799864307103698151[13] = delta_x[13] + nom_x[13];
   out_2799864307103698151[14] = delta_x[14] + nom_x[14];
   out_2799864307103698151[15] = delta_x[15] + nom_x[15];
   out_2799864307103698151[16] = delta_x[16] + nom_x[16];
   out_2799864307103698151[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5877455747313980100) {
   out_5877455747313980100[0] = -nom_x[0] + true_x[0];
   out_5877455747313980100[1] = -nom_x[1] + true_x[1];
   out_5877455747313980100[2] = -nom_x[2] + true_x[2];
   out_5877455747313980100[3] = -nom_x[3] + true_x[3];
   out_5877455747313980100[4] = -nom_x[4] + true_x[4];
   out_5877455747313980100[5] = -nom_x[5] + true_x[5];
   out_5877455747313980100[6] = -nom_x[6] + true_x[6];
   out_5877455747313980100[7] = -nom_x[7] + true_x[7];
   out_5877455747313980100[8] = -nom_x[8] + true_x[8];
   out_5877455747313980100[9] = -nom_x[9] + true_x[9];
   out_5877455747313980100[10] = -nom_x[10] + true_x[10];
   out_5877455747313980100[11] = -nom_x[11] + true_x[11];
   out_5877455747313980100[12] = -nom_x[12] + true_x[12];
   out_5877455747313980100[13] = -nom_x[13] + true_x[13];
   out_5877455747313980100[14] = -nom_x[14] + true_x[14];
   out_5877455747313980100[15] = -nom_x[15] + true_x[15];
   out_5877455747313980100[16] = -nom_x[16] + true_x[16];
   out_5877455747313980100[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6139442128438511871) {
   out_6139442128438511871[0] = 1.0;
   out_6139442128438511871[1] = 0.0;
   out_6139442128438511871[2] = 0.0;
   out_6139442128438511871[3] = 0.0;
   out_6139442128438511871[4] = 0.0;
   out_6139442128438511871[5] = 0.0;
   out_6139442128438511871[6] = 0.0;
   out_6139442128438511871[7] = 0.0;
   out_6139442128438511871[8] = 0.0;
   out_6139442128438511871[9] = 0.0;
   out_6139442128438511871[10] = 0.0;
   out_6139442128438511871[11] = 0.0;
   out_6139442128438511871[12] = 0.0;
   out_6139442128438511871[13] = 0.0;
   out_6139442128438511871[14] = 0.0;
   out_6139442128438511871[15] = 0.0;
   out_6139442128438511871[16] = 0.0;
   out_6139442128438511871[17] = 0.0;
   out_6139442128438511871[18] = 0.0;
   out_6139442128438511871[19] = 1.0;
   out_6139442128438511871[20] = 0.0;
   out_6139442128438511871[21] = 0.0;
   out_6139442128438511871[22] = 0.0;
   out_6139442128438511871[23] = 0.0;
   out_6139442128438511871[24] = 0.0;
   out_6139442128438511871[25] = 0.0;
   out_6139442128438511871[26] = 0.0;
   out_6139442128438511871[27] = 0.0;
   out_6139442128438511871[28] = 0.0;
   out_6139442128438511871[29] = 0.0;
   out_6139442128438511871[30] = 0.0;
   out_6139442128438511871[31] = 0.0;
   out_6139442128438511871[32] = 0.0;
   out_6139442128438511871[33] = 0.0;
   out_6139442128438511871[34] = 0.0;
   out_6139442128438511871[35] = 0.0;
   out_6139442128438511871[36] = 0.0;
   out_6139442128438511871[37] = 0.0;
   out_6139442128438511871[38] = 1.0;
   out_6139442128438511871[39] = 0.0;
   out_6139442128438511871[40] = 0.0;
   out_6139442128438511871[41] = 0.0;
   out_6139442128438511871[42] = 0.0;
   out_6139442128438511871[43] = 0.0;
   out_6139442128438511871[44] = 0.0;
   out_6139442128438511871[45] = 0.0;
   out_6139442128438511871[46] = 0.0;
   out_6139442128438511871[47] = 0.0;
   out_6139442128438511871[48] = 0.0;
   out_6139442128438511871[49] = 0.0;
   out_6139442128438511871[50] = 0.0;
   out_6139442128438511871[51] = 0.0;
   out_6139442128438511871[52] = 0.0;
   out_6139442128438511871[53] = 0.0;
   out_6139442128438511871[54] = 0.0;
   out_6139442128438511871[55] = 0.0;
   out_6139442128438511871[56] = 0.0;
   out_6139442128438511871[57] = 1.0;
   out_6139442128438511871[58] = 0.0;
   out_6139442128438511871[59] = 0.0;
   out_6139442128438511871[60] = 0.0;
   out_6139442128438511871[61] = 0.0;
   out_6139442128438511871[62] = 0.0;
   out_6139442128438511871[63] = 0.0;
   out_6139442128438511871[64] = 0.0;
   out_6139442128438511871[65] = 0.0;
   out_6139442128438511871[66] = 0.0;
   out_6139442128438511871[67] = 0.0;
   out_6139442128438511871[68] = 0.0;
   out_6139442128438511871[69] = 0.0;
   out_6139442128438511871[70] = 0.0;
   out_6139442128438511871[71] = 0.0;
   out_6139442128438511871[72] = 0.0;
   out_6139442128438511871[73] = 0.0;
   out_6139442128438511871[74] = 0.0;
   out_6139442128438511871[75] = 0.0;
   out_6139442128438511871[76] = 1.0;
   out_6139442128438511871[77] = 0.0;
   out_6139442128438511871[78] = 0.0;
   out_6139442128438511871[79] = 0.0;
   out_6139442128438511871[80] = 0.0;
   out_6139442128438511871[81] = 0.0;
   out_6139442128438511871[82] = 0.0;
   out_6139442128438511871[83] = 0.0;
   out_6139442128438511871[84] = 0.0;
   out_6139442128438511871[85] = 0.0;
   out_6139442128438511871[86] = 0.0;
   out_6139442128438511871[87] = 0.0;
   out_6139442128438511871[88] = 0.0;
   out_6139442128438511871[89] = 0.0;
   out_6139442128438511871[90] = 0.0;
   out_6139442128438511871[91] = 0.0;
   out_6139442128438511871[92] = 0.0;
   out_6139442128438511871[93] = 0.0;
   out_6139442128438511871[94] = 0.0;
   out_6139442128438511871[95] = 1.0;
   out_6139442128438511871[96] = 0.0;
   out_6139442128438511871[97] = 0.0;
   out_6139442128438511871[98] = 0.0;
   out_6139442128438511871[99] = 0.0;
   out_6139442128438511871[100] = 0.0;
   out_6139442128438511871[101] = 0.0;
   out_6139442128438511871[102] = 0.0;
   out_6139442128438511871[103] = 0.0;
   out_6139442128438511871[104] = 0.0;
   out_6139442128438511871[105] = 0.0;
   out_6139442128438511871[106] = 0.0;
   out_6139442128438511871[107] = 0.0;
   out_6139442128438511871[108] = 0.0;
   out_6139442128438511871[109] = 0.0;
   out_6139442128438511871[110] = 0.0;
   out_6139442128438511871[111] = 0.0;
   out_6139442128438511871[112] = 0.0;
   out_6139442128438511871[113] = 0.0;
   out_6139442128438511871[114] = 1.0;
   out_6139442128438511871[115] = 0.0;
   out_6139442128438511871[116] = 0.0;
   out_6139442128438511871[117] = 0.0;
   out_6139442128438511871[118] = 0.0;
   out_6139442128438511871[119] = 0.0;
   out_6139442128438511871[120] = 0.0;
   out_6139442128438511871[121] = 0.0;
   out_6139442128438511871[122] = 0.0;
   out_6139442128438511871[123] = 0.0;
   out_6139442128438511871[124] = 0.0;
   out_6139442128438511871[125] = 0.0;
   out_6139442128438511871[126] = 0.0;
   out_6139442128438511871[127] = 0.0;
   out_6139442128438511871[128] = 0.0;
   out_6139442128438511871[129] = 0.0;
   out_6139442128438511871[130] = 0.0;
   out_6139442128438511871[131] = 0.0;
   out_6139442128438511871[132] = 0.0;
   out_6139442128438511871[133] = 1.0;
   out_6139442128438511871[134] = 0.0;
   out_6139442128438511871[135] = 0.0;
   out_6139442128438511871[136] = 0.0;
   out_6139442128438511871[137] = 0.0;
   out_6139442128438511871[138] = 0.0;
   out_6139442128438511871[139] = 0.0;
   out_6139442128438511871[140] = 0.0;
   out_6139442128438511871[141] = 0.0;
   out_6139442128438511871[142] = 0.0;
   out_6139442128438511871[143] = 0.0;
   out_6139442128438511871[144] = 0.0;
   out_6139442128438511871[145] = 0.0;
   out_6139442128438511871[146] = 0.0;
   out_6139442128438511871[147] = 0.0;
   out_6139442128438511871[148] = 0.0;
   out_6139442128438511871[149] = 0.0;
   out_6139442128438511871[150] = 0.0;
   out_6139442128438511871[151] = 0.0;
   out_6139442128438511871[152] = 1.0;
   out_6139442128438511871[153] = 0.0;
   out_6139442128438511871[154] = 0.0;
   out_6139442128438511871[155] = 0.0;
   out_6139442128438511871[156] = 0.0;
   out_6139442128438511871[157] = 0.0;
   out_6139442128438511871[158] = 0.0;
   out_6139442128438511871[159] = 0.0;
   out_6139442128438511871[160] = 0.0;
   out_6139442128438511871[161] = 0.0;
   out_6139442128438511871[162] = 0.0;
   out_6139442128438511871[163] = 0.0;
   out_6139442128438511871[164] = 0.0;
   out_6139442128438511871[165] = 0.0;
   out_6139442128438511871[166] = 0.0;
   out_6139442128438511871[167] = 0.0;
   out_6139442128438511871[168] = 0.0;
   out_6139442128438511871[169] = 0.0;
   out_6139442128438511871[170] = 0.0;
   out_6139442128438511871[171] = 1.0;
   out_6139442128438511871[172] = 0.0;
   out_6139442128438511871[173] = 0.0;
   out_6139442128438511871[174] = 0.0;
   out_6139442128438511871[175] = 0.0;
   out_6139442128438511871[176] = 0.0;
   out_6139442128438511871[177] = 0.0;
   out_6139442128438511871[178] = 0.0;
   out_6139442128438511871[179] = 0.0;
   out_6139442128438511871[180] = 0.0;
   out_6139442128438511871[181] = 0.0;
   out_6139442128438511871[182] = 0.0;
   out_6139442128438511871[183] = 0.0;
   out_6139442128438511871[184] = 0.0;
   out_6139442128438511871[185] = 0.0;
   out_6139442128438511871[186] = 0.0;
   out_6139442128438511871[187] = 0.0;
   out_6139442128438511871[188] = 0.0;
   out_6139442128438511871[189] = 0.0;
   out_6139442128438511871[190] = 1.0;
   out_6139442128438511871[191] = 0.0;
   out_6139442128438511871[192] = 0.0;
   out_6139442128438511871[193] = 0.0;
   out_6139442128438511871[194] = 0.0;
   out_6139442128438511871[195] = 0.0;
   out_6139442128438511871[196] = 0.0;
   out_6139442128438511871[197] = 0.0;
   out_6139442128438511871[198] = 0.0;
   out_6139442128438511871[199] = 0.0;
   out_6139442128438511871[200] = 0.0;
   out_6139442128438511871[201] = 0.0;
   out_6139442128438511871[202] = 0.0;
   out_6139442128438511871[203] = 0.0;
   out_6139442128438511871[204] = 0.0;
   out_6139442128438511871[205] = 0.0;
   out_6139442128438511871[206] = 0.0;
   out_6139442128438511871[207] = 0.0;
   out_6139442128438511871[208] = 0.0;
   out_6139442128438511871[209] = 1.0;
   out_6139442128438511871[210] = 0.0;
   out_6139442128438511871[211] = 0.0;
   out_6139442128438511871[212] = 0.0;
   out_6139442128438511871[213] = 0.0;
   out_6139442128438511871[214] = 0.0;
   out_6139442128438511871[215] = 0.0;
   out_6139442128438511871[216] = 0.0;
   out_6139442128438511871[217] = 0.0;
   out_6139442128438511871[218] = 0.0;
   out_6139442128438511871[219] = 0.0;
   out_6139442128438511871[220] = 0.0;
   out_6139442128438511871[221] = 0.0;
   out_6139442128438511871[222] = 0.0;
   out_6139442128438511871[223] = 0.0;
   out_6139442128438511871[224] = 0.0;
   out_6139442128438511871[225] = 0.0;
   out_6139442128438511871[226] = 0.0;
   out_6139442128438511871[227] = 0.0;
   out_6139442128438511871[228] = 1.0;
   out_6139442128438511871[229] = 0.0;
   out_6139442128438511871[230] = 0.0;
   out_6139442128438511871[231] = 0.0;
   out_6139442128438511871[232] = 0.0;
   out_6139442128438511871[233] = 0.0;
   out_6139442128438511871[234] = 0.0;
   out_6139442128438511871[235] = 0.0;
   out_6139442128438511871[236] = 0.0;
   out_6139442128438511871[237] = 0.0;
   out_6139442128438511871[238] = 0.0;
   out_6139442128438511871[239] = 0.0;
   out_6139442128438511871[240] = 0.0;
   out_6139442128438511871[241] = 0.0;
   out_6139442128438511871[242] = 0.0;
   out_6139442128438511871[243] = 0.0;
   out_6139442128438511871[244] = 0.0;
   out_6139442128438511871[245] = 0.0;
   out_6139442128438511871[246] = 0.0;
   out_6139442128438511871[247] = 1.0;
   out_6139442128438511871[248] = 0.0;
   out_6139442128438511871[249] = 0.0;
   out_6139442128438511871[250] = 0.0;
   out_6139442128438511871[251] = 0.0;
   out_6139442128438511871[252] = 0.0;
   out_6139442128438511871[253] = 0.0;
   out_6139442128438511871[254] = 0.0;
   out_6139442128438511871[255] = 0.0;
   out_6139442128438511871[256] = 0.0;
   out_6139442128438511871[257] = 0.0;
   out_6139442128438511871[258] = 0.0;
   out_6139442128438511871[259] = 0.0;
   out_6139442128438511871[260] = 0.0;
   out_6139442128438511871[261] = 0.0;
   out_6139442128438511871[262] = 0.0;
   out_6139442128438511871[263] = 0.0;
   out_6139442128438511871[264] = 0.0;
   out_6139442128438511871[265] = 0.0;
   out_6139442128438511871[266] = 1.0;
   out_6139442128438511871[267] = 0.0;
   out_6139442128438511871[268] = 0.0;
   out_6139442128438511871[269] = 0.0;
   out_6139442128438511871[270] = 0.0;
   out_6139442128438511871[271] = 0.0;
   out_6139442128438511871[272] = 0.0;
   out_6139442128438511871[273] = 0.0;
   out_6139442128438511871[274] = 0.0;
   out_6139442128438511871[275] = 0.0;
   out_6139442128438511871[276] = 0.0;
   out_6139442128438511871[277] = 0.0;
   out_6139442128438511871[278] = 0.0;
   out_6139442128438511871[279] = 0.0;
   out_6139442128438511871[280] = 0.0;
   out_6139442128438511871[281] = 0.0;
   out_6139442128438511871[282] = 0.0;
   out_6139442128438511871[283] = 0.0;
   out_6139442128438511871[284] = 0.0;
   out_6139442128438511871[285] = 1.0;
   out_6139442128438511871[286] = 0.0;
   out_6139442128438511871[287] = 0.0;
   out_6139442128438511871[288] = 0.0;
   out_6139442128438511871[289] = 0.0;
   out_6139442128438511871[290] = 0.0;
   out_6139442128438511871[291] = 0.0;
   out_6139442128438511871[292] = 0.0;
   out_6139442128438511871[293] = 0.0;
   out_6139442128438511871[294] = 0.0;
   out_6139442128438511871[295] = 0.0;
   out_6139442128438511871[296] = 0.0;
   out_6139442128438511871[297] = 0.0;
   out_6139442128438511871[298] = 0.0;
   out_6139442128438511871[299] = 0.0;
   out_6139442128438511871[300] = 0.0;
   out_6139442128438511871[301] = 0.0;
   out_6139442128438511871[302] = 0.0;
   out_6139442128438511871[303] = 0.0;
   out_6139442128438511871[304] = 1.0;
   out_6139442128438511871[305] = 0.0;
   out_6139442128438511871[306] = 0.0;
   out_6139442128438511871[307] = 0.0;
   out_6139442128438511871[308] = 0.0;
   out_6139442128438511871[309] = 0.0;
   out_6139442128438511871[310] = 0.0;
   out_6139442128438511871[311] = 0.0;
   out_6139442128438511871[312] = 0.0;
   out_6139442128438511871[313] = 0.0;
   out_6139442128438511871[314] = 0.0;
   out_6139442128438511871[315] = 0.0;
   out_6139442128438511871[316] = 0.0;
   out_6139442128438511871[317] = 0.0;
   out_6139442128438511871[318] = 0.0;
   out_6139442128438511871[319] = 0.0;
   out_6139442128438511871[320] = 0.0;
   out_6139442128438511871[321] = 0.0;
   out_6139442128438511871[322] = 0.0;
   out_6139442128438511871[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3038167595385286646) {
   out_3038167595385286646[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3038167595385286646[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3038167595385286646[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3038167595385286646[3] = dt*state[12] + state[3];
   out_3038167595385286646[4] = dt*state[13] + state[4];
   out_3038167595385286646[5] = dt*state[14] + state[5];
   out_3038167595385286646[6] = state[6];
   out_3038167595385286646[7] = state[7];
   out_3038167595385286646[8] = state[8];
   out_3038167595385286646[9] = state[9];
   out_3038167595385286646[10] = state[10];
   out_3038167595385286646[11] = state[11];
   out_3038167595385286646[12] = state[12];
   out_3038167595385286646[13] = state[13];
   out_3038167595385286646[14] = state[14];
   out_3038167595385286646[15] = state[15];
   out_3038167595385286646[16] = state[16];
   out_3038167595385286646[17] = state[17];
}
void F_fun(double *state, double dt, double *out_33138222388702694) {
   out_33138222388702694[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_33138222388702694[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_33138222388702694[2] = 0;
   out_33138222388702694[3] = 0;
   out_33138222388702694[4] = 0;
   out_33138222388702694[5] = 0;
   out_33138222388702694[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_33138222388702694[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_33138222388702694[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_33138222388702694[9] = 0;
   out_33138222388702694[10] = 0;
   out_33138222388702694[11] = 0;
   out_33138222388702694[12] = 0;
   out_33138222388702694[13] = 0;
   out_33138222388702694[14] = 0;
   out_33138222388702694[15] = 0;
   out_33138222388702694[16] = 0;
   out_33138222388702694[17] = 0;
   out_33138222388702694[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_33138222388702694[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_33138222388702694[20] = 0;
   out_33138222388702694[21] = 0;
   out_33138222388702694[22] = 0;
   out_33138222388702694[23] = 0;
   out_33138222388702694[24] = 0;
   out_33138222388702694[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_33138222388702694[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_33138222388702694[27] = 0;
   out_33138222388702694[28] = 0;
   out_33138222388702694[29] = 0;
   out_33138222388702694[30] = 0;
   out_33138222388702694[31] = 0;
   out_33138222388702694[32] = 0;
   out_33138222388702694[33] = 0;
   out_33138222388702694[34] = 0;
   out_33138222388702694[35] = 0;
   out_33138222388702694[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_33138222388702694[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_33138222388702694[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_33138222388702694[39] = 0;
   out_33138222388702694[40] = 0;
   out_33138222388702694[41] = 0;
   out_33138222388702694[42] = 0;
   out_33138222388702694[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_33138222388702694[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_33138222388702694[45] = 0;
   out_33138222388702694[46] = 0;
   out_33138222388702694[47] = 0;
   out_33138222388702694[48] = 0;
   out_33138222388702694[49] = 0;
   out_33138222388702694[50] = 0;
   out_33138222388702694[51] = 0;
   out_33138222388702694[52] = 0;
   out_33138222388702694[53] = 0;
   out_33138222388702694[54] = 0;
   out_33138222388702694[55] = 0;
   out_33138222388702694[56] = 0;
   out_33138222388702694[57] = 1;
   out_33138222388702694[58] = 0;
   out_33138222388702694[59] = 0;
   out_33138222388702694[60] = 0;
   out_33138222388702694[61] = 0;
   out_33138222388702694[62] = 0;
   out_33138222388702694[63] = 0;
   out_33138222388702694[64] = 0;
   out_33138222388702694[65] = 0;
   out_33138222388702694[66] = dt;
   out_33138222388702694[67] = 0;
   out_33138222388702694[68] = 0;
   out_33138222388702694[69] = 0;
   out_33138222388702694[70] = 0;
   out_33138222388702694[71] = 0;
   out_33138222388702694[72] = 0;
   out_33138222388702694[73] = 0;
   out_33138222388702694[74] = 0;
   out_33138222388702694[75] = 0;
   out_33138222388702694[76] = 1;
   out_33138222388702694[77] = 0;
   out_33138222388702694[78] = 0;
   out_33138222388702694[79] = 0;
   out_33138222388702694[80] = 0;
   out_33138222388702694[81] = 0;
   out_33138222388702694[82] = 0;
   out_33138222388702694[83] = 0;
   out_33138222388702694[84] = 0;
   out_33138222388702694[85] = dt;
   out_33138222388702694[86] = 0;
   out_33138222388702694[87] = 0;
   out_33138222388702694[88] = 0;
   out_33138222388702694[89] = 0;
   out_33138222388702694[90] = 0;
   out_33138222388702694[91] = 0;
   out_33138222388702694[92] = 0;
   out_33138222388702694[93] = 0;
   out_33138222388702694[94] = 0;
   out_33138222388702694[95] = 1;
   out_33138222388702694[96] = 0;
   out_33138222388702694[97] = 0;
   out_33138222388702694[98] = 0;
   out_33138222388702694[99] = 0;
   out_33138222388702694[100] = 0;
   out_33138222388702694[101] = 0;
   out_33138222388702694[102] = 0;
   out_33138222388702694[103] = 0;
   out_33138222388702694[104] = dt;
   out_33138222388702694[105] = 0;
   out_33138222388702694[106] = 0;
   out_33138222388702694[107] = 0;
   out_33138222388702694[108] = 0;
   out_33138222388702694[109] = 0;
   out_33138222388702694[110] = 0;
   out_33138222388702694[111] = 0;
   out_33138222388702694[112] = 0;
   out_33138222388702694[113] = 0;
   out_33138222388702694[114] = 1;
   out_33138222388702694[115] = 0;
   out_33138222388702694[116] = 0;
   out_33138222388702694[117] = 0;
   out_33138222388702694[118] = 0;
   out_33138222388702694[119] = 0;
   out_33138222388702694[120] = 0;
   out_33138222388702694[121] = 0;
   out_33138222388702694[122] = 0;
   out_33138222388702694[123] = 0;
   out_33138222388702694[124] = 0;
   out_33138222388702694[125] = 0;
   out_33138222388702694[126] = 0;
   out_33138222388702694[127] = 0;
   out_33138222388702694[128] = 0;
   out_33138222388702694[129] = 0;
   out_33138222388702694[130] = 0;
   out_33138222388702694[131] = 0;
   out_33138222388702694[132] = 0;
   out_33138222388702694[133] = 1;
   out_33138222388702694[134] = 0;
   out_33138222388702694[135] = 0;
   out_33138222388702694[136] = 0;
   out_33138222388702694[137] = 0;
   out_33138222388702694[138] = 0;
   out_33138222388702694[139] = 0;
   out_33138222388702694[140] = 0;
   out_33138222388702694[141] = 0;
   out_33138222388702694[142] = 0;
   out_33138222388702694[143] = 0;
   out_33138222388702694[144] = 0;
   out_33138222388702694[145] = 0;
   out_33138222388702694[146] = 0;
   out_33138222388702694[147] = 0;
   out_33138222388702694[148] = 0;
   out_33138222388702694[149] = 0;
   out_33138222388702694[150] = 0;
   out_33138222388702694[151] = 0;
   out_33138222388702694[152] = 1;
   out_33138222388702694[153] = 0;
   out_33138222388702694[154] = 0;
   out_33138222388702694[155] = 0;
   out_33138222388702694[156] = 0;
   out_33138222388702694[157] = 0;
   out_33138222388702694[158] = 0;
   out_33138222388702694[159] = 0;
   out_33138222388702694[160] = 0;
   out_33138222388702694[161] = 0;
   out_33138222388702694[162] = 0;
   out_33138222388702694[163] = 0;
   out_33138222388702694[164] = 0;
   out_33138222388702694[165] = 0;
   out_33138222388702694[166] = 0;
   out_33138222388702694[167] = 0;
   out_33138222388702694[168] = 0;
   out_33138222388702694[169] = 0;
   out_33138222388702694[170] = 0;
   out_33138222388702694[171] = 1;
   out_33138222388702694[172] = 0;
   out_33138222388702694[173] = 0;
   out_33138222388702694[174] = 0;
   out_33138222388702694[175] = 0;
   out_33138222388702694[176] = 0;
   out_33138222388702694[177] = 0;
   out_33138222388702694[178] = 0;
   out_33138222388702694[179] = 0;
   out_33138222388702694[180] = 0;
   out_33138222388702694[181] = 0;
   out_33138222388702694[182] = 0;
   out_33138222388702694[183] = 0;
   out_33138222388702694[184] = 0;
   out_33138222388702694[185] = 0;
   out_33138222388702694[186] = 0;
   out_33138222388702694[187] = 0;
   out_33138222388702694[188] = 0;
   out_33138222388702694[189] = 0;
   out_33138222388702694[190] = 1;
   out_33138222388702694[191] = 0;
   out_33138222388702694[192] = 0;
   out_33138222388702694[193] = 0;
   out_33138222388702694[194] = 0;
   out_33138222388702694[195] = 0;
   out_33138222388702694[196] = 0;
   out_33138222388702694[197] = 0;
   out_33138222388702694[198] = 0;
   out_33138222388702694[199] = 0;
   out_33138222388702694[200] = 0;
   out_33138222388702694[201] = 0;
   out_33138222388702694[202] = 0;
   out_33138222388702694[203] = 0;
   out_33138222388702694[204] = 0;
   out_33138222388702694[205] = 0;
   out_33138222388702694[206] = 0;
   out_33138222388702694[207] = 0;
   out_33138222388702694[208] = 0;
   out_33138222388702694[209] = 1;
   out_33138222388702694[210] = 0;
   out_33138222388702694[211] = 0;
   out_33138222388702694[212] = 0;
   out_33138222388702694[213] = 0;
   out_33138222388702694[214] = 0;
   out_33138222388702694[215] = 0;
   out_33138222388702694[216] = 0;
   out_33138222388702694[217] = 0;
   out_33138222388702694[218] = 0;
   out_33138222388702694[219] = 0;
   out_33138222388702694[220] = 0;
   out_33138222388702694[221] = 0;
   out_33138222388702694[222] = 0;
   out_33138222388702694[223] = 0;
   out_33138222388702694[224] = 0;
   out_33138222388702694[225] = 0;
   out_33138222388702694[226] = 0;
   out_33138222388702694[227] = 0;
   out_33138222388702694[228] = 1;
   out_33138222388702694[229] = 0;
   out_33138222388702694[230] = 0;
   out_33138222388702694[231] = 0;
   out_33138222388702694[232] = 0;
   out_33138222388702694[233] = 0;
   out_33138222388702694[234] = 0;
   out_33138222388702694[235] = 0;
   out_33138222388702694[236] = 0;
   out_33138222388702694[237] = 0;
   out_33138222388702694[238] = 0;
   out_33138222388702694[239] = 0;
   out_33138222388702694[240] = 0;
   out_33138222388702694[241] = 0;
   out_33138222388702694[242] = 0;
   out_33138222388702694[243] = 0;
   out_33138222388702694[244] = 0;
   out_33138222388702694[245] = 0;
   out_33138222388702694[246] = 0;
   out_33138222388702694[247] = 1;
   out_33138222388702694[248] = 0;
   out_33138222388702694[249] = 0;
   out_33138222388702694[250] = 0;
   out_33138222388702694[251] = 0;
   out_33138222388702694[252] = 0;
   out_33138222388702694[253] = 0;
   out_33138222388702694[254] = 0;
   out_33138222388702694[255] = 0;
   out_33138222388702694[256] = 0;
   out_33138222388702694[257] = 0;
   out_33138222388702694[258] = 0;
   out_33138222388702694[259] = 0;
   out_33138222388702694[260] = 0;
   out_33138222388702694[261] = 0;
   out_33138222388702694[262] = 0;
   out_33138222388702694[263] = 0;
   out_33138222388702694[264] = 0;
   out_33138222388702694[265] = 0;
   out_33138222388702694[266] = 1;
   out_33138222388702694[267] = 0;
   out_33138222388702694[268] = 0;
   out_33138222388702694[269] = 0;
   out_33138222388702694[270] = 0;
   out_33138222388702694[271] = 0;
   out_33138222388702694[272] = 0;
   out_33138222388702694[273] = 0;
   out_33138222388702694[274] = 0;
   out_33138222388702694[275] = 0;
   out_33138222388702694[276] = 0;
   out_33138222388702694[277] = 0;
   out_33138222388702694[278] = 0;
   out_33138222388702694[279] = 0;
   out_33138222388702694[280] = 0;
   out_33138222388702694[281] = 0;
   out_33138222388702694[282] = 0;
   out_33138222388702694[283] = 0;
   out_33138222388702694[284] = 0;
   out_33138222388702694[285] = 1;
   out_33138222388702694[286] = 0;
   out_33138222388702694[287] = 0;
   out_33138222388702694[288] = 0;
   out_33138222388702694[289] = 0;
   out_33138222388702694[290] = 0;
   out_33138222388702694[291] = 0;
   out_33138222388702694[292] = 0;
   out_33138222388702694[293] = 0;
   out_33138222388702694[294] = 0;
   out_33138222388702694[295] = 0;
   out_33138222388702694[296] = 0;
   out_33138222388702694[297] = 0;
   out_33138222388702694[298] = 0;
   out_33138222388702694[299] = 0;
   out_33138222388702694[300] = 0;
   out_33138222388702694[301] = 0;
   out_33138222388702694[302] = 0;
   out_33138222388702694[303] = 0;
   out_33138222388702694[304] = 1;
   out_33138222388702694[305] = 0;
   out_33138222388702694[306] = 0;
   out_33138222388702694[307] = 0;
   out_33138222388702694[308] = 0;
   out_33138222388702694[309] = 0;
   out_33138222388702694[310] = 0;
   out_33138222388702694[311] = 0;
   out_33138222388702694[312] = 0;
   out_33138222388702694[313] = 0;
   out_33138222388702694[314] = 0;
   out_33138222388702694[315] = 0;
   out_33138222388702694[316] = 0;
   out_33138222388702694[317] = 0;
   out_33138222388702694[318] = 0;
   out_33138222388702694[319] = 0;
   out_33138222388702694[320] = 0;
   out_33138222388702694[321] = 0;
   out_33138222388702694[322] = 0;
   out_33138222388702694[323] = 1;
}
void h_4(double *state, double *unused, double *out_3971679263271049865) {
   out_3971679263271049865[0] = state[6] + state[9];
   out_3971679263271049865[1] = state[7] + state[10];
   out_3971679263271049865[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2939032227337041847) {
   out_2939032227337041847[0] = 0;
   out_2939032227337041847[1] = 0;
   out_2939032227337041847[2] = 0;
   out_2939032227337041847[3] = 0;
   out_2939032227337041847[4] = 0;
   out_2939032227337041847[5] = 0;
   out_2939032227337041847[6] = 1;
   out_2939032227337041847[7] = 0;
   out_2939032227337041847[8] = 0;
   out_2939032227337041847[9] = 1;
   out_2939032227337041847[10] = 0;
   out_2939032227337041847[11] = 0;
   out_2939032227337041847[12] = 0;
   out_2939032227337041847[13] = 0;
   out_2939032227337041847[14] = 0;
   out_2939032227337041847[15] = 0;
   out_2939032227337041847[16] = 0;
   out_2939032227337041847[17] = 0;
   out_2939032227337041847[18] = 0;
   out_2939032227337041847[19] = 0;
   out_2939032227337041847[20] = 0;
   out_2939032227337041847[21] = 0;
   out_2939032227337041847[22] = 0;
   out_2939032227337041847[23] = 0;
   out_2939032227337041847[24] = 0;
   out_2939032227337041847[25] = 1;
   out_2939032227337041847[26] = 0;
   out_2939032227337041847[27] = 0;
   out_2939032227337041847[28] = 1;
   out_2939032227337041847[29] = 0;
   out_2939032227337041847[30] = 0;
   out_2939032227337041847[31] = 0;
   out_2939032227337041847[32] = 0;
   out_2939032227337041847[33] = 0;
   out_2939032227337041847[34] = 0;
   out_2939032227337041847[35] = 0;
   out_2939032227337041847[36] = 0;
   out_2939032227337041847[37] = 0;
   out_2939032227337041847[38] = 0;
   out_2939032227337041847[39] = 0;
   out_2939032227337041847[40] = 0;
   out_2939032227337041847[41] = 0;
   out_2939032227337041847[42] = 0;
   out_2939032227337041847[43] = 0;
   out_2939032227337041847[44] = 1;
   out_2939032227337041847[45] = 0;
   out_2939032227337041847[46] = 0;
   out_2939032227337041847[47] = 1;
   out_2939032227337041847[48] = 0;
   out_2939032227337041847[49] = 0;
   out_2939032227337041847[50] = 0;
   out_2939032227337041847[51] = 0;
   out_2939032227337041847[52] = 0;
   out_2939032227337041847[53] = 0;
}
void h_10(double *state, double *unused, double *out_2701029870014423504) {
   out_2701029870014423504[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2701029870014423504[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2701029870014423504[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_161399328086974098) {
   out_161399328086974098[0] = 0;
   out_161399328086974098[1] = 9.8100000000000005*cos(state[1]);
   out_161399328086974098[2] = 0;
   out_161399328086974098[3] = 0;
   out_161399328086974098[4] = -state[8];
   out_161399328086974098[5] = state[7];
   out_161399328086974098[6] = 0;
   out_161399328086974098[7] = state[5];
   out_161399328086974098[8] = -state[4];
   out_161399328086974098[9] = 0;
   out_161399328086974098[10] = 0;
   out_161399328086974098[11] = 0;
   out_161399328086974098[12] = 1;
   out_161399328086974098[13] = 0;
   out_161399328086974098[14] = 0;
   out_161399328086974098[15] = 1;
   out_161399328086974098[16] = 0;
   out_161399328086974098[17] = 0;
   out_161399328086974098[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_161399328086974098[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_161399328086974098[20] = 0;
   out_161399328086974098[21] = state[8];
   out_161399328086974098[22] = 0;
   out_161399328086974098[23] = -state[6];
   out_161399328086974098[24] = -state[5];
   out_161399328086974098[25] = 0;
   out_161399328086974098[26] = state[3];
   out_161399328086974098[27] = 0;
   out_161399328086974098[28] = 0;
   out_161399328086974098[29] = 0;
   out_161399328086974098[30] = 0;
   out_161399328086974098[31] = 1;
   out_161399328086974098[32] = 0;
   out_161399328086974098[33] = 0;
   out_161399328086974098[34] = 1;
   out_161399328086974098[35] = 0;
   out_161399328086974098[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_161399328086974098[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_161399328086974098[38] = 0;
   out_161399328086974098[39] = -state[7];
   out_161399328086974098[40] = state[6];
   out_161399328086974098[41] = 0;
   out_161399328086974098[42] = state[4];
   out_161399328086974098[43] = -state[3];
   out_161399328086974098[44] = 0;
   out_161399328086974098[45] = 0;
   out_161399328086974098[46] = 0;
   out_161399328086974098[47] = 0;
   out_161399328086974098[48] = 0;
   out_161399328086974098[49] = 0;
   out_161399328086974098[50] = 1;
   out_161399328086974098[51] = 0;
   out_161399328086974098[52] = 0;
   out_161399328086974098[53] = 1;
}
void h_13(double *state, double *unused, double *out_7398716565882038657) {
   out_7398716565882038657[0] = state[3];
   out_7398716565882038657[1] = state[4];
   out_7398716565882038657[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3503634147018885951) {
   out_3503634147018885951[0] = 0;
   out_3503634147018885951[1] = 0;
   out_3503634147018885951[2] = 0;
   out_3503634147018885951[3] = 1;
   out_3503634147018885951[4] = 0;
   out_3503634147018885951[5] = 0;
   out_3503634147018885951[6] = 0;
   out_3503634147018885951[7] = 0;
   out_3503634147018885951[8] = 0;
   out_3503634147018885951[9] = 0;
   out_3503634147018885951[10] = 0;
   out_3503634147018885951[11] = 0;
   out_3503634147018885951[12] = 0;
   out_3503634147018885951[13] = 0;
   out_3503634147018885951[14] = 0;
   out_3503634147018885951[15] = 0;
   out_3503634147018885951[16] = 0;
   out_3503634147018885951[17] = 0;
   out_3503634147018885951[18] = 0;
   out_3503634147018885951[19] = 0;
   out_3503634147018885951[20] = 0;
   out_3503634147018885951[21] = 0;
   out_3503634147018885951[22] = 1;
   out_3503634147018885951[23] = 0;
   out_3503634147018885951[24] = 0;
   out_3503634147018885951[25] = 0;
   out_3503634147018885951[26] = 0;
   out_3503634147018885951[27] = 0;
   out_3503634147018885951[28] = 0;
   out_3503634147018885951[29] = 0;
   out_3503634147018885951[30] = 0;
   out_3503634147018885951[31] = 0;
   out_3503634147018885951[32] = 0;
   out_3503634147018885951[33] = 0;
   out_3503634147018885951[34] = 0;
   out_3503634147018885951[35] = 0;
   out_3503634147018885951[36] = 0;
   out_3503634147018885951[37] = 0;
   out_3503634147018885951[38] = 0;
   out_3503634147018885951[39] = 0;
   out_3503634147018885951[40] = 0;
   out_3503634147018885951[41] = 1;
   out_3503634147018885951[42] = 0;
   out_3503634147018885951[43] = 0;
   out_3503634147018885951[44] = 0;
   out_3503634147018885951[45] = 0;
   out_3503634147018885951[46] = 0;
   out_3503634147018885951[47] = 0;
   out_3503634147018885951[48] = 0;
   out_3503634147018885951[49] = 0;
   out_3503634147018885951[50] = 0;
   out_3503634147018885951[51] = 0;
   out_3503634147018885951[52] = 0;
   out_3503634147018885951[53] = 0;
}
void h_14(double *state, double *unused, double *out_8045481680876496902) {
   out_8045481680876496902[0] = state[6];
   out_8045481680876496902[1] = state[7];
   out_8045481680876496902[2] = state[8];
}
void H_14(double *state, double *unused, double *out_143756204958330449) {
   out_143756204958330449[0] = 0;
   out_143756204958330449[1] = 0;
   out_143756204958330449[2] = 0;
   out_143756204958330449[3] = 0;
   out_143756204958330449[4] = 0;
   out_143756204958330449[5] = 0;
   out_143756204958330449[6] = 1;
   out_143756204958330449[7] = 0;
   out_143756204958330449[8] = 0;
   out_143756204958330449[9] = 0;
   out_143756204958330449[10] = 0;
   out_143756204958330449[11] = 0;
   out_143756204958330449[12] = 0;
   out_143756204958330449[13] = 0;
   out_143756204958330449[14] = 0;
   out_143756204958330449[15] = 0;
   out_143756204958330449[16] = 0;
   out_143756204958330449[17] = 0;
   out_143756204958330449[18] = 0;
   out_143756204958330449[19] = 0;
   out_143756204958330449[20] = 0;
   out_143756204958330449[21] = 0;
   out_143756204958330449[22] = 0;
   out_143756204958330449[23] = 0;
   out_143756204958330449[24] = 0;
   out_143756204958330449[25] = 1;
   out_143756204958330449[26] = 0;
   out_143756204958330449[27] = 0;
   out_143756204958330449[28] = 0;
   out_143756204958330449[29] = 0;
   out_143756204958330449[30] = 0;
   out_143756204958330449[31] = 0;
   out_143756204958330449[32] = 0;
   out_143756204958330449[33] = 0;
   out_143756204958330449[34] = 0;
   out_143756204958330449[35] = 0;
   out_143756204958330449[36] = 0;
   out_143756204958330449[37] = 0;
   out_143756204958330449[38] = 0;
   out_143756204958330449[39] = 0;
   out_143756204958330449[40] = 0;
   out_143756204958330449[41] = 0;
   out_143756204958330449[42] = 0;
   out_143756204958330449[43] = 0;
   out_143756204958330449[44] = 1;
   out_143756204958330449[45] = 0;
   out_143756204958330449[46] = 0;
   out_143756204958330449[47] = 0;
   out_143756204958330449[48] = 0;
   out_143756204958330449[49] = 0;
   out_143756204958330449[50] = 0;
   out_143756204958330449[51] = 0;
   out_143756204958330449[52] = 0;
   out_143756204958330449[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2799864307103698151) {
  err_fun(nom_x, delta_x, out_2799864307103698151);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5877455747313980100) {
  inv_err_fun(nom_x, true_x, out_5877455747313980100);
}
void pose_H_mod_fun(double *state, double *out_6139442128438511871) {
  H_mod_fun(state, out_6139442128438511871);
}
void pose_f_fun(double *state, double dt, double *out_3038167595385286646) {
  f_fun(state,  dt, out_3038167595385286646);
}
void pose_F_fun(double *state, double dt, double *out_33138222388702694) {
  F_fun(state,  dt, out_33138222388702694);
}
void pose_h_4(double *state, double *unused, double *out_3971679263271049865) {
  h_4(state, unused, out_3971679263271049865);
}
void pose_H_4(double *state, double *unused, double *out_2939032227337041847) {
  H_4(state, unused, out_2939032227337041847);
}
void pose_h_10(double *state, double *unused, double *out_2701029870014423504) {
  h_10(state, unused, out_2701029870014423504);
}
void pose_H_10(double *state, double *unused, double *out_161399328086974098) {
  H_10(state, unused, out_161399328086974098);
}
void pose_h_13(double *state, double *unused, double *out_7398716565882038657) {
  h_13(state, unused, out_7398716565882038657);
}
void pose_H_13(double *state, double *unused, double *out_3503634147018885951) {
  H_13(state, unused, out_3503634147018885951);
}
void pose_h_14(double *state, double *unused, double *out_8045481680876496902) {
  h_14(state, unused, out_8045481680876496902);
}
void pose_H_14(double *state, double *unused, double *out_143756204958330449) {
  H_14(state, unused, out_143756204958330449);
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
