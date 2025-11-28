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
void err_fun(double *nom_x, double *delta_x, double *out_7199008258596550885) {
   out_7199008258596550885[0] = delta_x[0] + nom_x[0];
   out_7199008258596550885[1] = delta_x[1] + nom_x[1];
   out_7199008258596550885[2] = delta_x[2] + nom_x[2];
   out_7199008258596550885[3] = delta_x[3] + nom_x[3];
   out_7199008258596550885[4] = delta_x[4] + nom_x[4];
   out_7199008258596550885[5] = delta_x[5] + nom_x[5];
   out_7199008258596550885[6] = delta_x[6] + nom_x[6];
   out_7199008258596550885[7] = delta_x[7] + nom_x[7];
   out_7199008258596550885[8] = delta_x[8] + nom_x[8];
   out_7199008258596550885[9] = delta_x[9] + nom_x[9];
   out_7199008258596550885[10] = delta_x[10] + nom_x[10];
   out_7199008258596550885[11] = delta_x[11] + nom_x[11];
   out_7199008258596550885[12] = delta_x[12] + nom_x[12];
   out_7199008258596550885[13] = delta_x[13] + nom_x[13];
   out_7199008258596550885[14] = delta_x[14] + nom_x[14];
   out_7199008258596550885[15] = delta_x[15] + nom_x[15];
   out_7199008258596550885[16] = delta_x[16] + nom_x[16];
   out_7199008258596550885[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1195653358946799825) {
   out_1195653358946799825[0] = -nom_x[0] + true_x[0];
   out_1195653358946799825[1] = -nom_x[1] + true_x[1];
   out_1195653358946799825[2] = -nom_x[2] + true_x[2];
   out_1195653358946799825[3] = -nom_x[3] + true_x[3];
   out_1195653358946799825[4] = -nom_x[4] + true_x[4];
   out_1195653358946799825[5] = -nom_x[5] + true_x[5];
   out_1195653358946799825[6] = -nom_x[6] + true_x[6];
   out_1195653358946799825[7] = -nom_x[7] + true_x[7];
   out_1195653358946799825[8] = -nom_x[8] + true_x[8];
   out_1195653358946799825[9] = -nom_x[9] + true_x[9];
   out_1195653358946799825[10] = -nom_x[10] + true_x[10];
   out_1195653358946799825[11] = -nom_x[11] + true_x[11];
   out_1195653358946799825[12] = -nom_x[12] + true_x[12];
   out_1195653358946799825[13] = -nom_x[13] + true_x[13];
   out_1195653358946799825[14] = -nom_x[14] + true_x[14];
   out_1195653358946799825[15] = -nom_x[15] + true_x[15];
   out_1195653358946799825[16] = -nom_x[16] + true_x[16];
   out_1195653358946799825[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4022220117762460438) {
   out_4022220117762460438[0] = 1.0;
   out_4022220117762460438[1] = 0.0;
   out_4022220117762460438[2] = 0.0;
   out_4022220117762460438[3] = 0.0;
   out_4022220117762460438[4] = 0.0;
   out_4022220117762460438[5] = 0.0;
   out_4022220117762460438[6] = 0.0;
   out_4022220117762460438[7] = 0.0;
   out_4022220117762460438[8] = 0.0;
   out_4022220117762460438[9] = 0.0;
   out_4022220117762460438[10] = 0.0;
   out_4022220117762460438[11] = 0.0;
   out_4022220117762460438[12] = 0.0;
   out_4022220117762460438[13] = 0.0;
   out_4022220117762460438[14] = 0.0;
   out_4022220117762460438[15] = 0.0;
   out_4022220117762460438[16] = 0.0;
   out_4022220117762460438[17] = 0.0;
   out_4022220117762460438[18] = 0.0;
   out_4022220117762460438[19] = 1.0;
   out_4022220117762460438[20] = 0.0;
   out_4022220117762460438[21] = 0.0;
   out_4022220117762460438[22] = 0.0;
   out_4022220117762460438[23] = 0.0;
   out_4022220117762460438[24] = 0.0;
   out_4022220117762460438[25] = 0.0;
   out_4022220117762460438[26] = 0.0;
   out_4022220117762460438[27] = 0.0;
   out_4022220117762460438[28] = 0.0;
   out_4022220117762460438[29] = 0.0;
   out_4022220117762460438[30] = 0.0;
   out_4022220117762460438[31] = 0.0;
   out_4022220117762460438[32] = 0.0;
   out_4022220117762460438[33] = 0.0;
   out_4022220117762460438[34] = 0.0;
   out_4022220117762460438[35] = 0.0;
   out_4022220117762460438[36] = 0.0;
   out_4022220117762460438[37] = 0.0;
   out_4022220117762460438[38] = 1.0;
   out_4022220117762460438[39] = 0.0;
   out_4022220117762460438[40] = 0.0;
   out_4022220117762460438[41] = 0.0;
   out_4022220117762460438[42] = 0.0;
   out_4022220117762460438[43] = 0.0;
   out_4022220117762460438[44] = 0.0;
   out_4022220117762460438[45] = 0.0;
   out_4022220117762460438[46] = 0.0;
   out_4022220117762460438[47] = 0.0;
   out_4022220117762460438[48] = 0.0;
   out_4022220117762460438[49] = 0.0;
   out_4022220117762460438[50] = 0.0;
   out_4022220117762460438[51] = 0.0;
   out_4022220117762460438[52] = 0.0;
   out_4022220117762460438[53] = 0.0;
   out_4022220117762460438[54] = 0.0;
   out_4022220117762460438[55] = 0.0;
   out_4022220117762460438[56] = 0.0;
   out_4022220117762460438[57] = 1.0;
   out_4022220117762460438[58] = 0.0;
   out_4022220117762460438[59] = 0.0;
   out_4022220117762460438[60] = 0.0;
   out_4022220117762460438[61] = 0.0;
   out_4022220117762460438[62] = 0.0;
   out_4022220117762460438[63] = 0.0;
   out_4022220117762460438[64] = 0.0;
   out_4022220117762460438[65] = 0.0;
   out_4022220117762460438[66] = 0.0;
   out_4022220117762460438[67] = 0.0;
   out_4022220117762460438[68] = 0.0;
   out_4022220117762460438[69] = 0.0;
   out_4022220117762460438[70] = 0.0;
   out_4022220117762460438[71] = 0.0;
   out_4022220117762460438[72] = 0.0;
   out_4022220117762460438[73] = 0.0;
   out_4022220117762460438[74] = 0.0;
   out_4022220117762460438[75] = 0.0;
   out_4022220117762460438[76] = 1.0;
   out_4022220117762460438[77] = 0.0;
   out_4022220117762460438[78] = 0.0;
   out_4022220117762460438[79] = 0.0;
   out_4022220117762460438[80] = 0.0;
   out_4022220117762460438[81] = 0.0;
   out_4022220117762460438[82] = 0.0;
   out_4022220117762460438[83] = 0.0;
   out_4022220117762460438[84] = 0.0;
   out_4022220117762460438[85] = 0.0;
   out_4022220117762460438[86] = 0.0;
   out_4022220117762460438[87] = 0.0;
   out_4022220117762460438[88] = 0.0;
   out_4022220117762460438[89] = 0.0;
   out_4022220117762460438[90] = 0.0;
   out_4022220117762460438[91] = 0.0;
   out_4022220117762460438[92] = 0.0;
   out_4022220117762460438[93] = 0.0;
   out_4022220117762460438[94] = 0.0;
   out_4022220117762460438[95] = 1.0;
   out_4022220117762460438[96] = 0.0;
   out_4022220117762460438[97] = 0.0;
   out_4022220117762460438[98] = 0.0;
   out_4022220117762460438[99] = 0.0;
   out_4022220117762460438[100] = 0.0;
   out_4022220117762460438[101] = 0.0;
   out_4022220117762460438[102] = 0.0;
   out_4022220117762460438[103] = 0.0;
   out_4022220117762460438[104] = 0.0;
   out_4022220117762460438[105] = 0.0;
   out_4022220117762460438[106] = 0.0;
   out_4022220117762460438[107] = 0.0;
   out_4022220117762460438[108] = 0.0;
   out_4022220117762460438[109] = 0.0;
   out_4022220117762460438[110] = 0.0;
   out_4022220117762460438[111] = 0.0;
   out_4022220117762460438[112] = 0.0;
   out_4022220117762460438[113] = 0.0;
   out_4022220117762460438[114] = 1.0;
   out_4022220117762460438[115] = 0.0;
   out_4022220117762460438[116] = 0.0;
   out_4022220117762460438[117] = 0.0;
   out_4022220117762460438[118] = 0.0;
   out_4022220117762460438[119] = 0.0;
   out_4022220117762460438[120] = 0.0;
   out_4022220117762460438[121] = 0.0;
   out_4022220117762460438[122] = 0.0;
   out_4022220117762460438[123] = 0.0;
   out_4022220117762460438[124] = 0.0;
   out_4022220117762460438[125] = 0.0;
   out_4022220117762460438[126] = 0.0;
   out_4022220117762460438[127] = 0.0;
   out_4022220117762460438[128] = 0.0;
   out_4022220117762460438[129] = 0.0;
   out_4022220117762460438[130] = 0.0;
   out_4022220117762460438[131] = 0.0;
   out_4022220117762460438[132] = 0.0;
   out_4022220117762460438[133] = 1.0;
   out_4022220117762460438[134] = 0.0;
   out_4022220117762460438[135] = 0.0;
   out_4022220117762460438[136] = 0.0;
   out_4022220117762460438[137] = 0.0;
   out_4022220117762460438[138] = 0.0;
   out_4022220117762460438[139] = 0.0;
   out_4022220117762460438[140] = 0.0;
   out_4022220117762460438[141] = 0.0;
   out_4022220117762460438[142] = 0.0;
   out_4022220117762460438[143] = 0.0;
   out_4022220117762460438[144] = 0.0;
   out_4022220117762460438[145] = 0.0;
   out_4022220117762460438[146] = 0.0;
   out_4022220117762460438[147] = 0.0;
   out_4022220117762460438[148] = 0.0;
   out_4022220117762460438[149] = 0.0;
   out_4022220117762460438[150] = 0.0;
   out_4022220117762460438[151] = 0.0;
   out_4022220117762460438[152] = 1.0;
   out_4022220117762460438[153] = 0.0;
   out_4022220117762460438[154] = 0.0;
   out_4022220117762460438[155] = 0.0;
   out_4022220117762460438[156] = 0.0;
   out_4022220117762460438[157] = 0.0;
   out_4022220117762460438[158] = 0.0;
   out_4022220117762460438[159] = 0.0;
   out_4022220117762460438[160] = 0.0;
   out_4022220117762460438[161] = 0.0;
   out_4022220117762460438[162] = 0.0;
   out_4022220117762460438[163] = 0.0;
   out_4022220117762460438[164] = 0.0;
   out_4022220117762460438[165] = 0.0;
   out_4022220117762460438[166] = 0.0;
   out_4022220117762460438[167] = 0.0;
   out_4022220117762460438[168] = 0.0;
   out_4022220117762460438[169] = 0.0;
   out_4022220117762460438[170] = 0.0;
   out_4022220117762460438[171] = 1.0;
   out_4022220117762460438[172] = 0.0;
   out_4022220117762460438[173] = 0.0;
   out_4022220117762460438[174] = 0.0;
   out_4022220117762460438[175] = 0.0;
   out_4022220117762460438[176] = 0.0;
   out_4022220117762460438[177] = 0.0;
   out_4022220117762460438[178] = 0.0;
   out_4022220117762460438[179] = 0.0;
   out_4022220117762460438[180] = 0.0;
   out_4022220117762460438[181] = 0.0;
   out_4022220117762460438[182] = 0.0;
   out_4022220117762460438[183] = 0.0;
   out_4022220117762460438[184] = 0.0;
   out_4022220117762460438[185] = 0.0;
   out_4022220117762460438[186] = 0.0;
   out_4022220117762460438[187] = 0.0;
   out_4022220117762460438[188] = 0.0;
   out_4022220117762460438[189] = 0.0;
   out_4022220117762460438[190] = 1.0;
   out_4022220117762460438[191] = 0.0;
   out_4022220117762460438[192] = 0.0;
   out_4022220117762460438[193] = 0.0;
   out_4022220117762460438[194] = 0.0;
   out_4022220117762460438[195] = 0.0;
   out_4022220117762460438[196] = 0.0;
   out_4022220117762460438[197] = 0.0;
   out_4022220117762460438[198] = 0.0;
   out_4022220117762460438[199] = 0.0;
   out_4022220117762460438[200] = 0.0;
   out_4022220117762460438[201] = 0.0;
   out_4022220117762460438[202] = 0.0;
   out_4022220117762460438[203] = 0.0;
   out_4022220117762460438[204] = 0.0;
   out_4022220117762460438[205] = 0.0;
   out_4022220117762460438[206] = 0.0;
   out_4022220117762460438[207] = 0.0;
   out_4022220117762460438[208] = 0.0;
   out_4022220117762460438[209] = 1.0;
   out_4022220117762460438[210] = 0.0;
   out_4022220117762460438[211] = 0.0;
   out_4022220117762460438[212] = 0.0;
   out_4022220117762460438[213] = 0.0;
   out_4022220117762460438[214] = 0.0;
   out_4022220117762460438[215] = 0.0;
   out_4022220117762460438[216] = 0.0;
   out_4022220117762460438[217] = 0.0;
   out_4022220117762460438[218] = 0.0;
   out_4022220117762460438[219] = 0.0;
   out_4022220117762460438[220] = 0.0;
   out_4022220117762460438[221] = 0.0;
   out_4022220117762460438[222] = 0.0;
   out_4022220117762460438[223] = 0.0;
   out_4022220117762460438[224] = 0.0;
   out_4022220117762460438[225] = 0.0;
   out_4022220117762460438[226] = 0.0;
   out_4022220117762460438[227] = 0.0;
   out_4022220117762460438[228] = 1.0;
   out_4022220117762460438[229] = 0.0;
   out_4022220117762460438[230] = 0.0;
   out_4022220117762460438[231] = 0.0;
   out_4022220117762460438[232] = 0.0;
   out_4022220117762460438[233] = 0.0;
   out_4022220117762460438[234] = 0.0;
   out_4022220117762460438[235] = 0.0;
   out_4022220117762460438[236] = 0.0;
   out_4022220117762460438[237] = 0.0;
   out_4022220117762460438[238] = 0.0;
   out_4022220117762460438[239] = 0.0;
   out_4022220117762460438[240] = 0.0;
   out_4022220117762460438[241] = 0.0;
   out_4022220117762460438[242] = 0.0;
   out_4022220117762460438[243] = 0.0;
   out_4022220117762460438[244] = 0.0;
   out_4022220117762460438[245] = 0.0;
   out_4022220117762460438[246] = 0.0;
   out_4022220117762460438[247] = 1.0;
   out_4022220117762460438[248] = 0.0;
   out_4022220117762460438[249] = 0.0;
   out_4022220117762460438[250] = 0.0;
   out_4022220117762460438[251] = 0.0;
   out_4022220117762460438[252] = 0.0;
   out_4022220117762460438[253] = 0.0;
   out_4022220117762460438[254] = 0.0;
   out_4022220117762460438[255] = 0.0;
   out_4022220117762460438[256] = 0.0;
   out_4022220117762460438[257] = 0.0;
   out_4022220117762460438[258] = 0.0;
   out_4022220117762460438[259] = 0.0;
   out_4022220117762460438[260] = 0.0;
   out_4022220117762460438[261] = 0.0;
   out_4022220117762460438[262] = 0.0;
   out_4022220117762460438[263] = 0.0;
   out_4022220117762460438[264] = 0.0;
   out_4022220117762460438[265] = 0.0;
   out_4022220117762460438[266] = 1.0;
   out_4022220117762460438[267] = 0.0;
   out_4022220117762460438[268] = 0.0;
   out_4022220117762460438[269] = 0.0;
   out_4022220117762460438[270] = 0.0;
   out_4022220117762460438[271] = 0.0;
   out_4022220117762460438[272] = 0.0;
   out_4022220117762460438[273] = 0.0;
   out_4022220117762460438[274] = 0.0;
   out_4022220117762460438[275] = 0.0;
   out_4022220117762460438[276] = 0.0;
   out_4022220117762460438[277] = 0.0;
   out_4022220117762460438[278] = 0.0;
   out_4022220117762460438[279] = 0.0;
   out_4022220117762460438[280] = 0.0;
   out_4022220117762460438[281] = 0.0;
   out_4022220117762460438[282] = 0.0;
   out_4022220117762460438[283] = 0.0;
   out_4022220117762460438[284] = 0.0;
   out_4022220117762460438[285] = 1.0;
   out_4022220117762460438[286] = 0.0;
   out_4022220117762460438[287] = 0.0;
   out_4022220117762460438[288] = 0.0;
   out_4022220117762460438[289] = 0.0;
   out_4022220117762460438[290] = 0.0;
   out_4022220117762460438[291] = 0.0;
   out_4022220117762460438[292] = 0.0;
   out_4022220117762460438[293] = 0.0;
   out_4022220117762460438[294] = 0.0;
   out_4022220117762460438[295] = 0.0;
   out_4022220117762460438[296] = 0.0;
   out_4022220117762460438[297] = 0.0;
   out_4022220117762460438[298] = 0.0;
   out_4022220117762460438[299] = 0.0;
   out_4022220117762460438[300] = 0.0;
   out_4022220117762460438[301] = 0.0;
   out_4022220117762460438[302] = 0.0;
   out_4022220117762460438[303] = 0.0;
   out_4022220117762460438[304] = 1.0;
   out_4022220117762460438[305] = 0.0;
   out_4022220117762460438[306] = 0.0;
   out_4022220117762460438[307] = 0.0;
   out_4022220117762460438[308] = 0.0;
   out_4022220117762460438[309] = 0.0;
   out_4022220117762460438[310] = 0.0;
   out_4022220117762460438[311] = 0.0;
   out_4022220117762460438[312] = 0.0;
   out_4022220117762460438[313] = 0.0;
   out_4022220117762460438[314] = 0.0;
   out_4022220117762460438[315] = 0.0;
   out_4022220117762460438[316] = 0.0;
   out_4022220117762460438[317] = 0.0;
   out_4022220117762460438[318] = 0.0;
   out_4022220117762460438[319] = 0.0;
   out_4022220117762460438[320] = 0.0;
   out_4022220117762460438[321] = 0.0;
   out_4022220117762460438[322] = 0.0;
   out_4022220117762460438[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_573638959019262461) {
   out_573638959019262461[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_573638959019262461[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_573638959019262461[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_573638959019262461[3] = dt*state[12] + state[3];
   out_573638959019262461[4] = dt*state[13] + state[4];
   out_573638959019262461[5] = dt*state[14] + state[5];
   out_573638959019262461[6] = state[6];
   out_573638959019262461[7] = state[7];
   out_573638959019262461[8] = state[8];
   out_573638959019262461[9] = state[9];
   out_573638959019262461[10] = state[10];
   out_573638959019262461[11] = state[11];
   out_573638959019262461[12] = state[12];
   out_573638959019262461[13] = state[13];
   out_573638959019262461[14] = state[14];
   out_573638959019262461[15] = state[15];
   out_573638959019262461[16] = state[16];
   out_573638959019262461[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7727313238387066272) {
   out_7727313238387066272[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7727313238387066272[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7727313238387066272[2] = 0;
   out_7727313238387066272[3] = 0;
   out_7727313238387066272[4] = 0;
   out_7727313238387066272[5] = 0;
   out_7727313238387066272[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7727313238387066272[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7727313238387066272[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7727313238387066272[9] = 0;
   out_7727313238387066272[10] = 0;
   out_7727313238387066272[11] = 0;
   out_7727313238387066272[12] = 0;
   out_7727313238387066272[13] = 0;
   out_7727313238387066272[14] = 0;
   out_7727313238387066272[15] = 0;
   out_7727313238387066272[16] = 0;
   out_7727313238387066272[17] = 0;
   out_7727313238387066272[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7727313238387066272[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7727313238387066272[20] = 0;
   out_7727313238387066272[21] = 0;
   out_7727313238387066272[22] = 0;
   out_7727313238387066272[23] = 0;
   out_7727313238387066272[24] = 0;
   out_7727313238387066272[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7727313238387066272[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7727313238387066272[27] = 0;
   out_7727313238387066272[28] = 0;
   out_7727313238387066272[29] = 0;
   out_7727313238387066272[30] = 0;
   out_7727313238387066272[31] = 0;
   out_7727313238387066272[32] = 0;
   out_7727313238387066272[33] = 0;
   out_7727313238387066272[34] = 0;
   out_7727313238387066272[35] = 0;
   out_7727313238387066272[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7727313238387066272[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7727313238387066272[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7727313238387066272[39] = 0;
   out_7727313238387066272[40] = 0;
   out_7727313238387066272[41] = 0;
   out_7727313238387066272[42] = 0;
   out_7727313238387066272[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7727313238387066272[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7727313238387066272[45] = 0;
   out_7727313238387066272[46] = 0;
   out_7727313238387066272[47] = 0;
   out_7727313238387066272[48] = 0;
   out_7727313238387066272[49] = 0;
   out_7727313238387066272[50] = 0;
   out_7727313238387066272[51] = 0;
   out_7727313238387066272[52] = 0;
   out_7727313238387066272[53] = 0;
   out_7727313238387066272[54] = 0;
   out_7727313238387066272[55] = 0;
   out_7727313238387066272[56] = 0;
   out_7727313238387066272[57] = 1;
   out_7727313238387066272[58] = 0;
   out_7727313238387066272[59] = 0;
   out_7727313238387066272[60] = 0;
   out_7727313238387066272[61] = 0;
   out_7727313238387066272[62] = 0;
   out_7727313238387066272[63] = 0;
   out_7727313238387066272[64] = 0;
   out_7727313238387066272[65] = 0;
   out_7727313238387066272[66] = dt;
   out_7727313238387066272[67] = 0;
   out_7727313238387066272[68] = 0;
   out_7727313238387066272[69] = 0;
   out_7727313238387066272[70] = 0;
   out_7727313238387066272[71] = 0;
   out_7727313238387066272[72] = 0;
   out_7727313238387066272[73] = 0;
   out_7727313238387066272[74] = 0;
   out_7727313238387066272[75] = 0;
   out_7727313238387066272[76] = 1;
   out_7727313238387066272[77] = 0;
   out_7727313238387066272[78] = 0;
   out_7727313238387066272[79] = 0;
   out_7727313238387066272[80] = 0;
   out_7727313238387066272[81] = 0;
   out_7727313238387066272[82] = 0;
   out_7727313238387066272[83] = 0;
   out_7727313238387066272[84] = 0;
   out_7727313238387066272[85] = dt;
   out_7727313238387066272[86] = 0;
   out_7727313238387066272[87] = 0;
   out_7727313238387066272[88] = 0;
   out_7727313238387066272[89] = 0;
   out_7727313238387066272[90] = 0;
   out_7727313238387066272[91] = 0;
   out_7727313238387066272[92] = 0;
   out_7727313238387066272[93] = 0;
   out_7727313238387066272[94] = 0;
   out_7727313238387066272[95] = 1;
   out_7727313238387066272[96] = 0;
   out_7727313238387066272[97] = 0;
   out_7727313238387066272[98] = 0;
   out_7727313238387066272[99] = 0;
   out_7727313238387066272[100] = 0;
   out_7727313238387066272[101] = 0;
   out_7727313238387066272[102] = 0;
   out_7727313238387066272[103] = 0;
   out_7727313238387066272[104] = dt;
   out_7727313238387066272[105] = 0;
   out_7727313238387066272[106] = 0;
   out_7727313238387066272[107] = 0;
   out_7727313238387066272[108] = 0;
   out_7727313238387066272[109] = 0;
   out_7727313238387066272[110] = 0;
   out_7727313238387066272[111] = 0;
   out_7727313238387066272[112] = 0;
   out_7727313238387066272[113] = 0;
   out_7727313238387066272[114] = 1;
   out_7727313238387066272[115] = 0;
   out_7727313238387066272[116] = 0;
   out_7727313238387066272[117] = 0;
   out_7727313238387066272[118] = 0;
   out_7727313238387066272[119] = 0;
   out_7727313238387066272[120] = 0;
   out_7727313238387066272[121] = 0;
   out_7727313238387066272[122] = 0;
   out_7727313238387066272[123] = 0;
   out_7727313238387066272[124] = 0;
   out_7727313238387066272[125] = 0;
   out_7727313238387066272[126] = 0;
   out_7727313238387066272[127] = 0;
   out_7727313238387066272[128] = 0;
   out_7727313238387066272[129] = 0;
   out_7727313238387066272[130] = 0;
   out_7727313238387066272[131] = 0;
   out_7727313238387066272[132] = 0;
   out_7727313238387066272[133] = 1;
   out_7727313238387066272[134] = 0;
   out_7727313238387066272[135] = 0;
   out_7727313238387066272[136] = 0;
   out_7727313238387066272[137] = 0;
   out_7727313238387066272[138] = 0;
   out_7727313238387066272[139] = 0;
   out_7727313238387066272[140] = 0;
   out_7727313238387066272[141] = 0;
   out_7727313238387066272[142] = 0;
   out_7727313238387066272[143] = 0;
   out_7727313238387066272[144] = 0;
   out_7727313238387066272[145] = 0;
   out_7727313238387066272[146] = 0;
   out_7727313238387066272[147] = 0;
   out_7727313238387066272[148] = 0;
   out_7727313238387066272[149] = 0;
   out_7727313238387066272[150] = 0;
   out_7727313238387066272[151] = 0;
   out_7727313238387066272[152] = 1;
   out_7727313238387066272[153] = 0;
   out_7727313238387066272[154] = 0;
   out_7727313238387066272[155] = 0;
   out_7727313238387066272[156] = 0;
   out_7727313238387066272[157] = 0;
   out_7727313238387066272[158] = 0;
   out_7727313238387066272[159] = 0;
   out_7727313238387066272[160] = 0;
   out_7727313238387066272[161] = 0;
   out_7727313238387066272[162] = 0;
   out_7727313238387066272[163] = 0;
   out_7727313238387066272[164] = 0;
   out_7727313238387066272[165] = 0;
   out_7727313238387066272[166] = 0;
   out_7727313238387066272[167] = 0;
   out_7727313238387066272[168] = 0;
   out_7727313238387066272[169] = 0;
   out_7727313238387066272[170] = 0;
   out_7727313238387066272[171] = 1;
   out_7727313238387066272[172] = 0;
   out_7727313238387066272[173] = 0;
   out_7727313238387066272[174] = 0;
   out_7727313238387066272[175] = 0;
   out_7727313238387066272[176] = 0;
   out_7727313238387066272[177] = 0;
   out_7727313238387066272[178] = 0;
   out_7727313238387066272[179] = 0;
   out_7727313238387066272[180] = 0;
   out_7727313238387066272[181] = 0;
   out_7727313238387066272[182] = 0;
   out_7727313238387066272[183] = 0;
   out_7727313238387066272[184] = 0;
   out_7727313238387066272[185] = 0;
   out_7727313238387066272[186] = 0;
   out_7727313238387066272[187] = 0;
   out_7727313238387066272[188] = 0;
   out_7727313238387066272[189] = 0;
   out_7727313238387066272[190] = 1;
   out_7727313238387066272[191] = 0;
   out_7727313238387066272[192] = 0;
   out_7727313238387066272[193] = 0;
   out_7727313238387066272[194] = 0;
   out_7727313238387066272[195] = 0;
   out_7727313238387066272[196] = 0;
   out_7727313238387066272[197] = 0;
   out_7727313238387066272[198] = 0;
   out_7727313238387066272[199] = 0;
   out_7727313238387066272[200] = 0;
   out_7727313238387066272[201] = 0;
   out_7727313238387066272[202] = 0;
   out_7727313238387066272[203] = 0;
   out_7727313238387066272[204] = 0;
   out_7727313238387066272[205] = 0;
   out_7727313238387066272[206] = 0;
   out_7727313238387066272[207] = 0;
   out_7727313238387066272[208] = 0;
   out_7727313238387066272[209] = 1;
   out_7727313238387066272[210] = 0;
   out_7727313238387066272[211] = 0;
   out_7727313238387066272[212] = 0;
   out_7727313238387066272[213] = 0;
   out_7727313238387066272[214] = 0;
   out_7727313238387066272[215] = 0;
   out_7727313238387066272[216] = 0;
   out_7727313238387066272[217] = 0;
   out_7727313238387066272[218] = 0;
   out_7727313238387066272[219] = 0;
   out_7727313238387066272[220] = 0;
   out_7727313238387066272[221] = 0;
   out_7727313238387066272[222] = 0;
   out_7727313238387066272[223] = 0;
   out_7727313238387066272[224] = 0;
   out_7727313238387066272[225] = 0;
   out_7727313238387066272[226] = 0;
   out_7727313238387066272[227] = 0;
   out_7727313238387066272[228] = 1;
   out_7727313238387066272[229] = 0;
   out_7727313238387066272[230] = 0;
   out_7727313238387066272[231] = 0;
   out_7727313238387066272[232] = 0;
   out_7727313238387066272[233] = 0;
   out_7727313238387066272[234] = 0;
   out_7727313238387066272[235] = 0;
   out_7727313238387066272[236] = 0;
   out_7727313238387066272[237] = 0;
   out_7727313238387066272[238] = 0;
   out_7727313238387066272[239] = 0;
   out_7727313238387066272[240] = 0;
   out_7727313238387066272[241] = 0;
   out_7727313238387066272[242] = 0;
   out_7727313238387066272[243] = 0;
   out_7727313238387066272[244] = 0;
   out_7727313238387066272[245] = 0;
   out_7727313238387066272[246] = 0;
   out_7727313238387066272[247] = 1;
   out_7727313238387066272[248] = 0;
   out_7727313238387066272[249] = 0;
   out_7727313238387066272[250] = 0;
   out_7727313238387066272[251] = 0;
   out_7727313238387066272[252] = 0;
   out_7727313238387066272[253] = 0;
   out_7727313238387066272[254] = 0;
   out_7727313238387066272[255] = 0;
   out_7727313238387066272[256] = 0;
   out_7727313238387066272[257] = 0;
   out_7727313238387066272[258] = 0;
   out_7727313238387066272[259] = 0;
   out_7727313238387066272[260] = 0;
   out_7727313238387066272[261] = 0;
   out_7727313238387066272[262] = 0;
   out_7727313238387066272[263] = 0;
   out_7727313238387066272[264] = 0;
   out_7727313238387066272[265] = 0;
   out_7727313238387066272[266] = 1;
   out_7727313238387066272[267] = 0;
   out_7727313238387066272[268] = 0;
   out_7727313238387066272[269] = 0;
   out_7727313238387066272[270] = 0;
   out_7727313238387066272[271] = 0;
   out_7727313238387066272[272] = 0;
   out_7727313238387066272[273] = 0;
   out_7727313238387066272[274] = 0;
   out_7727313238387066272[275] = 0;
   out_7727313238387066272[276] = 0;
   out_7727313238387066272[277] = 0;
   out_7727313238387066272[278] = 0;
   out_7727313238387066272[279] = 0;
   out_7727313238387066272[280] = 0;
   out_7727313238387066272[281] = 0;
   out_7727313238387066272[282] = 0;
   out_7727313238387066272[283] = 0;
   out_7727313238387066272[284] = 0;
   out_7727313238387066272[285] = 1;
   out_7727313238387066272[286] = 0;
   out_7727313238387066272[287] = 0;
   out_7727313238387066272[288] = 0;
   out_7727313238387066272[289] = 0;
   out_7727313238387066272[290] = 0;
   out_7727313238387066272[291] = 0;
   out_7727313238387066272[292] = 0;
   out_7727313238387066272[293] = 0;
   out_7727313238387066272[294] = 0;
   out_7727313238387066272[295] = 0;
   out_7727313238387066272[296] = 0;
   out_7727313238387066272[297] = 0;
   out_7727313238387066272[298] = 0;
   out_7727313238387066272[299] = 0;
   out_7727313238387066272[300] = 0;
   out_7727313238387066272[301] = 0;
   out_7727313238387066272[302] = 0;
   out_7727313238387066272[303] = 0;
   out_7727313238387066272[304] = 1;
   out_7727313238387066272[305] = 0;
   out_7727313238387066272[306] = 0;
   out_7727313238387066272[307] = 0;
   out_7727313238387066272[308] = 0;
   out_7727313238387066272[309] = 0;
   out_7727313238387066272[310] = 0;
   out_7727313238387066272[311] = 0;
   out_7727313238387066272[312] = 0;
   out_7727313238387066272[313] = 0;
   out_7727313238387066272[314] = 0;
   out_7727313238387066272[315] = 0;
   out_7727313238387066272[316] = 0;
   out_7727313238387066272[317] = 0;
   out_7727313238387066272[318] = 0;
   out_7727313238387066272[319] = 0;
   out_7727313238387066272[320] = 0;
   out_7727313238387066272[321] = 0;
   out_7727313238387066272[322] = 0;
   out_7727313238387066272[323] = 1;
}
void h_4(double *state, double *unused, double *out_1574587782542937358) {
   out_1574587782542937358[0] = state[6] + state[9];
   out_1574587782542937358[1] = state[7] + state[10];
   out_1574587782542937358[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_9174738502801541173) {
   out_9174738502801541173[0] = 0;
   out_9174738502801541173[1] = 0;
   out_9174738502801541173[2] = 0;
   out_9174738502801541173[3] = 0;
   out_9174738502801541173[4] = 0;
   out_9174738502801541173[5] = 0;
   out_9174738502801541173[6] = 1;
   out_9174738502801541173[7] = 0;
   out_9174738502801541173[8] = 0;
   out_9174738502801541173[9] = 1;
   out_9174738502801541173[10] = 0;
   out_9174738502801541173[11] = 0;
   out_9174738502801541173[12] = 0;
   out_9174738502801541173[13] = 0;
   out_9174738502801541173[14] = 0;
   out_9174738502801541173[15] = 0;
   out_9174738502801541173[16] = 0;
   out_9174738502801541173[17] = 0;
   out_9174738502801541173[18] = 0;
   out_9174738502801541173[19] = 0;
   out_9174738502801541173[20] = 0;
   out_9174738502801541173[21] = 0;
   out_9174738502801541173[22] = 0;
   out_9174738502801541173[23] = 0;
   out_9174738502801541173[24] = 0;
   out_9174738502801541173[25] = 1;
   out_9174738502801541173[26] = 0;
   out_9174738502801541173[27] = 0;
   out_9174738502801541173[28] = 1;
   out_9174738502801541173[29] = 0;
   out_9174738502801541173[30] = 0;
   out_9174738502801541173[31] = 0;
   out_9174738502801541173[32] = 0;
   out_9174738502801541173[33] = 0;
   out_9174738502801541173[34] = 0;
   out_9174738502801541173[35] = 0;
   out_9174738502801541173[36] = 0;
   out_9174738502801541173[37] = 0;
   out_9174738502801541173[38] = 0;
   out_9174738502801541173[39] = 0;
   out_9174738502801541173[40] = 0;
   out_9174738502801541173[41] = 0;
   out_9174738502801541173[42] = 0;
   out_9174738502801541173[43] = 0;
   out_9174738502801541173[44] = 1;
   out_9174738502801541173[45] = 0;
   out_9174738502801541173[46] = 0;
   out_9174738502801541173[47] = 1;
   out_9174738502801541173[48] = 0;
   out_9174738502801541173[49] = 0;
   out_9174738502801541173[50] = 0;
   out_9174738502801541173[51] = 0;
   out_9174738502801541173[52] = 0;
   out_9174738502801541173[53] = 0;
}
void h_10(double *state, double *unused, double *out_9067770934847269573) {
   out_9067770934847269573[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_9067770934847269573[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_9067770934847269573[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3107776501730799288) {
   out_3107776501730799288[0] = 0;
   out_3107776501730799288[1] = 9.8100000000000005*cos(state[1]);
   out_3107776501730799288[2] = 0;
   out_3107776501730799288[3] = 0;
   out_3107776501730799288[4] = -state[8];
   out_3107776501730799288[5] = state[7];
   out_3107776501730799288[6] = 0;
   out_3107776501730799288[7] = state[5];
   out_3107776501730799288[8] = -state[4];
   out_3107776501730799288[9] = 0;
   out_3107776501730799288[10] = 0;
   out_3107776501730799288[11] = 0;
   out_3107776501730799288[12] = 1;
   out_3107776501730799288[13] = 0;
   out_3107776501730799288[14] = 0;
   out_3107776501730799288[15] = 1;
   out_3107776501730799288[16] = 0;
   out_3107776501730799288[17] = 0;
   out_3107776501730799288[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3107776501730799288[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3107776501730799288[20] = 0;
   out_3107776501730799288[21] = state[8];
   out_3107776501730799288[22] = 0;
   out_3107776501730799288[23] = -state[6];
   out_3107776501730799288[24] = -state[5];
   out_3107776501730799288[25] = 0;
   out_3107776501730799288[26] = state[3];
   out_3107776501730799288[27] = 0;
   out_3107776501730799288[28] = 0;
   out_3107776501730799288[29] = 0;
   out_3107776501730799288[30] = 0;
   out_3107776501730799288[31] = 1;
   out_3107776501730799288[32] = 0;
   out_3107776501730799288[33] = 0;
   out_3107776501730799288[34] = 1;
   out_3107776501730799288[35] = 0;
   out_3107776501730799288[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3107776501730799288[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3107776501730799288[38] = 0;
   out_3107776501730799288[39] = -state[7];
   out_3107776501730799288[40] = state[6];
   out_3107776501730799288[41] = 0;
   out_3107776501730799288[42] = state[4];
   out_3107776501730799288[43] = -state[3];
   out_3107776501730799288[44] = 0;
   out_3107776501730799288[45] = 0;
   out_3107776501730799288[46] = 0;
   out_3107776501730799288[47] = 0;
   out_3107776501730799288[48] = 0;
   out_3107776501730799288[49] = 0;
   out_3107776501730799288[50] = 1;
   out_3107776501730799288[51] = 0;
   out_3107776501730799288[52] = 0;
   out_3107776501730799288[53] = 1;
}
void h_13(double *state, double *unused, double *out_1089679770148776292) {
   out_1089679770148776292[0] = state[3];
   out_1089679770148776292[1] = state[4];
   out_1089679770148776292[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6059731745575677642) {
   out_6059731745575677642[0] = 0;
   out_6059731745575677642[1] = 0;
   out_6059731745575677642[2] = 0;
   out_6059731745575677642[3] = 1;
   out_6059731745575677642[4] = 0;
   out_6059731745575677642[5] = 0;
   out_6059731745575677642[6] = 0;
   out_6059731745575677642[7] = 0;
   out_6059731745575677642[8] = 0;
   out_6059731745575677642[9] = 0;
   out_6059731745575677642[10] = 0;
   out_6059731745575677642[11] = 0;
   out_6059731745575677642[12] = 0;
   out_6059731745575677642[13] = 0;
   out_6059731745575677642[14] = 0;
   out_6059731745575677642[15] = 0;
   out_6059731745575677642[16] = 0;
   out_6059731745575677642[17] = 0;
   out_6059731745575677642[18] = 0;
   out_6059731745575677642[19] = 0;
   out_6059731745575677642[20] = 0;
   out_6059731745575677642[21] = 0;
   out_6059731745575677642[22] = 1;
   out_6059731745575677642[23] = 0;
   out_6059731745575677642[24] = 0;
   out_6059731745575677642[25] = 0;
   out_6059731745575677642[26] = 0;
   out_6059731745575677642[27] = 0;
   out_6059731745575677642[28] = 0;
   out_6059731745575677642[29] = 0;
   out_6059731745575677642[30] = 0;
   out_6059731745575677642[31] = 0;
   out_6059731745575677642[32] = 0;
   out_6059731745575677642[33] = 0;
   out_6059731745575677642[34] = 0;
   out_6059731745575677642[35] = 0;
   out_6059731745575677642[36] = 0;
   out_6059731745575677642[37] = 0;
   out_6059731745575677642[38] = 0;
   out_6059731745575677642[39] = 0;
   out_6059731745575677642[40] = 0;
   out_6059731745575677642[41] = 1;
   out_6059731745575677642[42] = 0;
   out_6059731745575677642[43] = 0;
   out_6059731745575677642[44] = 0;
   out_6059731745575677642[45] = 0;
   out_6059731745575677642[46] = 0;
   out_6059731745575677642[47] = 0;
   out_6059731745575677642[48] = 0;
   out_6059731745575677642[49] = 0;
   out_6059731745575677642[50] = 0;
   out_6059731745575677642[51] = 0;
   out_6059731745575677642[52] = 0;
   out_6059731745575677642[53] = 0;
}
void h_14(double *state, double *unused, double *out_41555095050834232) {
   out_41555095050834232[0] = state[6];
   out_41555095050834232[1] = state[7];
   out_41555095050834232[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6091950070506168877) {
   out_6091950070506168877[0] = 0;
   out_6091950070506168877[1] = 0;
   out_6091950070506168877[2] = 0;
   out_6091950070506168877[3] = 0;
   out_6091950070506168877[4] = 0;
   out_6091950070506168877[5] = 0;
   out_6091950070506168877[6] = 1;
   out_6091950070506168877[7] = 0;
   out_6091950070506168877[8] = 0;
   out_6091950070506168877[9] = 0;
   out_6091950070506168877[10] = 0;
   out_6091950070506168877[11] = 0;
   out_6091950070506168877[12] = 0;
   out_6091950070506168877[13] = 0;
   out_6091950070506168877[14] = 0;
   out_6091950070506168877[15] = 0;
   out_6091950070506168877[16] = 0;
   out_6091950070506168877[17] = 0;
   out_6091950070506168877[18] = 0;
   out_6091950070506168877[19] = 0;
   out_6091950070506168877[20] = 0;
   out_6091950070506168877[21] = 0;
   out_6091950070506168877[22] = 0;
   out_6091950070506168877[23] = 0;
   out_6091950070506168877[24] = 0;
   out_6091950070506168877[25] = 1;
   out_6091950070506168877[26] = 0;
   out_6091950070506168877[27] = 0;
   out_6091950070506168877[28] = 0;
   out_6091950070506168877[29] = 0;
   out_6091950070506168877[30] = 0;
   out_6091950070506168877[31] = 0;
   out_6091950070506168877[32] = 0;
   out_6091950070506168877[33] = 0;
   out_6091950070506168877[34] = 0;
   out_6091950070506168877[35] = 0;
   out_6091950070506168877[36] = 0;
   out_6091950070506168877[37] = 0;
   out_6091950070506168877[38] = 0;
   out_6091950070506168877[39] = 0;
   out_6091950070506168877[40] = 0;
   out_6091950070506168877[41] = 0;
   out_6091950070506168877[42] = 0;
   out_6091950070506168877[43] = 0;
   out_6091950070506168877[44] = 1;
   out_6091950070506168877[45] = 0;
   out_6091950070506168877[46] = 0;
   out_6091950070506168877[47] = 0;
   out_6091950070506168877[48] = 0;
   out_6091950070506168877[49] = 0;
   out_6091950070506168877[50] = 0;
   out_6091950070506168877[51] = 0;
   out_6091950070506168877[52] = 0;
   out_6091950070506168877[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_7199008258596550885) {
  err_fun(nom_x, delta_x, out_7199008258596550885);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1195653358946799825) {
  inv_err_fun(nom_x, true_x, out_1195653358946799825);
}
void pose_H_mod_fun(double *state, double *out_4022220117762460438) {
  H_mod_fun(state, out_4022220117762460438);
}
void pose_f_fun(double *state, double dt, double *out_573638959019262461) {
  f_fun(state,  dt, out_573638959019262461);
}
void pose_F_fun(double *state, double dt, double *out_7727313238387066272) {
  F_fun(state,  dt, out_7727313238387066272);
}
void pose_h_4(double *state, double *unused, double *out_1574587782542937358) {
  h_4(state, unused, out_1574587782542937358);
}
void pose_H_4(double *state, double *unused, double *out_9174738502801541173) {
  H_4(state, unused, out_9174738502801541173);
}
void pose_h_10(double *state, double *unused, double *out_9067770934847269573) {
  h_10(state, unused, out_9067770934847269573);
}
void pose_H_10(double *state, double *unused, double *out_3107776501730799288) {
  H_10(state, unused, out_3107776501730799288);
}
void pose_h_13(double *state, double *unused, double *out_1089679770148776292) {
  h_13(state, unused, out_1089679770148776292);
}
void pose_H_13(double *state, double *unused, double *out_6059731745575677642) {
  H_13(state, unused, out_6059731745575677642);
}
void pose_h_14(double *state, double *unused, double *out_41555095050834232) {
  h_14(state, unused, out_41555095050834232);
}
void pose_H_14(double *state, double *unused, double *out_6091950070506168877) {
  H_14(state, unused, out_6091950070506168877);
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
