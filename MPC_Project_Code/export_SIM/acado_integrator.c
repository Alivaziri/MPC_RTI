/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
/* Vector of auxiliary variables; number of elements: 13. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[1]));
a[1] = ((a[0])*(a[0]));
a[2] = ((xd[3])*(xd[3]));
a[3] = (sin(xd[1]));
a[4] = (cos(xd[1]));
a[5] = (sin(xd[1]));
a[6] = (cos(xd[1]));
a[7] = ((a[6])*(a[6]));
a[8] = ((xd[3])*(xd[3]));
a[9] = (sin(xd[1]));
a[10] = (cos(xd[1]));
a[11] = (sin(xd[1]));
a[12] = (sin(xd[1]));

/* Compute outputs: */
out[0] = xd[2];
out[1] = xd[3];
out[2] = (((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(5.0000000000000003e-02)*a[2])*a[3])+(((real_t)(9.8100000000000009e-01)*a[4])*a[5]))+u[0]));
out[3] = ((((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[10])-((real_t)(9.8100000000000005e+00)*a[11]))-((real_t)(9.8100000000000009e-01)*a[12])))/(real_t)(5.0000000000000000e-01));
}

void acado_rhs_ext(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 24;
/* Vector of auxiliary variables; number of elements: 96. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[1]));
a[1] = ((a[0])*(a[0]));
a[2] = ((xd[3])*(xd[3]));
a[3] = (sin(xd[1]));
a[4] = (cos(xd[1]));
a[5] = (sin(xd[1]));
a[6] = (cos(xd[1]));
a[7] = ((a[6])*(a[6]));
a[8] = ((xd[3])*(xd[3]));
a[9] = (sin(xd[1]));
a[10] = (cos(xd[1]));
a[11] = (sin(xd[1]));
a[12] = (sin(xd[1]));
a[13] = ((real_t)(2.0000000000000000e+00)*a[0]);
a[14] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[15] = (xd[8]*a[14]);
a[16] = (a[13]*a[15]);
a[17] = ((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)));
a[18] = (a[17]*a[17]);
a[19] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[20] = (a[19]*xd[16]);
a[21] = (cos(xd[1]));
a[22] = (xd[8]*a[21]);
a[23] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[24] = (xd[8]*a[23]);
a[25] = (cos(xd[1]));
a[26] = (xd[8]*a[25]);
a[27] = (xd[9]*a[14]);
a[28] = (a[13]*a[27]);
a[29] = (a[19]*xd[17]);
a[30] = (xd[9]*a[21]);
a[31] = (xd[9]*a[23]);
a[32] = (xd[9]*a[25]);
a[33] = (xd[10]*a[14]);
a[34] = (a[13]*a[33]);
a[35] = (a[19]*xd[18]);
a[36] = (xd[10]*a[21]);
a[37] = (xd[10]*a[23]);
a[38] = (xd[10]*a[25]);
a[39] = (xd[11]*a[14]);
a[40] = (a[13]*a[39]);
a[41] = (a[19]*xd[19]);
a[42] = (xd[11]*a[21]);
a[43] = (xd[11]*a[23]);
a[44] = (xd[11]*a[25]);
a[45] = ((real_t)(2.0000000000000000e+00)*a[6]);
a[46] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[47] = (xd[8]*a[46]);
a[48] = (a[45]*a[47]);
a[49] = ((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)));
a[50] = (a[49]*a[49]);
a[51] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[52] = (a[51]*xd[16]);
a[53] = (cos(xd[1]));
a[54] = (xd[8]*a[53]);
a[55] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[56] = (xd[8]*a[55]);
a[57] = (cos(xd[1]));
a[58] = (xd[8]*a[57]);
a[59] = (cos(xd[1]));
a[60] = (xd[8]*a[59]);
a[61] = ((real_t)(1.0000000000000000e+00)/(real_t)(5.0000000000000000e-01));
a[62] = (xd[9]*a[46]);
a[63] = (a[45]*a[62]);
a[64] = (a[51]*xd[17]);
a[65] = (xd[9]*a[53]);
a[66] = (xd[9]*a[55]);
a[67] = (xd[9]*a[57]);
a[68] = (xd[9]*a[59]);
a[69] = (xd[10]*a[46]);
a[70] = (a[45]*a[69]);
a[71] = (a[51]*xd[18]);
a[72] = (xd[10]*a[53]);
a[73] = (xd[10]*a[55]);
a[74] = (xd[10]*a[57]);
a[75] = (xd[10]*a[59]);
a[76] = (xd[11]*a[46]);
a[77] = (a[45]*a[76]);
a[78] = (a[51]*xd[19]);
a[79] = (xd[11]*a[53]);
a[80] = (xd[11]*a[55]);
a[81] = (xd[11]*a[57]);
a[82] = (xd[11]*a[59]);
a[83] = (xd[21]*a[14]);
a[84] = (a[13]*a[83]);
a[85] = (a[19]*xd[23]);
a[86] = (xd[21]*a[21]);
a[87] = (xd[21]*a[23]);
a[88] = (xd[21]*a[25]);
a[89] = (xd[21]*a[46]);
a[90] = (a[45]*a[89]);
a[91] = (a[51]*xd[23]);
a[92] = (xd[21]*a[53]);
a[93] = (xd[21]*a[55]);
a[94] = (xd[21]*a[57]);
a[95] = (xd[21]*a[59]);

/* Compute outputs: */
out[0] = xd[2];
out[1] = xd[3];
out[2] = (((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(5.0000000000000003e-02)*a[2])*a[3])+(((real_t)(9.8100000000000009e-01)*a[4])*a[5]))+u[0]));
out[3] = ((((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[10])-((real_t)(9.8100000000000005e+00)*a[11]))-((real_t)(9.8100000000000009e-01)*a[12])))/(real_t)(5.0000000000000000e-01));
out[4] = xd[12];
out[5] = xd[13];
out[6] = xd[14];
out[7] = xd[15];
out[8] = xd[16];
out[9] = xd[17];
out[10] = xd[18];
out[11] = xd[19];
out[12] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[16])*a[18]))*(((((real_t)(5.0000000000000003e-02)*a[2])*a[3])+(((real_t)(9.8100000000000009e-01)*a[4])*a[5]))+u[0]))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(5.0000000000000003e-02)*a[20])*a[3])+(((real_t)(5.0000000000000003e-02)*a[2])*a[22]))+((((real_t)(9.8100000000000009e-01)*a[24])*a[5])+(((real_t)(9.8100000000000009e-01)*a[4])*a[26])))));
out[13] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[28])*a[18]))*(((((real_t)(5.0000000000000003e-02)*a[2])*a[3])+(((real_t)(9.8100000000000009e-01)*a[4])*a[5]))+u[0]))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(5.0000000000000003e-02)*a[29])*a[3])+(((real_t)(5.0000000000000003e-02)*a[2])*a[30]))+((((real_t)(9.8100000000000009e-01)*a[31])*a[5])+(((real_t)(9.8100000000000009e-01)*a[4])*a[32])))));
out[14] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[34])*a[18]))*(((((real_t)(5.0000000000000003e-02)*a[2])*a[3])+(((real_t)(9.8100000000000009e-01)*a[4])*a[5]))+u[0]))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(5.0000000000000003e-02)*a[35])*a[3])+(((real_t)(5.0000000000000003e-02)*a[2])*a[36]))+((((real_t)(9.8100000000000009e-01)*a[37])*a[5])+(((real_t)(9.8100000000000009e-01)*a[4])*a[38])))));
out[15] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[40])*a[18]))*(((((real_t)(5.0000000000000003e-02)*a[2])*a[3])+(((real_t)(9.8100000000000009e-01)*a[4])*a[5]))+u[0]))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(5.0000000000000003e-02)*a[41])*a[3])+(((real_t)(5.0000000000000003e-02)*a[2])*a[42]))+((((real_t)(9.8100000000000009e-01)*a[43])*a[5])+(((real_t)(9.8100000000000009e-01)*a[4])*a[44])))));
out[16] = (((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[48])*a[50]))*(((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[10])-((real_t)(9.8100000000000005e+00)*a[11]))-((real_t)(9.8100000000000009e-01)*a[12])))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*((((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[52])*a[9])+(((real_t)(5.0000000000000003e-02)*a[8])*a[54])))*a[10])+(((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[56]))-((real_t)(9.8100000000000005e+00)*a[58]))-((real_t)(9.8100000000000009e-01)*a[60]))))*a[61]);
out[17] = (((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[63])*a[50]))*(((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[10])-((real_t)(9.8100000000000005e+00)*a[11]))-((real_t)(9.8100000000000009e-01)*a[12])))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*((((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[64])*a[9])+(((real_t)(5.0000000000000003e-02)*a[8])*a[65])))*a[10])+(((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[66]))-((real_t)(9.8100000000000005e+00)*a[67]))-((real_t)(9.8100000000000009e-01)*a[68]))))*a[61]);
out[18] = (((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[70])*a[50]))*(((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[10])-((real_t)(9.8100000000000005e+00)*a[11]))-((real_t)(9.8100000000000009e-01)*a[12])))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*((((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[71])*a[9])+(((real_t)(5.0000000000000003e-02)*a[8])*a[72])))*a[10])+(((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[73]))-((real_t)(9.8100000000000005e+00)*a[74]))-((real_t)(9.8100000000000009e-01)*a[75]))))*a[61]);
out[19] = (((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[77])*a[50]))*(((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[10])-((real_t)(9.8100000000000005e+00)*a[11]))-((real_t)(9.8100000000000009e-01)*a[12])))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*((((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[78])*a[9])+(((real_t)(5.0000000000000003e-02)*a[8])*a[79])))*a[10])+(((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[80]))-((real_t)(9.8100000000000005e+00)*a[81]))-((real_t)(9.8100000000000009e-01)*a[82]))))*a[61]);
out[20] = xd[22];
out[21] = xd[23];
out[22] = (((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[84])*a[18]))*(((((real_t)(5.0000000000000003e-02)*a[2])*a[3])+(((real_t)(9.8100000000000009e-01)*a[4])*a[5]))+u[0]))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((((real_t)(5.0000000000000003e-02)*a[85])*a[3])+(((real_t)(5.0000000000000003e-02)*a[2])*a[86]))+((((real_t)(9.8100000000000009e-01)*a[87])*a[5])+(((real_t)(9.8100000000000009e-01)*a[4])*a[88])))))+((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[1])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01))));
out[23] = ((((((real_t)(0.0000000000000000e+00)-(((real_t)(-1.0000000000000001e-01)*a[90])*a[50]))*(((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[10])-((real_t)(9.8100000000000005e+00)*a[11]))-((real_t)(9.8100000000000009e-01)*a[12])))+(((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*((((((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[91])*a[9])+(((real_t)(5.0000000000000003e-02)*a[8])*a[92])))*a[10])+(((real_t)(0.0000000000000000e+00)-((((real_t)(5.0000000000000003e-02)*a[8])*a[9])+u[0]))*a[93]))-((real_t)(9.8100000000000005e+00)*a[94]))-((real_t)(9.8100000000000009e-01)*a[95]))))*a[61])+((((real_t)(1.0000000000000000e+00)/((((real_t)(-1.0000000000000001e-01)*a[7])+(real_t)(1.0000000000000000e+00))+(real_t)(1.0000000000000001e-01)))*(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[10]))*a[61]));
}

/* Fixed step size:0.05 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
rk_eta[4] = 1.0000000000000000e+00;
rk_eta[5] = 0.0000000000000000e+00;
rk_eta[6] = 0.0000000000000000e+00;
rk_eta[7] = 0.0000000000000000e+00;
rk_eta[8] = 0.0000000000000000e+00;
rk_eta[9] = 1.0000000000000000e+00;
rk_eta[10] = 0.0000000000000000e+00;
rk_eta[11] = 0.0000000000000000e+00;
rk_eta[12] = 0.0000000000000000e+00;
rk_eta[13] = 0.0000000000000000e+00;
rk_eta[14] = 1.0000000000000000e+00;
rk_eta[15] = 0.0000000000000000e+00;
rk_eta[16] = 0.0000000000000000e+00;
rk_eta[17] = 0.0000000000000000e+00;
rk_eta[18] = 0.0000000000000000e+00;
rk_eta[19] = 1.0000000000000000e+00;
rk_eta[20] = 0.0000000000000000e+00;
rk_eta[21] = 0.0000000000000000e+00;
rk_eta[22] = 0.0000000000000000e+00;
rk_eta[23] = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[24] = rk_eta[24];

for (run1 = 0; run1 < 1; ++run1)
{
acadoWorkspace.rk_xxx[0] = + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + rk_eta[23];
acado_rhs_ext( acadoWorkspace.rk_xxx, acadoWorkspace.rk_kkk );
acadoWorkspace.rk_xxx[0] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[0] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[1] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[2] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[3] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[4] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[5] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[6] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[7] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[8] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[9] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[10] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[11] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[12] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[13] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[14] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[15] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[16] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[17] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[18] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[19] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[20] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[21] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[22] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[23] + rk_eta[23];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 24 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[24] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[25] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[26] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[27] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[28] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[29] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[30] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[31] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[32] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[33] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[34] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[35] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[36] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[37] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[38] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[39] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[40] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[41] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[42] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[43] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[44] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[45] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[46] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)2.5000000000000001e-02*acadoWorkspace.rk_kkk[47] + rk_eta[23];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 48 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[48] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[49] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[50] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[51] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[52] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[53] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[54] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[55] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[56] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[57] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[58] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[59] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[60] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[61] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[62] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[63] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[64] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[65] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[66] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[67] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[68] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[69] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[70] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[71] + rk_eta[23];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 72 ]) );
rk_eta[0] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[0] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[24] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[48] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[72];
rk_eta[1] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[1] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[25] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[49] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[73];
rk_eta[2] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[2] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[26] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[50] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[74];
rk_eta[3] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[3] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[27] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[51] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[75];
rk_eta[4] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[4] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[28] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[52] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[76];
rk_eta[5] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[5] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[29] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[53] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[77];
rk_eta[6] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[6] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[30] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[54] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[78];
rk_eta[7] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[7] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[31] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[55] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[79];
rk_eta[8] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[8] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[32] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[56] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[80];
rk_eta[9] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[9] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[33] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[57] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[81];
rk_eta[10] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[10] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[34] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[58] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[82];
rk_eta[11] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[11] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[35] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[59] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[83];
rk_eta[12] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[12] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[36] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[60] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[84];
rk_eta[13] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[13] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[37] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[61] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[85];
rk_eta[14] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[14] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[38] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[62] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[86];
rk_eta[15] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[15] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[39] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[63] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[87];
rk_eta[16] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[16] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[40] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[64] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[88];
rk_eta[17] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[17] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[41] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[65] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[89];
rk_eta[18] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[18] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[42] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[66] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[90];
rk_eta[19] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[19] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[43] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[67] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[91];
rk_eta[20] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[20] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[44] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[68] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[92];
rk_eta[21] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[21] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[45] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[69] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[93];
rk_eta[22] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[22] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[46] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[70] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[94];
rk_eta[23] += + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[23] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[47] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[71] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[95];
acadoWorkspace.rk_ttt += 1.0000000000000000e+00;
}
error = 0;
return error;
}

