/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    DifferentialState w0;
    DifferentialState theta;
    DifferentialState dw0;
    DifferentialState dtheta;
    Control u;
    DMatrix acadodata_M1;
    acadodata_M1.read( "mpc_data_acadodata_M1.txt" );
    Function acadodata_f2;
    acadodata_f2 << w0;
    acadodata_f2 << theta;
    acadodata_f2 << dw0;
    acadodata_f2 << dtheta;
    acadodata_f2 << u;
    DMatrix acadodata_M2;
    acadodata_M2.read( "mpc_data_acadodata_M2.txt" );
    Function acadodata_f3;
    acadodata_f3 << w0;
    acadodata_f3 << theta;
    acadodata_f3 << dw0;
    acadodata_f3 << dtheta;
    OCP ocp1(0, 1, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo((-2.00000000000000000000e+00) <= w0 <= 2.00000000000000000000e+00);
    ocp1.subjectTo((-2.00000000000000000000e+01) <= u <= 2.00000000000000000000e+01);
    DifferentialEquation acadodata_f4;
    acadodata_f4 << dot(w0) == dw0;
    acadodata_f4 << dot(theta) == dtheta;
    acadodata_f4 << dot(dw0) == 1/((-1.00000000000000005551e-01)*pow(cos(theta),2.00000000000000000000e+00)+1.00000000000000000000e+00+1.00000000000000005551e-01)*(5.00000000000000027756e-02*pow(dtheta,2.00000000000000000000e+00)*sin(theta)+9.81000000000000094147e-01*cos(theta)*sin(theta)+u);
    acadodata_f4 << dot(dtheta) == 1/((-1.00000000000000005551e-01)*pow(cos(theta),2.00000000000000000000e+00)+1.00000000000000000000e+00+1.00000000000000005551e-01)*(-(5.00000000000000027756e-02*pow(dtheta,2.00000000000000000000e+00)*sin(theta)+u)*cos(theta)-9.81000000000000049738e+00*sin(theta)-9.81000000000000094147e-01*sin(theta))/5.00000000000000000000e-01;

    ocp1.setModel( acadodata_f4 );


    ocp1.setNU( 1 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 0 );
    OCPexport ExportModule2( ocp1 );
    ExportModule2.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule2.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule2.set( INTEGRATOR_TYPE, INT_RK4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule2.set( NUM_INTEGRATOR_STEPS, 20 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    uint export_flag;
    export_flag = ExportModule2.exportCode( "export_MPC" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

