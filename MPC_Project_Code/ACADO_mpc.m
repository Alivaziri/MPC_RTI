clc;
clear all;
close all;
BEGIN_ACADO;
GEN_ACADO;

DifferentialState w0 theta dw0 dtheta; 
Control u; 

% define model
l = 0.5; g = 9.81; m = 0.1; M = 1;
expr1 = -m*cos(theta)^2 + M + m;
expr2 = m*l*sin(theta)*dtheta^2 + u;
expr3 = m*g*sin(theta);
% x' = A x + B u
f = [ dot(w0) ; dot(theta) ; dot(dw0) ; dot(dtheta) ] == ...  
    [ dw0; ...
      dtheta; ...
      (expr2 + expr3 * cos(theta))/expr1; ...
     -(cos(theta)*expr2 + expr3 + M*g*sin(theta))/(l*expr1) ];

% integrator
acadoSet('problemname', 'sim');

sim = acado.SIMexport( 0.05); % sampling time Ts = 0.05s
sim.setModel(f);
sim.set('INTEGRATOR_TYPE','INT_RK4' ); % integration method
sim.set('NUM_INTEGRATOR_STEPS', 1) % num of steps


sim.exportCode( 'export_SIM' ); % export code and compile
cd export_SIM
make_acado_integrator('C:\Users\a288v853_a\Desktop\ACADOtoolkit\MPC_Project\acado_system');
cd C:\Users\a288v853_a\Desktop\ACADOtoolkit\MPC_Project

%% MPCexport
acadoSet('problemname', 'mpc');

ocp = acado.OCP(0,1,20);
W = diag([10 10 0.1 0.1 0.01]);
WN = diag([10 10 0.1 0.1]);
ocp.minimizeLSQ(W , [w0; theta; dw0; dtheta; u]);
ocp.minimizeLSQEndTerm(WN , [w0; theta; dw0; dtheta]);
ocp.subjectTo( -2 <= w0 <= 2);
ocp.subjectTo( -20 <= u <= 20 );
ocp.setModel( f );

mpc = acado.OCPexport( ocp );
mpc.set( 'QP_SOLVER','QP_QPOASES');
mpc.set('INTEGRATOR_TYPE','INT_RK4' );
mpc.set( 'NUM_INTEGRATOR_STEPS',20); % steps over horizon

mpc.exportCode( 'export_MPC' );
copyfile('C:\Users\a288v853_a\Desktop\ACADOtoolkit\external_packages\qpoases', 'export_MPC\qpoases', 'f');
cd export_MPC
make_acado_solver('C:\Users\a288v853_a\Desktop\ACADOtoolkit\MPC_Project\acado_RTIstep');
cd C:\Users\a288v853_a\Desktop\ACADOtoolkit\MPC_Project

time = 0; Ts = 0.1; tf = 10;
% reference trajectories
input.y = repmat([0 pi 0 0 0], 20, 1);
input.yN = [0 pi 0 0];
% initialize
input.x = repmat([-2 pi/2 1 1], 21, 1);
input.u = 0.5*zeros(20,1);

state_sim = [-2 pi/2 1 1];
j = 1;
while time < tf
    input.x0 = state_sim; % current state values
    state(j,:) = state_sim;
    output = acado_RTIstep(input); % solve OCP
    % shifting process
    input.x = [output.x(2:end,:); output.x(end,:)];
    input.u = [output.u(2:end,:); output.u(end,:)];
    control(j) = input.u(2);
    input_sim.x = state_sim';
    input_sim.u = output.u(1,:).';
    output_sim = acado_system(input_sim); % simulate system
    state_sim = output_sim.value;
    [a,b] = size(state_sim);
    if  a > b
        state_sim = state_sim';
    end
    time = time + Ts;
    j = j + 1;
end

END_ACADO;

%% figure
figure
plot(0:0.1:10,state(1:end,2),'--','LineWidth',2);
hold on
yline(pi,'Color','r' );
hold off
ylabel('theta'); xlabel('time');
figure
plot(0:0.1:10,state(1:end,1),'--','MarkerSize',10,'Color','r');
ylabel('cart position'); xlabel('time');

figure
plot(0:0.1:10,state(1:end,3),'--','MarkerSize',10,'Color','r');
ylabel('cart velocity'); xlabel('time');

figure
plot(0:0.1:10,state(1:end,4),'--','MarkerSize',10,'Color','r');
ylabel('angular velocity'); xlabel('time');

figure
stairs(0:0.1:10,control(:));
ylabel('control'); xlabel('time');



