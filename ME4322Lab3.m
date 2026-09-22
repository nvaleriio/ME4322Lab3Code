clc
clear


%% Mass of each link
%converted to kg

M_cover = 0.59;
M_small_lever = 0.022;
M_Vlever = 0.11;
M_rack = 0.009;

%% Distance

L1 = 0.013;
L3 = 0.130;
L2 = 0.240;
L6 = 0.009;
L7 = 0.021;
R_pinion = 0.003;

%% MMoI

MMoI_Vlever = 1/3*M_Vlever*L2^2;
MMoI_small_lever = 1/3*M_small_lever*L3^2;
MMoI_pinion = 3.2*10^-8;
MMoI_L_piece = 8.5*10^-6;


%% Define Spring Stiffness (N/m)

k_rack_spring = 4.25;
k_plate_spring = 120; 
k_lever_spring = 321; 

%% Mx''+ Dx' Kx = 0 


%% Meq Calculations

Meq1 = M_cover/4*L1^2 + MMoI_Vlever/2;
Meq2 = M_cover/4*L1^2 + MMoI_small_lever;
Meq3 = Meq2/L3^2;
Meq4 = Meq3*(L2/2)^2;
Meq5 = (Meq4 + Meq1)/L2^2;
Meq6 = 2*Meq5;
Meq7 = (L6^2)*Meq6+MMoI_L_piece;
Meq8 = Meq7/L7^2 + M_rack;

Meq = R_pinion^2*Meq8 + MMoI_pinion;

%% Keq calculations

Keq1 = k_plate_spring/2*L1^2;
Keq2 = k_plate_spring/2*L1^2;
Keq3 = Keq2/L3^2;
Keq4 = Keq3*(L2/2)^2;
Keq5 = (Keq4 + Keq1)/L2^2;
Keq6 = 2*Keq5;
Keq7 = Keq6 + k_lever_spring;
Keq8 = Keq7*L6^2;
Keq9 = Keq8/(L7^2) + k_rack_spring;

Keq = Keq9*R_pinion^2; %at pinion

%% Deq calculations
% Assumptions
% Assuming coefficient of friction steel/steel =0.6
% Assuming friction force is greatest at initial contact
% Assuming frictional force is half of normal force
%

% Asuming that normal force in each link is 10% of the initial normal
% force divided by 4

% D1 = coef_friction * Force_normal * 4 * 1/pi
% assuming input frequency and amplitude of oscillation to be 1


Deq1 = 4*(1/pi)*0.6*845/4*L1^2; 
Deq2 = 4*(1/pi)*0.6*845/4*L1^2; 
Deq3 = Deq2/L3^2; 
Deq4 = Deq3*(L2/2)^2;  
Deq5 = (Deq4 + Deq1)/L2^2;
Deq6 = 2*Deq5; 
Deq7 = Deq6 + 4*(1/pi)*0.4*(45);
Deq8 = L6^2*Deq7 ; 
Deq9 = Deq8/L7^2 +4*(1/pi)*0.4*(19); 

Deq = Deq9*R_pinion^2;  %at pinion
%% Solving for Feq

Feq1 = 845/4*L1; 
Feq2 = 845/4*L1; 
Feq3 = Feq2/L3; 
Feq4 = Feq3*(L2/2); 
Feq5 = (Feq4 + Feq1)/L2; 
Feq6 = 2*Feq5; 
Feq7 = (L6)*Feq6; 
Feq8 = Feq7/L7; 

Feq = R_pinion*Feq8; % Lumping at pinion

%% Plot

% x'=diff(x,t)
syms x(t)

%first equation + switching the coefficients to rotational units since we are lumping to a rotational component
Jeq = Meq;
Beq = Deq;
K_Teq = Keq;
Teq = Feq;

eqn1 = Jeq*diff(x,t,2) + Beq*diff(x,t) + K_Teq*x == Teq;

%defining a variable
Dx = diff(x,t);

%defining our initial conditions
initialCondition = [x(0)==0,Dx(0)==0];


% Solve
sol = dsolve(eqn1,initialCondition);

figure;
fplot(sol, [0 10]);
title("Angular Displacement vs Time");
xlabel("Time (s)");
ylabel("Angular Displacement (rad)");


figure;
fplot(diff(sol,t), [0 10])
title("Angular Velocity vs Time");
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");