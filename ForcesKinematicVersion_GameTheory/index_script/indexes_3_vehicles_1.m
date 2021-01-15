%% State and Input Definitions 
global index1

% inputs go kart 1
index1.dotab = 1;
index1.dotbeta = 2;
index1.ds = 3;
%index.slack = 4;

% inputs go kart 2
index1.dotab_k2 = 4;
index1.dotbeta_k2 = 5;
index1.ds_k2 = 6;
%index.slack_k2 = 8;

% inputs go kart 2
index1.dotab_k3 = 7;
index1.dotbeta_k3 = 8;
index1.ds_k3 = 9;
%index.slack_k3 = 12;

% shared
index1.slack2 = 10;
index1.slack3 = 11;
index1.slack4 = 12;

% states
index1.x = 13;
index1.y = 14;
index1.theta = 15;
index1.v = 16;
index1.ab = 17;
index1.beta = 18;
index1.s = 19;
% states kart 2
index1.x_k2 = 20;
index1.y_k2 = 21;
index1.theta_k2 = 22;
index1.v_k2 = 23;
index1.ab_k2 = 24;
index1.beta_k2 = 25;
index1.s_k2 = 26;
% states kart 3
index1.x_k3 = 27;
index1.y_k3 = 28;
index1.theta_k3 = 29;
index1.v_k3 = 30;
index1.ab_k3 = 31;
index1.beta_k3 = 32;
index1.s_k3 = 33;
index1.laterror_k2=34;
% Number of States
index1.ns = 21;%22

% Number of Inputs
index1.nu = 12;

% Number of Variables
index1.nv = index1.ns+index1.nu;
index1.sb = index1.nu+1;

% Parameters
index1.ps = 1;
index1.pax = 2;
index1.pay = 3;
index1.pll = 4;
index1.prae = 5;
index1.ptve = 6;
index1.pbre = 7;

%% ADDED

index1.plag = 8;
index1.plat = 9;
index1.pprog = 10;
index1.pab = 11;
index1.pdotbeta = 12;
index1.pspeedcost = 13;
index1.pslack = 14;
index1.pslack2 = 15;
index1.dist = 16;

index1.pointsO=pointsO;
index1.pointsN=pointsN;
index1.pointsN2=pointsN2;
index1.pointsN3=pointsN3;

%% model initialization
model1.N = P_H_length;
model1.nvar = index1.nv;
model1.neq = index1.ns;
model1.npar = pointsO + 3*pointsN + 3*pointsN2 + 3*pointsN3;
