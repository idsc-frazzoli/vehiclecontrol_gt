%% State and Input Definitions 
global index

% inputs go kart 1
index.dotab = 1;
index.dotbeta = 2;
index.ds = 3;
index.slack = 4;

% inputs go kart 2
index.dotab_k2 = 5;
index.dotbeta_k2 = 6;
index.ds_k2 = 7;
index.slack_k2 = 8;

% inputs go kart 2
index.dotab_k3 = 9;
index.dotbeta_k3 = 10;
index.ds_k3 = 11;
index.slack_k3 = 12;

% shared
index.slack2 = 13;
index.slack3 = 14;
index.slack4 = 15;

% states
index.x = 16;
index.y = 17;
index.theta = 18;
index.v = 19;
index.ab = 20;
index.beta = 21;
index.s = 22;
% states kart 2
index.x_k2 = 23;
index.y_k2 = 24;
index.theta_k2 = 25;
index.v_k2 = 26;
index.ab_k2 = 27;
index.beta_k2 = 28;
index.s_k2 = 29;
% states kart 3
index.x_k3 = 30;
index.y_k3 = 31;
index.theta_k3 = 32;
index.v_k3 = 33;
index.ab_k3 = 34;
index.beta_k3 = 35;
index.s_k3 = 36;
% Number of States
index.ns = 21;

% Number of Inputs
index.nu = 15;

% Number of Variables
index.nv = index.ns+index.nu;
index.sb = index.nu+1;

% Parameters
index.ps = 1;
index.pax = 2;
index.pay = 3;
index.pll = 4;
index.prae = 5;
index.ptve = 6;
index.pbre = 7;

%% ADDED

index.plag = 8;
index.plat = 9;
index.pprog = 10;
index.pab = 11;
index.pdotbeta = 12;
index.pspeedcost = 13;
index.pslack = 14;
index.pslack2 = 15;
index.dist = 16;

index.pointsO=pointsO;
index.pointsN=pointsN;
index.pointsN2=pointsN2;
index.pointsN3=pointsN3;

%% model initialization
model.N = P_H_length;
model.nvar = index.nv;
model.neq = index.ns;
model.npar = pointsO + 3*pointsN + 3*pointsN2 + 3*pointsN3;
