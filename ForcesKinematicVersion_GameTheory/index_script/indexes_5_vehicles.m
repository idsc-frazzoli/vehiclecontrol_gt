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

% inputs go kart 3
index.dotab_k3 = 9;
index.dotbeta_k3 = 10;
index.ds_k3 = 11;
index.slack_k3 = 12;

% inputs go kart 4
index.dotab_k4 = 13;
index.dotbeta_k4 = 14;
index.ds_k4 = 15;
index.slack_k4 = 16;

% inputs go kart 5
index.dotab_k5 = 17;
index.dotbeta_k5 = 18;
index.ds_k5 = 19;
index.slack_k5 = 20;

% shared
index.slack2 = 21;
index.slack3 = 22;
index.slack4 = 23;

% states
index.x = 24;
index.y = 25;
index.theta = 26;
index.v = 27;
index.ab = 28;
index.beta = 29;
index.s = 30;
% states kart 2
index.x_k2 = 31;
index.y_k2 = 32;
index.theta_k2 = 33;
index.v_k2 = 34;
index.ab_k2 = 35;
index.beta_k2 = 36;
index.s_k2 = 37;
% states kart 3
index.x_k3 = 38;
index.y_k3 = 39;
index.theta_k3 = 40;
index.v_k3 = 41;
index.ab_k3 = 42;
index.beta_k3 = 43;
index.s_k3 = 44;

% states kart 3
index.x_k4 = 45;
index.y_k4 = 46;
index.theta_k4 = 47;
index.v_k4 = 48;
index.ab_k4 = 49;
index.beta_k4 = 50;
index.s_k4 = 51;
% states kart 3
index.x_k5 = 52;
index.y_k5 = 53;
index.theta_k5 = 54;
index.v_k5 = 55;
index.ab_k5 = 56;
index.beta_k5 = 57;
index.s_k5 = 58;

% Number of States
index.ns = 35;

% Number of Inputs
index.nu = 23;

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
index.pointsN4=pointsN4;
index.pointsN5=pointsN5;
%% model initialization
model.N = P_H_length;
model.nvar = index.nv;
model.neq = index.ns;
model.npar = pointsO + 3*pointsN + 3*pointsN2 + 3*pointsN3 +...
             3*pointsN4 + 3*pointsN5;
