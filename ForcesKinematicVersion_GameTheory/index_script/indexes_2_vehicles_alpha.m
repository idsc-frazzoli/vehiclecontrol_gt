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

% shared
index.slack2=9;
% states
index.x = 10;
index.y = 11;
index.theta = 12;
index.v = 13;
index.ab = 14;
index.beta = 15;
index.s = 16;
% states kart 2
index.x_k2 = 17;
index.y_k2 = 18;
index.theta_k2 = 19;
index.v_k2 = 20;
index.ab_k2 = 21;
index.beta_k2 = 22;
index.s_k2 = 23;

% Number of States
index.ns = 14;

% Number of Inputs
index.nu = 9;

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
index.alpha1 = 17;
index.alpha2 = 18;
index.pointsO = pointsO;
index.pointsN = pointsN;
index.pointsN2 = pointsN2;

%% model initialization
model.N = P_H_length;
model.nvar = index.nv;
model.neq = index.ns;
model.npar = pointsO + 3*pointsN + 3*pointsN2;
