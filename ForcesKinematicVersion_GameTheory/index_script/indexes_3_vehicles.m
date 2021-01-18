%% State and Input Definitions 
global index

% inputs go kart 1
index.dotab = 1;
index.dotbeta = 2;
index.ds = 3;
%index.slack = 4;

% inputs go kart 2
index.dotab_k2 = 4;
index.dotbeta_k2 = 5;
index.ds_k2 = 6;
%index.slack_k2 = 8;

% inputs go kart 2
index.dotab_k3 = 7;
index.dotbeta_k3 = 8;
index.ds_k3 = 9;
%index.slack_k3 = 12;

% shared
index.slack2 = 10;
index.slack3 = 11;
index.slack4 = 12;

% states
index.x = 13;
index.y = 14;
index.theta = 15;
index.v = 16;
index.ab = 17;
index.beta = 18;
index.s = 19;
index.laterror=20;
index.slack_s=21;
% states kart 2
index.x_k2 = 22;
index.y_k2 = 23;
index.theta_k2 = 24;
index.v_k2 = 25;
index.ab_k2 = 26;
index.beta_k2 = 27;
index.s_k2 = 28;
index.laterror_k2=29;
index.slack_s_k2=30;
% states kart 3
index.x_k3 = 31;
index.y_k3 = 32;
index.theta_k3 = 33;
index.v_k3 = 34;
index.ab_k3 = 35;
index.beta_k3 = 36;
index.s_k3 = 37;
index.laterror_k3=38;
index.slack_s_k3=39;
% Number of States
index.ns = 27;

% Number of Inputs
index.nu = 12;

% Number of Variables
index.nv = index.ns+index.nu;
index.sb = index.nu+1;

% Parameters
index.ps = 1;
index.pSlackCost = 2;
index.pLaterrorCost = 3;
index.SpeedMax = 4;
index.pProgMax = 5;
index.pSpeedMax = 6;
index.ppos4 = 7;

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

% %% model initialization
% model1.N = P_H_length;
% model1.nvar = index.nv;
% model1.neq = index.ns;
% model1.npar = pointsO + 3*pointsN + 3*pointsN2 + 3*pointsN3;