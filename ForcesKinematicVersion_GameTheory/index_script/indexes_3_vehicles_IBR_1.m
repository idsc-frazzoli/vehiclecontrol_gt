%% State and Input Definitions
global index_IBR_1

% inputs go kart 1
index_IBR_1.dotab = 1;
index_IBR_1.dotbeta = 2;
index_IBR_1.ds = 3;
%index_IBR.slack = 4;
% shared
index_IBR_1.slack2=4;

% States
index_IBR_1.x = 5;
index_IBR_1.y = 6;
index_IBR_1.theta = 7;
index_IBR_1.v = 8;
index_IBR_1.ab = 9;
index_IBR_1.beta = 10;
index_IBR_1.s = 11;
index_IBR_1.laterror = 12;
% Number of States
index_IBR_1.ns = 8;

% Number of Inputs
index_IBR_1.nu = 4;

% Number of Variables
index_IBR_1.nv = index_IBR_1.ns+index_IBR_1.nu;
index_IBR_1.sb = index_IBR_1.nu+1;

% Parameters
index_IBR_1.ps = 1;
index_IBR_1.pax = 2;
index_IBR_1.pay = 3;
index_IBR_1.pll = 4;
index_IBR_1.prae = 5;
index_IBR_1.ptve = 6;
index_IBR_1.pbre = 7;

%% ADDED
index_IBR_1.plag = 8;
index_IBR_1.plat = 9;
index_IBR_1.pprog = 10;
index_IBR_1.pab = 11;
index_IBR_1.pdotbeta = 12;
index_IBR_1.pspeedcost = 13;
index_IBR_1.pslack = 14;
index_IBR_1.pslack2 = 15;
index_IBR_1.dist = 16;
index_IBR_1.xComp2 = 17;
index_IBR_1.yComp2 = 18;
index_IBR_1.xComp3 = 19;
index_IBR_1.yComp3 = 20;
index_IBR_1.pointsO = pointsO;
index_IBR_1.pointsN = pointsN;

%% model_IBR initialization
model2.N = P_H_length;
model2.nvar = index_IBR_1.nv;
model2.neq = index_IBR_1.ns;
model2.npar = pointsO + 3*pointsN;
