%% State and Input Definitions
global index_IBR

% inputs go kart 1
index_IBR.dotab = 1;
index_IBR.dotbeta = 2;
index_IBR.ds = 3;
index_IBR.slack = 4;
% shared
index_IBR.slack2=5;

% States
index_IBR.x = 6;
index_IBR.y = 7;
index_IBR.theta = 8;
index_IBR.v = 9;
index_IBR.ab = 10;
index_IBR.beta = 11;
index_IBR.s = 12;

% Number of States
index_IBR.ns = 7;

% Number of Inputs
index_IBR.nu = 5;

% Number of Variables
index_IBR.nv = index_IBR.ns+index_IBR.nu;
index_IBR.sb = index_IBR.nu+1;

% Parameters
index_IBR.ps = 1;
index_IBR.pax = 2;
index_IBR.pay = 3;
index_IBR.pll = 4;
index_IBR.prae = 5;
index_IBR.ptve = 6;
index_IBR.pbre = 7;

%% ADDED
index_IBR.plag = 8;
index_IBR.plat = 9;
index_IBR.pprog = 10;
index_IBR.pab = 11;
index_IBR.pdotbeta = 12;
index_IBR.pspeedcost = 13;
index_IBR.pslack = 14;
index_IBR.pslack2 = 15;
index_IBR.dist = 16;
index_IBR.xComp = 17;
index_IBR.yComp = 18;
index_IBR.pointsO = pointsO;
index_IBR.pointsN = pointsN;

%% model_IBR initialization
model.N = P_H_length;
model.nvar = index_IBR.nv;
model.neq = index_IBR.ns;
model.npar = pointsO + 3*pointsN;
