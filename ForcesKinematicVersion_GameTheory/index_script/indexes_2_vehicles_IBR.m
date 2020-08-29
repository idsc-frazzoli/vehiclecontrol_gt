%% State and Input Definitions
global index

% inputs go kart 1
index.dotab = 1;
index.dotbeta = 2;
index.ds = 3;
index.slack = 4;
% shared
index.slack2=5;

% States
index.x = 6;
index.y = 7;
index.theta = 8;
index.v = 9;
index.ab = 10;
index.beta = 11;
index.s = 12;

% Number of States
index.ns = 7;

% Number of Inputs
index.nu = 5;

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
index.xComp = 17;
index.yComp = 18;
index.pointsO = pointsO;
index.pointsN = pointsN;

%% model initialization
model.N = P_H_length;
model.nvar = index.nv;
model.neq = index.ns;
model.npar = pointsO + 3*pointsN;
