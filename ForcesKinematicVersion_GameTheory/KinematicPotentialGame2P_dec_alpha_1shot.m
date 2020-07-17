%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Game Theory Potential Game
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% code by em
% 2 vehicle running in the same track in opponent direction, without any 
% constraints on collisions
%add force path (change that for yourself)
addpath('..');
userDir = getuserdir;
addpath('casadi');
    
clear model
clear problem
clear all
close all

%% Parameters Definitions
% Splines
pointsO = 18;
pointsN = 10;
pointsN2 = 10;
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

% Stepsize
integrator_stepsize = 0.1;
trajectorytimestep = integrator_stepsize;

%% model definition
model.N = 31;                       % Forward horizon Length
model.nvar = index.nv;
model.neq = index.ns;
model.eq = @(z,p) RK4(...
	z(index.sb:end), ...
	z(1:index.nu), ...
	@(x,u,p)interstagedx_alpha(x,u), ...
	integrator_stepsize,...
	p);
model.E = [zeros(index.ns,index.nu), eye(index.ns)];

%% Non-Linear Constraints

%limit lateral acceleration
model.nh = 7; 
model.ineq = @(z,p) nlconst_alpha(z,p);
model.hu = [0;0;0;0;0;0;0];%
model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;-inf];

%% Number of parameters required
model.npar = pointsO + 3*pointsN + 3*pointsN2;

%% Objective Function
for i=1:model.N
   model.objective{i} = @(z,p)objective_PG_2(...
       z,...
       getPointsFromParameters(p, pointsO, pointsN),...
       getRadiiFromParameters(p, pointsO, pointsN),...
       getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
       getRadiiFromParameters(p, pointsO + 3*pointsN, pointsN2),...
       p(index.ps),...
       p(index.pax),...
       p(index.pay),...
       p(index.pll),...
       p(index.prae),...
       p(index.ptve),...
       p(index.pbre),...
       p(index.plag),...
       p(index.plat),...
       p(index.pprog),...
       p(index.pab),...
       p(index.pdotbeta),...
       p(index.pspeedcost),...
       p(index.pslack),...
       p(index.pslack2),...
       p(index.alpha1),...
       p(index.alpha2));
end

%% Equality Constraints
model.xinitidx = index.sb:index.nv;

% initialization
model.ub = ones(1,index.nv)*inf;
model.lb = -ones(1,index.nv)*inf;

% Path Progress rate Constraint (input)
model.ub(index.ds)=5;
model.lb(index.ds)=-1;

% Acceleration Constraint (input)
model.lb(index.ab)=-inf;

% Slack Variables Constraint (input)
model.lb(index.slack)=0;

% Speed Constraint (state)
model.lb(index.v)=0;

% Steering Angle Constraint (input)
model.ub(index.beta)=0.5;
model.lb(index.beta)=-0.5;

% Path Progress Constraint (input)
model.ub(index.s)=pointsN-2;
model.lb(index.s)=0;

% Path Progress rate Constraint (input)
model.ub(index.ds_k2)=5;
model.lb(index.ds_k2)=-1;

% Acceleration Constraint (input)
model.lb(index.ab_k2)=-inf;

% Slack Variables Constraint (input)
model.lb(index.slack_k2)=0;

% Speed Constraint (state)
model.lb(index.v_k2)=0;

% Steering Angle Constraint (input)
model.ub(index.beta_k2)=0.5;
model.lb(index.beta_k2)=-0.5;

% Path Progress Constraint (input)
model.ub(index.s_k2)=pointsN2-2;
model.lb(index.s_k2)=0;

model.lb(index.slack2)=0;

%% CodeOptions for FORCES solver
codeoptions = getOptions('MPCPathFollowing_v1');
codeoptions.maxit = 500;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;
codeoptions.nlp.TolIneq = 1E-8; % tol. on inequality constraints
output = newOutput('alldata', 1:model.N, 1:model.nvar);

FORCES_NLP(model, codeoptions,output);
%% CodeOptions for FORCES solver
model2 = model;
for i=1:model2.N
   model2.objective{i} = @(z,p)objective_PG_2(...
       z,...
       getPointsFromParameters(p, pointsO, pointsN),...
       getRadiiFromParameters(p, pointsO, pointsN),...
       getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
       getRadiiFromParameters(p, pointsO + 3*pointsN, pointsN2),...
       p(index.ps),...
       p(index.pax),...
       p(index.pay),...
       p(index.pll),...
       p(index.prae),...
       p(index.ptve),...
       p(index.pbre),...
       p(index.plag),...
       p(index.plat),...
       p(index.pprog),...
       p(index.pab),...
       p(index.pdotbeta),...
       p(index.pspeedcost),...
       p(index.pslack),...
       p(index.pslack2),...
       p(index.alpha1),...
       p(index.alpha2));
end
codeoptions2 = getOptions('MPCPathFollowing_v2');
codeoptions2.maxit = 500;    % Maximum number of iterations
codeoptions2.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions2.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions2.cleanup = false;
codeoptions2.timing = 1;
codeoptions2.nlp.TolIneq = 1E-8; % tol. on inequality constraints
output2 = newOutput('alldata', 1:model2.N, 1:model2.nvar);

FORCES_NLP(model2, codeoptions2,output2);
%% State and Input Definitions IBR
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

%% model_IBR Definition

model_IBR.N = 31;
model_IBR.nvar = index_IBR.nv;
model_IBR.neq = index_IBR.ns;
model_IBR.eq = @(z,p) RK4(...
    z(index_IBR.sb:end),...
    z(1:index_IBR.nu),...
    @(x,u,p)interstagedx_IBR_1s(x,u),...
    integrator_stepsize,...
    p);
model_IBR.E = [zeros(index_IBR.ns,index_IBR.nu), eye(index_IBR.ns)];

%% Non-Linear Constraints

%limit lateral acceleration
model_IBR.nh = 4; 
model_IBR.ineq = @(z,p) nlconst_IBR_1s(z,p);
model_IBR.hu = [0;0;0;0];
model_IBR.hl = [-inf;-inf;-inf;-inf];

%% Number of parameters required
model_IBR.npar = pointsO + 3*pointsN;

%% Objective Function
model_IBR.npar = pointsO + 3*pointsN;
for i=1:model_IBR.N
    model_IBR.objective{i} = @(z,p)objective_IBR_1s(z,...
    getPointsFromParameters(p, pointsO, pointsN),...
    p(index_IBR.ps),...
    p(index_IBR.plag),...
    p(index_IBR.plat),...
    p(index_IBR.pprog),...
    p(index_IBR.pab),...
    p(index_IBR.pdotbeta),...
    p(index_IBR.pspeedcost),...
    p(index_IBR.pslack),...
    p(index_IBR.pslack2));
end

%% Equality Constraints
model_IBR.xinitidx = index_IBR.sb:index_IBR.nv;

% initialization
model_IBR.ub = ones(1,index_IBR.nv)*inf;
model_IBR.lb = -ones(1,index_IBR.nv)*inf;

% Path Progress rate Constraint (input)
model_IBR.ub(index_IBR.ds)=5;
model_IBR.lb(index_IBR.ds)=-1;

% Acceleration Constraint (input)
model_IBR.lb(index_IBR.ab)=-inf;

% Slack Variables Constraint (input)
model_IBR.lb(index_IBR.slack)=0;

% Speed Constraint (state)
model_IBR.lb(index_IBR.v)=0;

% Steering Angle Constraint (input)
model_IBR.ub(index_IBR.beta)=0.5;
model_IBR.lb(index_IBR.beta)=-0.5;

% Path Progress Constraint (input)
model_IBR.ub(index_IBR.s)=pointsN-2;
model_IBR.lb(index_IBR.s)=0;

% Slack
model_IBR.lb(index_IBR.slack2)=0;

%% CodeOptions for FORCES solver
codeoptions = getOptions('MPCPathFollowing_IBR');
codeoptions.maxit = 200;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;
codeoptions.nlp.TolIneq = 1E-8; % tol. on inequality constraints
output = newOutput('alldata', 1:model_IBR.N, 1:model_IBR.nvar);

FORCES_NLP(model_IBR, codeoptions,output);


RunAlpha_1shot