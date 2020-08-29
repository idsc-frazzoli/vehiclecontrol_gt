%add force path (change that for yourself)
addpath('..');
userDir = getuserdir;
addpath('casadi');
addpath('models');  
addpath('draw_files');
addpath('parameters_vector');
addpath('objective_function');
addpath('constraints');
addpath('index_script');

clear model
clear problem
clear all
close all

%% Parameters Definitions
parameters_1_vehicle

%% State and Input Definitions 
global index
indexes_1_vehicle

%% Model Definition (models folder)

% if you change interstagedx file, remember to change also in
% simulation!!!!
model.eq = @(z,p) RK4( z(index.sb:end), z(1:index.nu),...
             @(x,u,p)interstagedx(x,u), integrator_stepsize,p);

model.E = [zeros(index.ns,index.nu), eye(index.ns)];

%% Objective Function (objective_function folder)
for i=1:model.N
    model.objective{i} = @(z,p)objective(z,...
    getPointsFromParameters(p, pointsO, pointsN),...
    getRadiiFromParameters(p, pointsO, pointsN),...
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
    p(index.pslack));
end

%% Linear Constraints
model.xinitidx = index.sb:index.nv;

% initialization
model.ub = ones(1,index.nv)*inf;
model.lb = -ones(1,index.nv)*inf;

% Delta path progress
model.ub(index.ds)=ds_max;
model.lb(index.ds)=ds_min;

% Acceleration
model.lb(index.ab)=-inf;

% Slack
model.lb(index.slack)=0;

% Velocity
model.lb(index.v)=0;

% Steering Angle
model.ub(index.beta)=beta_max;
model.lb(index.beta)=beta_min;

%Path Progress
model.ub(index.s)=pointsN-2;
model.lb(index.s)=0;

%% Non-Linear Constraints (constraints folder)

%limit lateral acceleration
model.nh = NUM_const; 
model.ineq = @(z,p) nlconst(z,p);
model.hu = [0;1;0;0;0;0];
model.hl = [-inf;-inf;-inf;-inf;-inf;-inf];

%% Solver
codeoptions = getOptions('MPCPathFollowing_1v');
codeoptions.maxit = MAX_IT;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;

output = newOutput('alldata', 1:model.N, 1:model.nvar);

FORCES_NLP(model, codeoptions,output);