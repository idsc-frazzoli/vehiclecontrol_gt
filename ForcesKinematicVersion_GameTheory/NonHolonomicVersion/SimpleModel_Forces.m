%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Game Theory Potential Game
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% code by em
% 2 vehicle running in the same track in opponent direction, without any 
% constraints on collisions
%add force path (change that for yourself)
addpath('..');
userDir = getuserdir;
addpath('..\casadi');
addpath('..\models');  
addpath('..\draw_files');
addpath('..\parameters_vector');
addpath('..\objective_function');
addpath('..\constraints');
clear model
clear problem
clear all
close all

%% Parameters Definitions
pslack=50;
dist=2;
xend1=20;
yend1=0;
xend2=0;
yend2=20;

% Simulation length
tend = 1;
eulersteps=1;
solvetimes = [];
tstart = 1;
tstart2 = 1;
%% State and Input Definitions 
global index

% inputs vehicle 1
index.u_acc_v1 = 1;
index.u_steer_v1 = 2;

% states vehicle 1
index.x_v1 = 3;
index.y_v1 = 4;

% Number of States
index.ns = 2;

% Number of Inputs
index.nu = 2;

% Number of Variables
index.nv = index.ns+index.nu;
index.sb = index.nu+1;

% Parameters
index.pslack = 1;
index.dist   = 2;
index.xend1  = 3;
index.xend2  = 4;
index.yend1  = 5;
index.yend2  = 6;
index.x_v2   = 7;
index.y_v2   = 8;

% Stepsize
integrator_stepsize = 0.2;

%% model definition
model.N = 40;                       % Forward horizon Length
model.nvar = index.nv;
model.neq = index.ns;
model.eq = @(z,p) RK4(...
	z(index.sb:end), ...
	z(1:index.nu), ...
	@(x,u,p)simplemodel(x,u), ...
	integrator_stepsize,...
	p);
model.E = [zeros(index.ns,index.nu), eye(index.ns)];

%% Number of parameters required
model.npar = 6;

%% Objective Function
for i=1:model.N
   model.objective{i} = @(z,p)objective_simple(z,p(index.pslack),...
                        p(index.xend1),p(index.xend2),p(index.yend1),...
                        p(index.yend2));
end

%% Equality Constraints
model.xinitidx = index.sb:index.nv;

% initialization
model.ub = ones(1,index.nv)*inf;
model.lb = -ones(1,index.nv)*inf;

model.ub(index.u_acc_v1)=2;
model.lb(index.u_acc_v1)=0;

model.ub(index.u_steer_v1)=3;
model.lb(index.u_steer_v1)=-3;

model.ub(index.x_v1)=50;
model.lb(index.x_v1)=0;

model.ub(index.y_v1)=10;
model.lb(index.y_v1)=0;

%% CodeOptions for FORCES solver
codeoptions = getOptions('MPCPathFollowing_Simple');
codeoptions.maxit = 200;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;

output = newOutput('alldata', 1:model.N, 1:model.nvar);

FORCES_NLP(model, codeoptions,output);

%% Initialization for simulation kart 1

pstart = [0,5];
xs(index.x_v1-index.nu)=pstart(1);
xs(index.y_v1-index.nu)=pstart(2);

x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';

%% Simulation
history = zeros(tend,model.nvar+1);
a=0;

for i =1:tend
    
    problem.xinit = xs';
    % parameters
    problem.all_parameters = repmat(getParameters_simplePG(pslack,...
        dist,xend1,yend1,xend2,yend2), model.N ,1);
    problem.x0 = x0(:);       
    % solve mpc
    [output,exitflag,info] = MPCPathFollowing_Simple(problem);
    solvetimes(end+1)=info.solvetime;

    if(exitflag==0)
       a = a + 1; 
    end
    
    outputM = reshape(output.alldata,[model.nvar,model.N])';
    u = repmat(outputM(1,1:index.nu),eulersteps,1);
    %%
    x0 = outputM';
    [xhist,time] = euler(@(x,u)simplemodel(x,u),xs,u,integrator_stepsize/eulersteps);
    xs = xhist(end,:);
    % xs
    history((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=[time(1:end-1)+(tstart-1)*integrator_stepsize,u,xhist(1:end-1,:)];
    cost1(i)=info.pobj;
end
%%
figure
plot(outputM(:,index.x_v1),outputM(:,index.y_v1),'.-b')
hold on
plot(pstart(1),pstart(2),'r*')
plot(xend1,yend1,'r*')
figure
plot(outputM(:,index.u_acc_v1))
hold on
plot(outputM(:,index.u_steer_v1))
legend ('acc','steer')
