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
pslack=70;
pacc=1;
psteer=1;
dist=1;
xend1=23;
yend1=20;
xend2=20;
yend2=23;
%alpha=0.001;

% Simulation length
tend = 1;
eulersteps=1;
solvetimes = [];

solvetimes2 = [];
tstart = 1;
tstart2 = 1;
%% State and Input Definitions 
global index

% inputs vehicle 1
index.u_acc_v1   = 1;
index.u_steer_v1 = 2;
index.slack      = 3;
% states vehicle 1
index.x_v1       = 4;
index.y_v1       = 5;

% Number of States
index.ns         = 2;

% Number of Inputs
index.nu         = 3;

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
index.pacc   = 7;
index.psteer = 8;
index.x_v2   = 9;
index.y_v2   = 10;

% Stepsize
integrator_stepsize = 0.2;
%trajectorytimestep = integrator_stepsize;

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

%% Non-Linear Constraints

% collision constraint
model.nh = 1; 
model.ineq = @(z,p) nlconst_simple(z,p);
model.hu = 0;
model.hl = -inf;

%% Number of parameters required
model.npar = 10;

%% Objective Function
for i=1:model.N
   model.objective{i} = @(z,p)objective_simpleIBR(z,p(index.pslack),...
                        p(index.xend1),p(index.yend1),...
                        p(index.pacc),...
                        p(index.psteer));
end

%% Equality Constraints
model.xinitidx = index.sb:index.nv;

% initialization
model.ub = ones(1,index.nv)*inf;
model.lb = -ones(1,index.nv)*inf;

model.ub(index.u_acc_v1)=1;
model.lb(index.u_acc_v1)=0;

model.ub(index.u_steer_v1)=2;
model.lb(index.u_steer_v1)=-2;

model.ub(index.x_v1)=30;
model.lb(index.x_v1)=0;

model.ub(index.y_v1)=21.5;
model.lb(index.y_v1)=18;

% Slack Variables Constraint (input)
model.lb(index.slack)=0;

%% CodeOptions for FORCES solver
codeoptions = getOptions('MPCPathFollowing_SimpleIBR');
codeoptions.maxit = 200;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;

output = newOutput('alldata', 1:model.N, 1:model.nvar);

FORCES_NLP(model, codeoptions,output);


%% CodeOptions for FORCES solver
model2=model;
model2.eq = @(z,p) RK4(...
	z(index.sb:end), ...
	z(1:index.nu), ...
	@(x,u,p)simplemodel2(x,u), ...
	integrator_stepsize,...
	p);
model2.ub(index.y_v1)=30;
model2.lb(index.y_v1)=0;

model2.ub(index.x_v1)=21.5;
model2.lb(index.x_v1)=18;
for i=1:model2.N
   model2.objective{i} = @(z,p)objective_simpleIBR(z,p(index.pslack),...
                        p(index.xend2),p(index.yend2),...
                        p(index.pacc),...
                        p(index.psteer));
end
codeoptions = getOptions('MPCPathFollowing_SimpleIBR2');
codeoptions.maxit = 200;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;

output = newOutput('alldata', 1:model.N, 1:model.nvar);

FORCES_NLP(model2, codeoptions,output);
%% Initialization for simulation kart 1

pstart = [16,20];
xs(index.x_v1-index.nu)=pstart(1);
xs(index.y_v1-index.nu)=pstart(2);

pstart2 = [20,16];
xs2(index.x_v1-index.nu)=pstart2(1);
xs2(index.y_v1-index.nu)=pstart2(2);

x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';
x02 = [zeros(model2.N,index.nu),repmat(xs2,model2.N,1)]';
%% Simulation
history = zeros(tend,model.nvar+1);
a=0;

optA = zeros(tend,1);
optB = zeros(tend,1);
opt  = zeros(tend,1);

Pos1=repmat(pstart, model.N-1 ,1);
Pos2=repmat(pstart2, model.N-1 ,1);

for i =1:tend
    problem.xinit = xs(1:index.ns)';
    problem.x0 = x0(:);        
    problem2.xinit = xs2(1:index.ns)';
    problem2.x0 = x02(:);    
    % parameters
    problem.all_parameters = repmat(getParameters_simpleIBR(pslack,...
        dist,xend1,yend1,xend2,yend2,pacc,psteer,xs2(1),xs2(2)),model.N,1); 

    problem.all_parameters(index.x_v2:model.npar:end)=[Pos2(:,1);...
                                                    Pos2(end,1)];
    problem.all_parameters(index.y_v2:model.npar:end)=[Pos2(:,2);...
                                                    Pos2(end,2)];
    % parameters
    problem2.all_parameters = repmat(getParameters_simpleIBR(pslack,...
        dist,xend1,yend1,xend2,yend2,pacc,psteer,xs(1),xs(2)), model.N ,1);

    problem2.all_parameters(index.x_v2:model.npar:end)=[Pos1(:,1);...
                                                     Pos1(end,1)];
    problem2.all_parameters(index.y_v2:model.npar:end)=[Pos1(:,2);...
                                                     Pos1(end,2)];
    % solve mpc
    [output,exitflag,info] = MPCPathFollowing_SimpleIBR(problem);
    solvetimes(end+1)=info.solvetime;

    if(exitflag==0)
       a = a + 1; 
    end

    outputM = reshape(output.alldata,[model.nvar,model.N])';
    u = repmat(outputM(1,1:index.nu),eulersteps,1);
    %
    x0 = outputM';
    [xhist,time] = euler(@(x,u)simplemodel(x,u),xs,u,...
        integrator_stepsize/eulersteps);
    xs = xhist(end,:);
    % xs
    history((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
        [time(1:end-1)+(tstart-1)*integrator_stepsize,u,xhist(1:end-1,:)];
    %outputCell{i,kk}=outputM;
    problem2.all_parameters(index.x_v2:model.npar:end)=...
        outputM(:,index.x_v1);
    problem2.all_parameters(index.y_v2:model.npar:end)=...
        outputM(:,index.y_v1);
    [output2,exitflag2,info2] = MPCPathFollowing_SimpleIBR2(problem2);
    solvetimes2(end+1)=info2.solvetime;
    if(exitflag2==0)
       a2 = a2 + 1; 
    end
    outputM2 = reshape(output2.alldata,[model.nvar,model.N])';
    x02 = outputM2';
    u2 = repmat(outputM2(1,1:index.nu),eulersteps,1);
    [xhist2,time2] = euler(@(x,u)simplemodel2(x,u),xs2,u2,...
        integrator_stepsize/eulersteps);
    xs2 = xhist2(end,:);
    % xs
    history2((tstart2-1)*eulersteps+1:(tstart2)*eulersteps,:)=...
        [time2(1:end-1)+(tstart2-1)*integrator_stepsize,u2,...
        xhist2(1:end-1,:)];
    cost1(i)=info.pobj;
    cost2(i)=info2.pobj;

    Pos1=[outputM(2:end,index.x_v1),outputM(2:end,index.y_v1)];

    %outputM2 = reshape(output2.alldata,[model.nvar,model.N])';
    Pos2=[outputM2(2:end,index.x_v1),outputM2(2:end,index.y_v1)];
    if(1)
        figure(1)
        plot(pstart(1),pstart(2),'g*')
        hold on
        plot(pstart2(1),pstart2(2),'g*') 
        plot(xend1,yend1,'r*')
        plot(xend2,yend2,'r*')
        grid on
        title('trajectory')
        xlabel('X')
        ylabel('Y')

        figure(2)
        hold on
        grid on
        title('acc')
        xlabel('step')
        ylabel('')

        figure(3)
        hold on
        grid on
        title('steer')
        xlabel('step')
        ylabel('')

        figure(4)
        hold on
        grid on
        title('Costs')
        xlabel('Alpha')
        ylabel('Costs')
    end

    figure(1)
    plot(outputM(:,index.x_v1),outputM(:,index.y_v1),'.-','Color',[0,0,1])
    plot(outputM2(:,index.x_v1),outputM2(:,index.y_v1),'.-','Color',[1,0,0])

    figure(2)
    plot(outputM(:,index.u_acc_v1),'.-','Color',[0,0,1])
    plot(outputM2(:,index.u_acc_v1),'.-','Color',[1,0,0])

    figure(3)
    plot(outputM(:,index.u_steer_v1),'.-','Color',[0,0,1])
    plot(outputM2(:,index.u_steer_v1),'.-','Color',[1,0,0])

end

%%
figure(4)

plot(0,cost1,'*','Color',[0,0,1])
plot(0,cost2,'*','Color',[1,0,0])

%%
x_v1=outputM(:,index.x_v1);
y_v1=outputM(:,index.y_v1);

x_v2=outputM2(:,index.x_v1);
y_v2=outputM2(:,index.y_v1);

u1_v1=outputM(:,index.u_acc_v1);
u1_v2=outputM2(:,index.u_acc_v1);

u2_v1=outputM(:,index.u_steer_v1);
u2_v2=outputM2(:,index.u_steer_v1);

Cost_v1=cost1(1);
Cost_v2=cost2(1);

save('IBR','x_v1','y_v1','x_v2','y_v2','u1_v1','u2_v1','u1_v2',...
     'u2_v2','Cost_v1','Cost_v2')