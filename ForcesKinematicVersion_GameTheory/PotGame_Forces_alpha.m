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
pstart = [16,20];
pstart3 = pstart;
pstart2 = [20,16];
pstart4 = pstart2;
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
index.u_acc_v1 = 1;
index.u_steer_v1 = 2;

% inputs vehicle 2
index.u_acc_v2 = 3;
index.u_steer_v2 = 4;
% shared
index.slack=5;

% states vehicle 1
index.x_v1 = 6;
index.y_v1 = 7;

% states vehicle 1
index.x_v2 = 8;
index.y_v2 = 9;

% Number of States
index.ns = 4;

% Number of Inputs
index.nu = 5;

% Number of Variables
index.nv = index.ns+index.nu;
index.sb = index.nu+1;

% Parameters
index.pslack = 1;
index.dist  = 2;
index.xend1 = 3;
index.xend2 = 4;
index.yend1 = 5;
index.yend2 = 6;
index.alpha = 7;
index.pacc  = 8;
index.psteer= 9;

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
	@(x,u,p)simplemodeldx(x,u), ...
	integrator_stepsize,...
	p);
model.E = [zeros(index.ns,index.nu), eye(index.ns)];

%% Non-Linear Constraints

% collision constraint
model.nh = 1; 
model.ineq = @(z,p) nlconst_simplePG(z,p);
model.hu = 0;
model.hl = -inf;

%% Number of parameters required
model.npar = 9;

%% Objective Function
for i=1:model.N
   model.objective{i} = @(z,p)objective_simplePG_alpha1(z,p(index.pslack),...
                        p(index.xend1),p(index.xend2),p(index.yend1),...
                        p(index.yend2),p(index.alpha),p(index.pacc),p(index.psteer));
   %model.intidx{i}    = [2,4];
end

%% Equality Constraints
model.xinitidx = index.sb:index.nv;

% initialization
model.ub = ones(1,index.nv)*inf;
model.lb = -ones(1,index.nv)*inf;

model.ub(index.u_acc_v1)=1;
model.lb(index.u_acc_v1)=0;

model.ub(index.u_acc_v2)=1;
model.lb(index.u_acc_v2)=0;

model.ub(index.u_steer_v1)=2;
model.lb(index.u_steer_v1)=-2;

model.ub(index.u_steer_v2)=2;
model.lb(index.u_steer_v2)=-2;

model.ub(index.x_v1)=30;
model.lb(index.x_v1)=0;

model.ub(index.y_v1)=21.5;
model.lb(index.y_v1)=18.5;

model.ub(index.x_v2)=21.5;
model.lb(index.x_v2)=18.5;

model.ub(index.y_v2)=30;
model.lb(index.y_v2)=0;

% Slack Variables Constraint (input)
model.lb(index.slack)=0;

%% CodeOptions for FORCES solver
codeoptions = getOptions('MPCPathFollowing_PgSimple1');
codeoptions.maxit = 200;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;

output = newOutput('alldata', 1:model.N, 1:model.nvar);

FORCES_NLP(model, codeoptions,output);
%% CodeOptions for FORCES solver
model2 = model;

codeoptions2 = getOptions('MPCPathFollowing_PgSimple2');
codeoptions2.maxit = 200;    % Maximum number of iterations
codeoptions2.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions2.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions2.cleanup = false;
codeoptions2.timing = 1;

output2 = newOutput('alldata', 1:model2.N, 1:model2.nvar);

FORCES_NLP(model2, codeoptions2,output2);

%% Initialization for simulation kart 1

xs(index.x_v1-index.nu)=pstart(1);
xs(index.y_v1-index.nu)=pstart(2);

xs(index.x_v2-index.nu)=pstart2(1);
xs(index.y_v2-index.nu)=pstart2(2);
x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';

%% Initialization for simulation kart 2

xs2(index.x_v1-index.nu)=pstart(1);
xs2(index.y_v1-index.nu)=pstart(2);

xs2(index.x_v2-index.nu)=pstart2(1);
xs2(index.y_v2-index.nu)=pstart2(2);
x02 = [zeros(model2.N,index.nu),repmat(xs2,model2.N,1)]';

%% Simulation
history = zeros(tend,model.nvar+1);
a=0;

history2 = zeros(tend,model2.nvar+1);
a2=0;
alpha=0.1:0.1:1;
optA = zeros(tend,length(alpha));
optB = zeros(tend,length(alpha));
opt  = zeros(tend,length(alpha));

problem.xinit = [xs(1:index.ns/2),xs2(index.ns/2+1:end)]';
problem2.xinit = [xs(1:index.ns/2),xs2(index.ns/2+1:end)]';
problem.x0 = x0(:);
problem2.x0 = x02(:); 

for i =1:tend
    for kk=1:length(alpha)
         
       
        % parameters
        problem.all_parameters = repmat(getParameters_simplePGalpha(pslack,...
            dist,xend1,yend1,xend2,yend2,alpha(kk),pacc,psteer), model.N ,1); 
        
        % parameters
        problem2.all_parameters = repmat(getParameters_simplePGalpha(pslack,...
            dist,xend1,yend1,xend2,yend2,alpha(kk),pacc,psteer), model.N ,1);
        
        % solve mpc
        [output,exitflag,info] = MPCPathFollowing_PgSimple1(problem);
        solvetimes(end+1)=info.solvetime;
        [output2,exitflag2,info2] = MPCPathFollowing_PgSimple2(problem2);
        solvetimes2(end+1)=info2.solvetime;
        if(exitflag==0)
           a = a + 1; 
        end
        if(exitflag2==0)
           a2 = a2 + 1; 
        end
        outputM = reshape(output.alldata,[model.nvar,model.N])';
        u = repmat(outputM(1,1:index.nu),eulersteps,1);
        %%
        x0 = outputM';
        [xhist,time] = euler(@(x,u)simplemodeldx(x,u),xs,u,integrator_stepsize/eulersteps);
        xs = xhist(end,:);
        % xs
        history((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=[time(1:end-1)+(tstart-1)*integrator_stepsize,u,xhist(1:end-1,:)];
        %outputCell{i,kk}=outputM;
        outputM2 = reshape(output2.alldata,[model2.nvar,model2.N])';
        x02 = outputM2';
        u2 = repmat(outputM2(1,1:index.nu),eulersteps,1);
        [xhist2,time2] = euler(@(x,u)simplemodeldx(x,u),xs2,u2,integrator_stepsize/eulersteps);
        xs2 = xhist2(end,:);
        % xs
        history2((tstart2-1)*eulersteps+1:(tstart2)*eulersteps,:)=[time2(1:end-1)+(tstart2-1)*integrator_stepsize,u2,xhist2(1:end-1,:)];
        cost1(i,kk)=info.pobj;
        cost2(i,kk)=info2.pobj;

        for jj=1:length(outputM)
                out=outputM(jj,:);
                [u1cost_v1,u2cost_v1,u1cost_v2,u2cost_v2,xcost1,xcost2,ycost1,...
                 ycost2,slackcost,f,f1,f2] = objective_PG_simpleTestAlpha(...
                                    out,pslack,xend1,xend2,yend1,yend2,alpha(kk),pacc,psteer);
                optA(i,kk)= optA(i,kk) + f1;
                optB(i,kk)= optB(i,kk) + f2;
                opt(i,kk) = opt(i,kk)  + f;
        end
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
            
            figure(5)
            hold on
            grid on
            title('Percentage Costs')
            xlabel('Alpha')
            ylabel('Costs')
        end
        
        figure(1)
        plot(outputM(:,index.x_v1),outputM(:,index.y_v1),'.-','Color',[0,alpha(kk)/max(alpha),1])
        plot(outputM(:,index.x_v2),outputM(:,index.y_v2),'.-','Color',[1,alpha(kk)/max(alpha),0])
        
        figure(2)
        plot(outputM(:,index.u_acc_v1),'.-','Color',[0,alpha(kk)/max(alpha),1])
        plot(outputM(:,index.u_acc_v2),'.-','Color',[1,alpha(kk)/max(alpha),0])
        
        figure(3)
        plot(outputM(:,index.u_steer_v1),'.-','Color',[0,alpha(kk)/max(alpha),1])
        plot(outputM(:,index.u_steer_v2),'.-','Color',[1,alpha(kk)/max(alpha),0])

        distance(:,kk)=(outputM(:,index.x_v1)-outputM(:,index.x_v2)).^2+...
             (outputM(:,index.y_v1)-outputM(:,index.y_v2)).^2;
    end
end
%%
figure(4)
for kk=1:length(alpha)
plot(alpha(kk),optA(kk)/max(opt),'*','Color',[0,alpha(kk)/max(alpha),1])
plot(alpha(kk),optB(kk)/max(opt),'*','Color',[1,alpha(kk)/max(alpha),0])
end

figure(5)
for kk=1:length(alpha)
plot(alpha(kk),optA(kk)/(optA(kk)+optB(kk))*100,'*','Color',[0,alpha(kk)/max(alpha),1])
plot(alpha(kk),optB(kk)/(optA(kk)+optB(kk))*100,'*','Color',[1,alpha(kk)/max(alpha),0])
end

load('IBR.mat')
figure(1)
plot(x_v1,y_v1,'.-','Color',[0,0,1])
plot(x_v2,y_v2,'.-','Color',[1,0,0])

figure(2)
plot(u1_v1,'.-','Color',[0,0,1])
plot(u1_v2,'.-','Color',[1,0,0])

figure(3)
plot(u2_v1,'.-','Color',[0,0,1])
plot(u2_v2,'.-','Color',[1,0,0])

figure(4)
plot(0,Cost_v1/max(opt),'*','Color',[0,0,1])
plot(0,Cost_v2/max(opt),'*','Color',[1,0,0])

figure (5)
plot(0,Cost_v1/(Cost_v1+Cost_v2)*100,'*','Color',[0,0,1])
plot(0,Cost_v2/(Cost_v1+Cost_v2)*100,'*','Color',[1,0,0])
