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
addpath('models');  
addpath('draw_files');
addpath('parameters_vector');
addpath('objective_function');
addpath('constraints');    
clear model
clear problem
clear all
close all

%% Parameters Definitions
maxSpeed = 10;
maxxacc = 4;
maxyacc = 8;
latacclim = 6;
rotacceffect  = 2;
torqueveceffect = 3;
brakeeffect = 0; 
plagerror=1;
platerror=0.01;
pprog=0.2;
pab=0.0004;
pdotbeta=0.05;
pspeedcost=0.2;
pslack=5;
pslack2=50;
dist=2;
alpha1=0.5;
alpha2=1;

% Splines
pointsO = 18;
pointsN = 10;
pointsN2 = 10;
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;

% Simulation length
tend = 90;
eulersteps = 10;
planintervall = 1;

%% Spline Points

points2 = [5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110;...          %x
          50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
points = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
          5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';
solvetimes = [];

solvetimes2 = [];
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
	@(x,u,p)interstagedx_PG(x,u), ...
	integrator_stepsize,...
	p);
model.E = [zeros(index.ns,index.nu), eye(index.ns)];

%% Non-Linear Constraints

%limit lateral acceleration
model.nh = 13; 
model.ineq = @(z,p) nlconst_PG(z,p);
model.hu = [0;1;0;0;0;0;0;1;0;0;0;0;0];%
model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf];%

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

output2 = newOutput('alldata', 1:model2.N, 1:model2.nvar);

FORCES_NLP(model2, codeoptions2,output2);

%% Initialization for simulation kart 1
fpoints = points(1:2,1:2);
pdir = diff(fpoints);
[pstartx,pstarty] = casadiDynamicBSPLINE(0.01,points);
pstart = [pstartx,pstarty];
pangle = atan2(pdir(2),pdir(1));
xs(index.x-index.nu)=pstart(1);
xs(index.y-index.nu)=pstart(2);
xs(index.theta-index.nu)=pangle;
xs(index.v-index.nu)=1;
xs(index.ab-index.nu)=0;
xs(index.beta-index.nu)=0;
xs(index.s-index.nu)=0.01;
plansx = [];
plansy = [];
planss = [];
targets = [];

%Go-kart 2 initialization
fpoints2 = points2(1:2,1:2);
pdir2 = diff(fpoints2);
[pstartx2,pstarty2] = casadiDynamicBSPLINE(0.01,points2);
pstart2 = [pstartx2,pstarty2];
pangle2 = atan2(pdir2(2),pdir2(1));
xs(index.x_k2-index.nu)=pstart2(1);
xs(index.y_k2-index.nu)=pstart2(2);
xs(index.theta_k2-index.nu)=pangle2;
xs(index.v_k2-index.nu)=1;
xs(index.ab_k2-index.nu)=0;
xs(index.beta_k2-index.nu)=0;
xs(index.s_k2-index.nu)=0.01;
plansx2 = [];
plansy2 = [];
planss2 = [];
targets2 = [];
planc = 10;
tstart = 1;
x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';

%% Initialization for simulation kart 2
fpoints3 = points(1:2,1:2);
pdir3 = diff(fpoints3);
[pstartx3,pstarty3] = casadiDynamicBSPLINE(0.01,points);
pstart3 = [pstartx3,pstarty3];
pangle3 = atan2(pdir3(2),pdir3(1));
xs2(index.x-index.nu)=pstart3(1);
xs2(index.y-index.nu)=pstart3(2);
xs2(index.theta-index.nu)=pangle3;
xs2(index.v-index.nu)=1;
xs2(index.ab-index.nu)=0;
xs2(index.beta-index.nu)=0;
xs2(index.s-index.nu)=0.01;
plansx3 = [];
plansy3 = [];
planss3 = [];
targets3 = [];

%Go-kart 2 initialization
fpoints4 = points2(1:2,1:2);
pdir4 = diff(fpoints4);
[pstartx4,pstarty4] = casadiDynamicBSPLINE(0.01,points2);
pstart4 = [pstartx4,pstarty4];
pangle4 = atan2(pdir4(2),pdir4(1));
xs2(index.x_k2-index.nu)=pstart4(1);
xs2(index.y_k2-index.nu)=pstart4(2);
xs2(index.theta_k2-index.nu)=pangle4;
xs2(index.v_k2-index.nu)=1;
xs2(index.ab_k2-index.nu)=0;
xs2(index.beta_k2-index.nu)=0;
xs2(index.s_k2-index.nu)=0.01;
plansx4 = [];
plansy4 = [];
planss4 = [];
targets4 = [];
planc2 = 10;
tstart2 = 1;
x02 = [zeros(model.N,index.nu),repmat(xs2,model.N,1)]';
%% Simulation
history = zeros(tend*eulersteps,model.nvar+1);
splinepointhist = zeros(tend,pointsN*3+pointsN2*3+1);
a=0;

history2 = zeros(tend*eulersteps,model.nvar+1);
splinepointhist2 = zeros(tend,pointsN*3+pointsN2*3+1);
a2=0;
optA = zeros(tend,1);
optB = zeros(tend,1);
opt = zeros(tend,1);
optA2 = zeros(tend,1);
optB2 = zeros(tend,1);
opt2 = zeros(tend,1);
cost1 = zeros(tend,1);
cost2 = zeros(tend,1);
Progress1 = zeros(tend,1);
Progress2 = zeros(tend,1);

for i =1:tend
    tstart = i;
    tstart2 = i;
    if(1)
        if xs(index.s-index.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart = splinestart+1;
            xs(index.s-index.nu)=xs(index.s-index.nu)-1;
        end
    end
    if(1)
        if xs(index.s_k2-index.nu)>1
            %nextSplinePoints_k2;
            %spline step forward
            splinestart2 = splinestart2+1;
            xs(index.s_k2-index.nu)=xs(index.s_k2-index.nu)-1;
        end
    end
    if(1)
        if xs2(index.s-index.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart3 = splinestart3+1;
            xs2(index.s-index.nu)=xs2(index.s-index.nu)-1;
        end
    end
    if(1)
        if xs2(index.s_k2-index.nu)>1
            %nextSplinePoints_k2;
            %spline step forward
            splinestart4 = splinestart4+1;
            xs2(index.s_k2-index.nu)=xs2(index.s_k2-index.nu)-1;
        end
    end
    % go kart 1
    xs(index.ab-index.nu)=min(casadiGetMaxAcc(xs(index.v-index.nu))-0.0001,xs(index.ab-index.nu));
    % go kart 2
    xs(index.ab_k2-index.nu)=min(casadiGetMaxAcc(xs(index.v_k2-index.nu))-0.0001,xs(index.ab_k2-index.nu));
    % go kart 1
    xs2(index.ab-index.nu)=min(casadiGetMaxAcc(xs2(index.v-index.nu))-0.0001,xs2(index.ab-index.nu));
    % go kart 2
    xs2(index.ab_k2-index.nu)=min(casadiGetMaxAcc(xs2(index.v_k2-index.nu))-0.0001,xs2(index.ab_k2-index.nu));
    
    problem.xinit = [xs(1:index.ns/2),xs2(index.ns/2+1:end)]';
    problem2.xinit = [xs(1:index.ns/2),xs2(index.ns/2+1:end)]';
    
    ip = splinestart;
    ip2 = splinestart2;
    [nkp, ~] = size(points);
    [nkp2, ~] = size(points2);
    nextSplinePoints = zeros(pointsN,3);
    nextSplinePoints_k2 = zeros(pointsN2,3);
    for jj=1:pointsN
       while ip>nkp
            ip = ip -nkp;
       end
       nextSplinePoints(jj,:)=points(ip,:);
       ip = ip + 1;
    end
    for jj=1:pointsN2
       while ip2>nkp2
            ip2 = ip2 -nkp2;
       end
       nextSplinePoints_k2(jj,:)=points2(ip2,:);
       ip2 = ip2 + 1;
    end
    splinepointhist(i,:)=[xs(index.s-index.nu),[nextSplinePoints(:);...
        nextSplinePoints_k2(:)]'];
    
    ip3 = splinestart3;
    ip4 = splinestart4;
    [nkp3, ~] = size(points);
    [nkp4, ~] = size(points2);
    nextSplinePoints3 = zeros(pointsN,3);
    nextSplinePoints4_k2 = zeros(pointsN2,3);
    for jj=1:pointsN
       while ip3>nkp3
            ip3 = ip3 -nkp3;
       end
       nextSplinePoints3(jj,:)=points(ip3,:);
       ip3 = ip3 + 1;
    end
    for jj=1:pointsN2
       while ip4>nkp4
            ip4 = ip4 -nkp4;
       end
       nextSplinePoints4_k2(jj,:)=points2(ip4,:);
       ip4 = ip4 + 1;
    end
    splinepointhist2(i,:)=[xs2(index.s-index.nu),[nextSplinePoints3(:);...
        nextSplinePoints4_k2(:)]'];
    
    % parameters
    problem.all_parameters = repmat(getParameters_PG_alpha(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,alpha1,alpha2,nextSplinePoints,...
        nextSplinePoints_k2), model.N ,1);
    problem.x0 = x0(:);   
    % parameters
    problem2.all_parameters = repmat(getParameters_PG_alpha(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,alpha1,alpha2,nextSplinePoints3,...
        nextSplinePoints4_k2), model.N ,1);
    problem2.x0 = x02(:);   
    % solve mpc
    [output,exitflag,info] = MPCPathFollowing_v1(problem);
    solvetimes(end+1)=info.solvetime;
    [output2,exitflag2,info2] = MPCPathFollowing_v2(problem2);
    solvetimes2(end+1)=info2.solvetime;
    if(exitflag==0)
       a = a + 1; 
    end
    if(exitflag~=1 && exitflag ~=0)
      %  draw2P_dec
      %  keyboard
    end
    if(exitflag2==0)
       a2 = a2 + 1; 
    end
    if(exitflag2~=1 && exitflag2 ~=0)
      %  draw2P_dec
      %  keyboard
    end
    %nextSplinePoints
    %get output
    outputM = reshape(output.alldata,[model.nvar,model.N])';
   
    %%
    x0 = outputM';
    u = repmat(outputM(1,1:index.nu),eulersteps,1);
    [xhist,time] = euler(@(x,u)interstagedx_PG(x,u),xs,u,integrator_stepsize/eulersteps);
    xs = xhist(end,:);
    % xs
    history((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=[time(1:end-1)+(tstart-1)*integrator_stepsize,u,xhist(1:end-1,:)];
    planc = planc + 1;
    if(planc>planintervall)
        planc = 1;
        plansx = [plansx; outputM(:,index.x)'];
        plansy = [plansy; outputM(:,index.y)'];
        planss = [planss; outputM(:,index.s)'];
        plansx2 = [plansx2; outputM(:,index.x_k2)'];
        plansy2 = [plansy2; outputM(:,index.y_k2)'];
        planss2 = [planss2; outputM(:,index.s_k2)'];
        [tx,ty]=casadiDynamicBSPLINE(outputM(end,index.s),nextSplinePoints);
        targets = [targets;tx,ty];
        [tx2,ty2]=casadiDynamicBSPLINE(outputM(end,index.s_k2),nextSplinePoints_k2);
        targets2 = [targets2;tx2,ty2];
    end
    outputM2 = reshape(output2.alldata,[model2.nvar,model2.N])';
    %% Evaluation cost function
    for jj=1:length(outputM)
        [lagcost_A,latcost_A,reg_A,prog_A,slack_A,speedcost_A,lagcost_A_k2,...
        latcost_A_k2,reg_A_k2,prog_A_k2,slack_A_k2,speedcost_A_k2,f,f1,f2] =...
        objective_PG_TestAlpha(outputM(jj,:),nextSplinePoints,nextSplinePoints_k2,...
        maxSpeed,plagerror, platerror, pprog, pab, pdotbeta,...
        pspeedcost,pslack,pslack2,alpha1,alpha2);
        [lagcost_B,latcost_B,reg_B,prog_B,slack_B,speedcost_B,lagcost_B_k2,...
        latcost_B_k2,reg_B_k2,prog_B_k2,slack_B_k2,speedcost_B_k2,f3,f4,f5] =...
        objective_PG_TestAlpha(outputM2(jj,:),nextSplinePoints3,nextSplinePoints4_k2,...
        maxSpeed,plagerror, platerror, pprog, pab, pdotbeta,...
        pspeedcost,pslack,pslack2,alpha1,alpha2);
        optA(i)= optA(i)+ f1;
        optB(i)= optB(i)+ f2;
        opt(i) = opt(i)+ f;
        optA2(i)= optA2(i)+ f4;
        optB2(i)= optB2(i)+ f5;
        opt2(i) = opt2(i)+ f3;
    end
    
    x02 = outputM2';
    u2 = repmat(outputM2(1,1:index.nu),eulersteps,1);
    [xhist2,time2] = euler(@(x,u)interstagedx_PG(x,u),xs2,u2,...
        integrator_stepsize/eulersteps);
    xs2 = xhist2(end,:);
    % xs
    history2((tstart2-1)*eulersteps+1:(tstart2)*eulersteps,:)=...
        [time2(1:end-1)+(tstart2-1)*integrator_stepsize,u2,xhist2(1:end-1,:)];
    planc2 = planc2 + 1;
    if(planc2>planintervall)
        planc2 = 1;
        plansx3 = [plansx3; outputM2(:,index.x)'];
        plansy3 = [plansy3; outputM2(:,index.y)'];
        planss3 = [planss3; outputM2(:,index.s)'];
        plansx4 = [plansx4; outputM2(:,index.x_k2)'];
        plansy4 = [plansy4; outputM2(:,index.y_k2)'];
        planss4 = [planss4; outputM2(:,index.s_k2)'];
        [tx3,ty3]=casadiDynamicBSPLINE(outputM2(end,index.s),...
            nextSplinePoints3);
        targets3 = [targets3;tx3,ty3];
        [tx4,ty4]=casadiDynamicBSPLINE(outputM2(end,index.s_k2),...
            nextSplinePoints4_k2);
        targets4 = [targets4;tx4,ty4];
    end        
    Percentage=i/tend*100
    cost1(i)=info.pobj;
    cost2(i)=info2.pobj;
    %costS(i)=costS;
    Progress1(i)=outputM(1,index.s);
    Progress2(i)=outputM2(1,index.s_k2);
    
end

draw2P_dec

figure
hold on
plot(cost1,'b')
plot(cost2,'r')
plot(cost1+cost2,'g')
legend('Kart1','Kart2','Tot')
figure
hold on
plot(Progress1,'b')
plot(Progress2,'r')
legend('Kart1','Kart2')

figure
plot(opt,'b')
hold on
plot(optA,'c')
plot(optA2,'r')
plot(optB,'y')
plot(optB2,'k')
plot(cost1,'g')
legend