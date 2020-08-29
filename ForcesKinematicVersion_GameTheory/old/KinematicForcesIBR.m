%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kynematic MPC Script IBR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% code by em
% 2 vehicle running in the an intersection, with 
% constraints on collisions. The iterations run until a Nash equilibrium is
% reached (Iterated Best response)

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
%close all

%% Parameters Definitions
maxSpeed = 10;
maxxacc = 4;
maxyacc = 8;
latacclim = 6;
rotacceffect  = 2;
torqueveceffect = 3;
brakeeffect = 0; 
% objective function costs
plagerror=1;
platerror=0.01;
pprog=0.2;
pab=0.0004;
pdotbeta=0.01;
pspeedcost=0.2;
pslack=5;
pslack2=10;
dist=2;

% Splines
pointsO = 18;
pointsN = 10;
pointsN2 = 10;
splinestart = 1;
splinestart2 = 1;
%nextSplinePoints = 0;
%nextsplinepoints2 = 0;

% Simulation length
tend = 90;
Nit=1;
eulersteps = 10;
planintervall = 1;

%% Spline Points
%     
% points2 = [5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110;...          %x
%           50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
%           3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
% points = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
%           5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110; ...    %y
%           3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';

points = [10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95;...          %x
          50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
          10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
      

% % points = [41.8,36.2,52,57.2,53,52,47;...          %x
%           38.33,44.933,58.2,53.8,49,44,43; ...    %y
%           2.5,2.5,2.5,2.5,2.5,2.5,2.5]';         %width 
% points2 = [57.2,52,36.2,41.8,47,52,53;...          %x
%           53.8,58.2,44.933,38.33,43,44,49; ...    %y
%           2.5,2.5,2.5,2.5,2.5,2.5,2.5]';       

solvetimes = [];
solvetimes2=[];
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

integrator_stepsize = 0.1;
trajectorytimestep = integrator_stepsize;

%% Model Definition

model.N = 31;
model.nvar = index.nv;
model.neq = index.ns;
model.eq = @(z,p) RK4(...
    z(index.sb:end),...
    z(1:index.nu),...
    @(x,u,p)interstagedx(x,u),...
    integrator_stepsize,...
    p);
model.E = [zeros(index.ns,index.nu), eye(index.ns)];

%% Non-Linear Constraints

%limit lateral acceleration
model.nh = 7; 
model.ineq = @(z,p) nlconst_IBR(z,p);
model.hu = [0;1;0;0;0;0;0];
model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;-inf];

%% Number of parameters required
model.npar = pointsO + 3*pointsN;

%% Objective Function
model.npar = pointsO + 3*pointsN;
for i=1:model.N
    model.objective{i} = @(z,p)objective_IBR(z,...
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
    p(index.pslack),...
    p(index.pslack2));
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

% Slack
model.lb(index.slack2)=0;

%% CodeOptions for FORCES solver
codeoptions = getOptions('MPCPathFollowing_IBR1');
codeoptions.maxit = 200;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;

output = newOutput('alldata', 1:model.N, 1:model.nvar);

FORCES_NLP(model, codeoptions,output);

%% Initialization for simulation
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
planc = 10;

x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';

%% Initialization for simulation 2
fpoints2 = points2(1:2,1:2);
pdir2 = diff(fpoints2);
[pstartx2,pstarty2] = casadiDynamicBSPLINE(0.01,points2);
pstart2 = [pstartx2,pstarty2];
pangle2 = atan2(pdir2(2),pdir2(1));
xs2(index.x-index.nu)=pstart2(1);
xs2(index.y-index.nu)=pstart2(2);
xs2(index.theta-index.nu)=pangle2;
xs2(index.v-index.nu)=1;
xs2(index.ab-index.nu)=0;
xs2(index.beta-index.nu)=0;
xs2(index.s-index.nu)=0.01;
%% Simulation
history = zeros(tend*eulersteps,model.nvar+1);
splinepointhist = zeros(tend,pointsN*3+1);
plansx = [];
plansy = [];
planss = [];
targets = [];
planc = 10;
x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';
%% k2
history2 = zeros(tend*eulersteps,model.nvar+1);
splinepointhist2 = zeros(tend,pointsN*3+1);
plansx2 = [];
plansy2 = [];
planss2 = [];
targets2 = [];
planc2 = 10;

x02 = [zeros(model.N,index.nu),repmat(xs2,model.N,1)]';
tstart = 1;
%% Communal
a=0;
a2=0;
IND=[];
IND2=[];
Pos1=repmat(pstart, model.N-1 ,1);
Pos2=repmat(pstart2, model.N-1 ,1);
cost1 = zeros(tend,1);
cost2 = zeros(tend,1);
Progress1 = zeros(tend,1);
Progress2 = zeros(tend,1);
costT1 = zeros(tend,Nit);
costT2 = zeros(tend,Nit);
%% Simulation
for i =1:tend
    tstart = i;
    %find bspline
    if(1)
        if xs(index.s-index.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart = splinestart+1;
            xs(index.s-index.nu)=xs(index.s-index.nu)-1;
        end
        if xs2(index.s-index.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart2 = splinestart2+1;
            xs2(index.s-index.nu)=xs2(index.s-index.nu)-1;
        end
    end

    xs(index.ab-index.nu)=min(casadiGetMaxAcc(xs(index.v-index.nu))...
        -0.0001,xs(index.ab-index.nu));
    problem.xinit = xs';
    
    xs2(index.ab-index.nu)=min(casadiGetMaxAcc(xs2(index.v-index.nu))...
        -0.0001,xs2(index.ab-index.nu));
    problem2.xinit = xs2';
    %do it every time because we don't care about the performance of this
    %script
    
    ip = splinestart;
    [nkp, ~] = size(points);
    nextSplinePoints = zeros(pointsN,3);
    for ii=1:pointsN
       while ip>nkp
            ip = ip -nkp;
       end
       nextSplinePoints(ii,:)=points(ip,:);
       ip = ip + 1;
    end
    splinepointhist(i,:)=[xs(index.s-index.nu),nextSplinePoints(:)'];
    
    %go kart 2
    ip2 = splinestart2;
    [nkp2, ~] = size(points2);
    nextSplinePoints2 = zeros(pointsN,3);
    for ii=1:pointsN
       while ip2>nkp2
            ip2 = ip2 -nkp2;
       end
       nextSplinePoints2(ii,:)=points2(ip2,:);
       ip2 = ip2 + 1;
    end
    splinepointhist2(i,:)=[xs2(index.s-index.nu),nextSplinePoints2(:)'];
    
    % parameters
    problem.all_parameters = repmat (getParameters_IBR(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs2(1),xs2(2),nextSplinePoints),...
        model.N ,1);
    
    problem.all_parameters(index.xComp:model.npar:end)=[Pos2(:,1);...
                                                        Pos2(end,1)];
    problem.all_parameters(index.yComp:model.npar:end)=[Pos2(:,2);...
                                                        Pos2(end,2)];
   
    problem.x0 = x0(:);
       
    % parameters
    problem2.all_parameters = repmat (getParameters_IBR(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs(1),xs(2),nextSplinePoints2),...
        model.N ,1);
    
    problem2.all_parameters(index.xComp:model.npar:end)=[Pos1(:,1);...
                                                         Pos1(end,1)];
    problem2.all_parameters(index.yComp:model.npar:end)=[Pos1(:,2);...
                                                         Pos1(end,2)];
    problem2.x0 = x02(:);
    

    %go kart 1
    [output,exitflag,info] = MPCPathFollowing_IBR1(problem);
    solvetimes(end+1)=info.solvetime;
    if(exitflag==0)
        a =a+ 1;
        IND=[IND;i];
    end
    if(exitflag~=1 && exitflag ~=0)
       keyboard
    end
    outputM = reshape(output.alldata,[model.nvar,model.N])';
    problem2.all_parameters(index.xComp:model.npar:end)=...
        outputM(:,index.x);
    problem2.all_parameters(index.yComp:model.npar:end)=...
        outputM(:,index.y);
    %go kart 2
    [output2,exitflag2,info2] = MPCPathFollowing_IBR1(problem2);
    solvetimes2(end+1)=info2.solvetime;
    if(exitflag2==0)
        a2 =a2+ 1;
        IND2=[IND2;i];
    end
    if(exitflag2~=1 && exitflag2 ~=0)
        keyboard           
    end

    outputM2 = reshape(output2.alldata,[model.nvar,model.N])';

    problem.all_parameters(index.xComp:model.npar:end)=...
        outputM2(:,index.x);
    problem.all_parameters(index.yComp:model.npar:end)=...
        outputM2(:,index.y);
    costT1(i)=info.pobj;
    costT2(i)=info2.pobj;
    %outputM = reshape(output.alldata,[model.nvar,model.N])';

    x0 = outputM';
    u = repmat(outputM(1,1:index.nu),eulersteps,1);
    [xhist,time] = euler(@(x,u)interstagedx(x,u),xs,u,...
        integrator_stepsize/eulersteps);
    xs = xhist(end,:);
    xs
    history((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
        [time(1:end-1)+(tstart-1)*integrator_stepsize,u,xhist(1:end-1,:)];
    planc = planc + 1;
    if(planc>planintervall)
       planc = 1; 
       plansx = [plansx; outputM(:,index.x)'];
       plansy = [plansy; outputM(:,index.y)'];
       planss = [planss; outputM(:,index.s)'];
       [tx,ty]=casadiDynamicBSPLINE(outputM(end,index.s),nextSplinePoints);
       targets = [targets;tx,ty];
    end
    Pos1=[outputM(2:end,index.x),outputM(2:end,index.y)];
    
    %outputM2 = reshape(output2.alldata,[model.nvar,model.N])';
    Pos2=[outputM2(2:end,index.x),outputM2(2:end,index.y)];
    x02 = outputM2';
    u2 = repmat(outputM2(1,1:index.nu),eulersteps,1);
    [xhist2,time2] = euler(@(x2,u2)interstagedx(x2,u2),...
        xs2,u2,integrator_stepsize/eulersteps);
    xs2 = xhist2(end,:);
    xs2
    history2((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
        [time2(1:end-1)+(tstart-1)*integrator_stepsize,u2,...
        xhist2(1:end-1,:)];
    planc2 = planc2 + 1;
    if(planc2>planintervall)
       planc2 = 1; 
       plansx2 = [plansx2; outputM2(:,index.x)'];
       plansy2 = [plansy2; outputM2(:,index.y)'];
       planss2 = [planss2; outputM2(:,index.s)'];
       [tx2,ty2]=casadiDynamicBSPLINE(outputM2(end,index.s),nextSplinePoints2);
       targets2 = [targets2;tx2,ty2];
    end
    cost1(i)=info.pobj;
    cost2(i)=info2.pobj;
    %costS(i)=costS;
    Progress1(i)=outputM(1,index.s);
    Progress2(i)=outputM2(1,index.s);
    % check
    Pos2=[outputM2(2:end,index.x),outputM2(2:end,index.y)];
    distanceX=xs(1)-xs2(1);
    distanceY=xs(2)-xs2(2);
   
    squared_distance_array   = sqrt(distanceX.^2 + distanceY.^2);
    if squared_distance_array<=dist
        squared_distance_array
    end
end
%[t,ab,dotbeta,x,y,theta,v,beta,s]
drawIBR

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