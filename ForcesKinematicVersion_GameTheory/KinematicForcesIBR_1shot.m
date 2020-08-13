%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kynematic MPC Script IBR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% code by em
% 2 vehicle running in the an intersection, with 
% constraints on collisions. The iterations run until a Nash equilibrium is
% reached (Iterated Best response)

%add force path (change that for yourself)

addpath('..');
userDir = pwd;
addpath('/home/thomas/Matlab/Forces/forces_pro_client');
addpath('/home/thomas/Matlab/Forces/forces_pro_client/casadi-matlabR2014a-v2.4.2');

% addpath('C:\Users\me\Documents\FORCES_client');
addpath('casadi');
addpath('../shared_dynamic')
    
clear model_IBR
clear problem_IBR
clear all
%close all

%% Parameters Definitions
maxs_IBRpeed = 10;
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
pdotbeta=0.05;
pspeedcost=0.2;
pslack=5;
pslack2=50;
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
points2 = [5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110;...          %x
          50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
points = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
          5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';
% points = [41.8,36.2,52,57.2,53,52,47;...          %x
%           38.33,44.933,58.2,53.8,49,44,43; ...    %y
%           2.5,2.5,2.5,2.5,2.5,2.5,2.5]';         %width 
% points2 = [57.2,52,36.2,41.8,47,52,53;...          %x
%           53.8,58.2,44.933,38.33,43,44,49; ...    %y
%           2.5,2.5,2.5,2.5,2.5,2.5,2.5]';       

solvetimes = [];
solvetimes2=[];
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

integrator_stepsize = 0.1;
trajectorytimestep = integrator_stepsize;

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

output = newOutput('alldata', 1:model_IBR.N, 1:model_IBR.nvar);

FORCES_NLP(model_IBR, codeoptions,output);

%% Initialization for simulation
fpoints = points(1:2,1:2);
pdir = diff(fpoints);
[pstartx,pstarty] = casadiDynamicBSPLINE(0.01,points);
pstart = [pstartx,pstarty];
pangle = atan2(pdir(2),pdir(1));
xs_IBR(index_IBR.x-index_IBR.nu)=pstart(1);
xs_IBR(index_IBR.y-index_IBR.nu)=pstart(2);
xs_IBR(index_IBR.theta-index_IBR.nu)=pangle;
xs_IBR(index_IBR.v-index_IBR.nu)=1;
xs_IBR(index_IBR.ab-index_IBR.nu)=0;
xs_IBR(index_IBR.beta-index_IBR.nu)=0;
xs_IBR(index_IBR.s-index_IBR.nu)=0.01;

%% Initialization for simulation 2
fpoints2 = points2(1:2,1:2);
pdir2 = diff(fpoints2);
[pstartx2,pstarty2] = casadiDynamicBSPLINE(0.01,points2);
pstart2 = [pstartx2,pstarty2];
pangle2 = atan2(pdir2(2),pdir2(1));
xs_IBR2(index_IBR.x-index_IBR.nu)=pstart2(1);
xs_IBR2(index_IBR.y-index_IBR.nu)=pstart2(2);
xs_IBR2(index_IBR.theta-index_IBR.nu)=pangle2;
xs_IBR2(index_IBR.v-index_IBR.nu)=1;
xs_IBR2(index_IBR.ab-index_IBR.nu)=0;
xs_IBR2(index_IBR.beta-index_IBR.nu)=0;
xs_IBR2(index_IBR.s-index_IBR.nu)=0.01;
%% Simulation
history = zeros(tend*eulersteps,model_IBR.nvar+1);
splinepointhist = zeros(tend,pointsN*3+1);
plansx = [];
plansy = [];
planss = [];
targets = [];
planc = 10;
x0_IBR = [zeros(model_IBR.N,index_IBR.nu),repmat(xs_IBR,model_IBR.N,1)]';
%% k2
history2 = zeros(tend*eulersteps,model_IBR.nvar+1);
splinepointhist2 = zeros(tend,pointsN*3+1);
plansx2 = [];
plansy2 = [];
planss2 = [];
targets2 = [];
planc2 = 10;

x0_IBR2 = [zeros(model_IBR.N,index_IBR.nu),repmat(xs_IBR2,model_IBR.N,1)]';
tstart = 1;
%% Communal
a=0;
a2=0;
IND=[];
IND2=[];
Pos1=repmat(pstart, model_IBR.N-1 ,1);
Pos2=repmat(pstart2, model_IBR.N-1 ,1);
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
        if xs_IBR(index_IBR.s-index_IBR.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart = splinestart+1;
            xs_IBR(index_IBR.s-index_IBR.nu)=xs_IBR(index_IBR.s-index_IBR.nu)-1;
        end
        if xs_IBR2(index_IBR.s-index_IBR.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart2 = splinestart2+1;
            xs_IBR2(index_IBR.s-index_IBR.nu)=xs_IBR2(index_IBR.s-index_IBR.nu)-1;
        end
    end

    xs_IBR(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs_IBR(index_IBR.v-index_IBR.nu))...
        -0.0001,xs_IBR(index_IBR.ab-index_IBR.nu));
    problem_IBR.xinit = xs_IBR';
    
    xs_IBR2(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs_IBR2(index_IBR.v-index_IBR.nu))...
        -0.0001,xs_IBR2(index_IBR.ab-index_IBR.nu));
    problem_IBR2.xinit = xs_IBR2';
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
    splinepointhist(i,:)=[xs_IBR(index_IBR.s-index_IBR.nu),nextSplinePoints(:)'];
    
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
    splinepointhist2(i,:)=[xs_IBR2(index_IBR.s-index_IBR.nu),nextSplinePoints2(:)'];
    
    % parameters
    problem_IBR.all_parameters = repmat (getParameters_IBR(maxs_IBRpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs_IBR2(1),xs_IBR2(2),nextSplinePoints),...
        model_IBR.N ,1);
    
    problem_IBR.all_parameters(index_IBR.xComp:model_IBR.npar:end)=[Pos2(:,1);...
                                                        Pos2(end,1)];
    problem_IBR.all_parameters(index_IBR.yComp:model_IBR.npar:end)=[Pos2(:,2);...
                                                        Pos2(end,2)];
   
    problem_IBR.x0_IBR = x0_IBR(:);
       
    % parameters
    problem_IBR2.all_parameters = repmat (getParameters_IBR(maxs_IBRpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs_IBR(1),xs_IBR(2),nextSplinePoints2),...
        model_IBR.N ,1);
    
    problem_IBR2.all_parameters(index_IBR.xComp:model_IBR.npar:end)=[Pos1(:,1);...
                                                         Pos1(end,1)];
    problem_IBR2.all_parameters(index_IBR.yComp:model_IBR.npar:end)=[Pos1(:,2);...
                                                         Pos1(end,2)];
    problem_IBR2.x0_IBR = x0_IBR2(:);
    
    jj=1;
    while jj<=Nit
        
        %go kart 1
        [output,exitflag,info] = MPCPathFollowing_IBR(problem_IBR);
        solvetimes(end+1)=info.solvetime;
        if(exitflag==0)
            a =a+ 1;
            IND=[IND;i];
        end
        if(exitflag~=1 && exitflag ~=0)
           keyboard
        end
        outputM = reshape(output.alldata,[model_IBR.nvar,model_IBR.N])';
        problem_IBR2.all_parameters(index_IBR.xComp:model_IBR.npar:end)=...
            outputM(:,index_IBR.x);
        problem_IBR2.all_parameters(index_IBR.yComp:model_IBR.npar:end)=...
            outputM(:,index_IBR.y);
        %go kart 2
        [output2,exitflag2,info2] = MPCPathFollowing_IBR(problem_IBR2);
        solvetimes2(end+1)=info2.solvetime;
        if(exitflag2==0)
            a2 =a2+ 1;
            IND2=[IND2;i];
        end
        if(exitflag2~=1 && exitflag2 ~=0)
            keyboard           
        end
               
        outputM2 = reshape(output2.alldata,[model_IBR.nvar,model_IBR.N])';

        problem_IBR.all_parameters(index_IBR.xComp:model_IBR.npar:end)=...
            outputM2(:,index_IBR.x);
        problem_IBR.all_parameters(index_IBR.yComp:model_IBR.npar:end)=...
            outputM2(:,index_IBR.y);
        costT1(i,jj)=info.pobj;
        costT2(i,jj)=info2.pobj;
        jj=jj+1;

    end
    %outputM = reshape(output.alldata,[model_IBR.nvar,model_IBR.N])';

    x0_IBR = outputM';
    u = repmat(outputM(1,1:index_IBR.nu),eulersteps,1);
    [xhist,time] = euler(@(x,u)interstagedx(x,u),xs_IBR,u,...
        integrator_stepsize/eulersteps);
    xs_IBR = xhist(end,:);
    xs_IBR
    history((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
        [time(1:end-1)+(tstart-1)*integrator_stepsize,u,xhist(1:end-1,:)];
    planc = planc + 1;
    if(planc>planintervall)
       planc = 1; 
       plansx = [plansx; outputM(:,index_IBR.x)'];
       plansy = [plansy; outputM(:,index_IBR.y)'];
       planss = [planss; outputM(:,index_IBR.s)'];
       [tx,ty]=casadiDynamicBSPLINE(outputM(end,index_IBR.s),nextSplinePoints);
       targets = [targets;tx,ty];
    end
    Pos1=[outputM(2:end,index_IBR.x),outputM(2:end,index_IBR.y)];
    
    %outputM2 = reshape(output2.alldata,[model_IBR.nvar,model_IBR.N])';
    Pos2=[outputM2(2:end,index_IBR.x),outputM2(2:end,index_IBR.y)];
    x0_IBR2 = outputM2';
    u2 = repmat(outputM2(1,1:index_IBR.nu),eulersteps,1);
    [xhist2,time2] = euler(@(x2,u2)interstagedx(x2,u2),...
        xs_IBR2,u2,integrator_stepsize/eulersteps);
    xs_IBR2 = xhist2(end,:);
    xs_IBR2
    history2((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
        [time2(1:end-1)+(tstart-1)*integrator_stepsize,u2,...
        xhist2(1:end-1,:)];
    planc2 = planc2 + 1;
    if(planc2>planintervall)
       planc2 = 1; 
       plansx2 = [plansx2; outputM2(:,index_IBR.x)'];
       plansy2 = [plansy2; outputM2(:,index_IBR.y)'];
       planss2 = [planss2; outputM2(:,index_IBR.s)'];
       [tx2,ty2]=casadiDynamicBSPLINE(outputM2(end,index_IBR.s),nextSplinePoints2);
       targets2 = [targets2;tx2,ty2];
    end
    cost1(i)=info.pobj;
    cost2(i)=info2.pobj;
    %costS(i)=costS;
    Progress1(i)=outputM(1,index_IBR.s);
    Progress2(i)=outputM2(1,index_IBR.s);
    % check
    Pos2=[outputM2(2:end,index_IBR.x),outputM2(2:end,index_IBR.y)];
    distanceX=xs_IBR(1)-xs_IBR2(1);
    distanceY=xs_IBR(2)-xs_IBR2(2);
   
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