clear all
close all
clc
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
pslack=10;
pslack2=100;
dist=2;

% Splines
pointsO = 18;
pointsN = 10;
pointsN2 = 10;
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;

% Simulation length
tend = 1;
eulersteps = 10;
planintervall = 1;

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


%% Spline Points

points2 = [40,45,50,55,60,65,70,75,80,85,90,95,100,105,110;...          %x
          50,50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
          4,4,4,4,4,4,4,4,4,4,4,4,4,4,4]';  
points = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
          40,45,50,55,60,65,70,75,80,85,90,95,100,105,110; ...    %y
          4,4,4,4,4,4,4,4,4,4,4,4,4,4,4]';
solvetimes = [];

solvetimes2 = [];
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
model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;...
            -inf;-inf;-inf;-inf;-inf;-inf;-inf];%

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

%% problem 1
% Initialization for simulation kart 1
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
x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';

%% problem 2
%Initialization for simulation kart 1
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

x02 = [zeros(model.N,index.nu),repmat(xs2,model.N,1)]';
%% Simulation
history = zeros(tend*eulersteps,model.nvar+1);
splinepointhist = zeros(tend,pointsN*3+pointsN2*3+1);
a=0;

history2 = zeros(tend*eulersteps,model.nvar+1);
splinepointhist2 = zeros(tend,pointsN*3+pointsN2*3+1);
a2=0;
alpha1=0.2:0.2:1;
alpha2=0.2:0.2:1;
optA = zeros(1,length(alpha1));
optB = zeros(1,length(alpha1));
opt = zeros(1,length(alpha1));
optA2 = zeros(1,length(alpha2));
optB2 = zeros(1,length(alpha2));
opt2 = zeros(1,length(alpha2));
cost1 = zeros(1,length(alpha1));
cost2 = zeros(1,length(alpha2));
Progress1 = zeros(1,length(alpha1));
Progress2 = zeros(1,length(alpha2));
% go kart 1
xs(index.ab-index.nu)=min(casadiGetMaxAcc(xs(index.v...
    -index.nu))-0.0001,xs(index.ab-index.nu));
% go kart 2
xs(index.ab_k2-index.nu)=min(casadiGetMaxAcc(xs(index.v_k2...
    -index.nu))-0.0001,xs(index.ab_k2-index.nu));
% go kart 1
xs2(index.ab-index.nu)=min(casadiGetMaxAcc(xs2(index.v...
    -index.nu))-0.0001,xs2(index.ab-index.nu));
% go kart 2
xs2(index.ab_k2-index.nu)=min(casadiGetMaxAcc(xs2(index.v_k2...
    -index.nu))-0.0001,xs2(index.ab_k2-index.nu));

problem.xinit = [xs(1:index.ns/2),xs2(index.ns/2+1:end)]';
problem2.xinit = [xs(1:index.ns/2),xs2(index.ns/2+1:end)]';
problem.x0 = x0(:);   
problem2.x0 = x02(:); 
tstart = 1;
tstart2 = 1;
for kk=1:length(alpha1)

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
    
    % parameters
    problem.all_parameters = repmat(getParameters_PG_alpha(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,alpha1(kk),alpha2(end),nextSplinePoints,...
        nextSplinePoints_k2), model.N ,1);
    % parameters
    problem2.all_parameters = repmat(getParameters_PG_alpha(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,alpha1(end),alpha2(kk),nextSplinePoints3,...
        nextSplinePoints4_k2), model.N ,1);
    % solve mpc
    [output,exitflag,info] = MPCPathFollowing_v1(problem);
    [output2,exitflag2,info2] = MPCPathFollowing_v2(problem2);
    outputM = reshape(output.alldata,[model.nvar,model.N])';

    outputM2 = reshape(output2.alldata,[model2.nvar,model2.N])';
    %% Evaluation cost function
    for jj=1:length(outputM)
        [lagcost_A,latcost_A,reg_A,prog_A,slack_A,speedcost_A,lagcost_A_k2,...
        latcost_A_k2,reg_A_k2,prog_A_k2,slack_A_k2,speedcost_A_k2,f,f1,f2] =...
        objective_PG_TestAlpha(outputM(jj,:),nextSplinePoints,nextSplinePoints_k2,...
        maxSpeed,plagerror, platerror, pprog, pab, pdotbeta,...
        pspeedcost,pslack,pslack2,alpha1(kk),alpha2(end));
        [lagcost_B,latcost_B,reg_B,prog_B,slack_B,speedcost_B,lagcost_B_k2,...
        latcost_B_k2,reg_B_k2,prog_B_k2,slack_B_k2,speedcost_B_k2,f3,f4,f5] =...
        objective_PG_TestAlpha(outputM2(jj,:),nextSplinePoints3,nextSplinePoints4_k2,...
        maxSpeed,plagerror, platerror, pprog, pab, pdotbeta,...
        pspeedcost,pslack,pslack2,alpha1(end),alpha2(kk));
        optA(kk)= optA(kk)+ f1;
        optB(kk)= optB(kk)+ f2;
        opt(kk) = opt(kk)+ f;
        optA2(kk)= optA2(kk)+ f4;
        optB2(kk)= optB2(kk)+ f5;
        opt2(kk) = opt2(kk)+ f3;
    end
       
    cost1(kk)=info.pobj;
    cost2(kk)=info2.pobj;
    Progress1(kk)=outputM(1,index.s);
    Progress2(kk)=outputM2(1,index.s_k2);
    if(1)
        figure(1)
        plot(pstart(1),pstart(2),'g*')
        hold on
        plot(pstart2(1),pstart2(2),'g*') 
        plot([pstart(1)-2,pstart(1)-2],[pstart(2),54],'--k')
        plot([pstart(1)+2,pstart(1)+2],[pstart(2),54],'--k')
        plot([pstart2(1),54],[pstart2(2)-2,pstart2(2)-2],'--k')
        plot([pstart2(1),54],[pstart2(2)+2,pstart2(2)+2],'--k')
        grid on
        title('trajectory')
        xlabel('X')
        ylabel('Y')

        figure(2)
        hold on
        grid on
        title('steer')
        xlabel('step')
        ylabel('')

        figure(3)
        hold on
        grid on
        title('vel')
        xlabel('step')
        ylabel('')

        figure(4)
        hold on
        grid on
        title('Costs')
        xlabel('Alpha1,Alpha2')
        ylabel('Costs')

        figure(5)
        hold on
        grid on
        title('Percentage Costs')
        xlabel('Alpha1,Alpha2')
        ylabel('Costs')
    end

    figure(1)
    plot(outputM(:,index.x),outputM(:,index.y),'.-','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
    plot(outputM(:,index.x_k2),outputM(:,index.y_k2),'.-','Color',[1,alpha1(kk)/max(alpha1)*0.5,0])
    plot(outputM2(:,index.x),outputM2(:,index.y),'.-','Color',[0,1-alpha2(kk)/max(alpha2)*0.5,1])
    plot(outputM2(:,index.x_k2),outputM2(:,index.y_k2),'.-','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])

    figure(2)
    plot(outputM(:,index.theta),'.-','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
    plot(outputM(:,index.theta_k2),'.-','Color',[1,alpha1(kk)/max(alpha1)*0.5,0])
    plot(outputM2(:,index.theta),'.-','Color',[0,1-alpha2(kk)/max(alpha2)*0.5,1])
    plot(outputM2(:,index.theta_k2),'.-','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])
    
    figure(3)
    plot(outputM(:,index.v),'.-','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
    plot(outputM(:,index.v_k2),'.-','Color',[1,alpha1(kk)/max(alpha1)*0.5,0])
    plot(outputM2(:,index.v),'.-','Color',[0,1-alpha2(kk)/max(alpha2)*0.5,1])
    plot(outputM2(:,index.v_k2),'.-','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])
end

figure(4)
for kk=1:length(alpha1)
    plot(alpha1(kk),optA(kk),'*','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
    plot(alpha1(kk),optB(kk),'*','Color',[1,alpha1(kk)/max(alpha1)*0.5,0])
    plot(alpha1(kk),(optA(kk)+optB(kk))/2,'o','Color',[0,1,0])
    plot(2-alpha2(kk),optA2(kk),'*','Color',[0,1-alpha2(kk)/max(alpha2)*0.5,1])
    plot(2-alpha2(kk),optB2(kk),'*','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])
    plot(2-alpha2(kk),(optA2(kk)+optB2(kk))/2,'o','Color',[0,1,0])
end
legend('V1_{down}','V2_{left}','Tot/2')
xticklabels({'0.2,1','0.4,1','0.6,1',...
    '0.8,1','1,1','1,0.8','1,0.6',...
    '1,0.4','1,0.2'});
figure(5)
for kk=1:length(alpha1)
    plot(alpha1(kk),optA(kk)/(optA(kk)+optB(kk))*100,'*','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
    plot(alpha1(kk),optB(kk)/(optA(kk)+optB(kk))*100,'*','Color',[1,alpha1(kk)/max(alpha1)*0.5,0])
    plot(2-alpha2(kk),optA2(kk)/(optA2(kk)+optB2(kk))*100,'*','Color',[0,1-alpha2(kk)/max(alpha2)*0.5,1])
    plot(2-alpha2(kk),optB2(kk)/(optA2(kk)+optB2(kk))*100,'*','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])
end
legend('V1_{down}','V2_{left}')
xticklabels({'0.2,1','0.4,1','0.6,1',...
    '0.8,1','1,1','1,0.8','1,0.6',...
    '1,0.4','1,0.2'});
% xticklabels({'1,0.1','1,0.2','1,0.3','1,0.4','1,0.5','1,0.6','1,0.7',...
%     '1,0.8','1,0.9','1,1','0.9,1','0.8,1','0.7,1','0.6,1','0.5,1',...
%     '0.4,1','0.3,1','0.2,1','0.1,1'});