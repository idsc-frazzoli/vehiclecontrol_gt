%add force path (change that for yourself)
% addpath('..');
% userDir = getuserdir;
% addpath('casadi');
% addpath('models');  
% addpath('draw_files');
% addpath('parameters_vector');
% addpath('objective_function');
% addpath('constraints');
% addpath('index_script');

%% Parameters Definitions
% parameters_2_vehicles
% pointsO=18;
%% Initialization for simulation
global index_IBR
% indexes_2_vehicles_IBR

%% Initialization for simulation
fpoints = points(1:2,1:2);
pdir = diff(fpoints);
[pstartx,pstarty] = casadiDynamicBSPLINE(0.01,points);
pstart = [pstartx,pstarty];
pangle = atan2(pdir(2),pdir(1));
xs(index_IBR.x-index_IBR.nu)=pstart(1);
xs(index_IBR.y-index_IBR.nu)=pstart(2);
xs(index_IBR.theta-index_IBR.nu)=pangle;
xs(index_IBR.v-index_IBR.nu)=1;
xs(index_IBR.ab-index_IBR.nu)=0;
xs(index_IBR.beta-index_IBR.nu)=0;
xs(index_IBR.s-index_IBR.nu)=0.01;
plansx = [];
plansy = [];
planss = [];
targets = [];
planc = 10;

x0 = [zeros(model.N,index_IBR.nu),repmat(xs,model.N,1)]';

%% Initialization for simulation 2
fpoints2 = points2(1:2,1:2);
pdir2 = diff(fpoints2);
[pstartx2,pstarty2] = casadiDynamicBSPLINE(0.01,points2);
pstart2 = [pstartx2,pstarty2];
pangle2 = atan2(pdir2(2),pdir2(1));
xs2(index_IBR.x-index_IBR.nu)=pstart2(1);
xs2(index_IBR.y-index_IBR.nu)=pstart2(2);
xs2(index_IBR.theta-index_IBR.nu)=pangle2;
xs2(index_IBR.v-index_IBR.nu)=1;
xs2(index_IBR.ab-index_IBR.nu)=0;
xs2(index_IBR.beta-index_IBR.nu)=0;
xs2(index_IBR.s-index_IBR.nu)=0.01;
%% Simulation
history = zeros(tend*eulersteps,model.nvar+1);
splinepointhist = zeros(tend,pointsN*3+1);
plansx = [];
plansy = [];
planss = [];
targets = [];
planc = 10;
x0 = [zeros(model.N,index_IBR.nu),repmat(xs,model.N,1)]';
%% k2
history2 = zeros(tend*eulersteps,model.nvar+1);
splinepointhist2 = zeros(tend,pointsN*3+1);
plansx2 = [];
plansy2 = [];
planss2 = [];
targets2 = [];
planc2 = 10;

x02 = [zeros(model.N,index_IBR.nu),repmat(xs2,model.N,1)]';
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
costT1 = zeros(tend,1);
costT2 = zeros(tend,1);
%% Simulation
for i =1:tend
    tstart = i;
    %find bspline
    if(1)
        if xs(index_IBR.s-index_IBR.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart = splinestart+1;
            xs(index_IBR.s-index_IBR.nu)=xs(index_IBR.s-index_IBR.nu)-1;
        end
        if xs2(index_IBR.s-index_IBR.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart2 = splinestart2+1;
            xs2(index_IBR.s-index_IBR.nu)=xs2(index_IBR.s-index_IBR.nu)-1;
        end
    end

    xs(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs(index_IBR.v-index_IBR.nu))...
        -0.0001,xs(index_IBR.ab-index_IBR.nu));
    problem.xinit = xs';
    
    xs2(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs2(index_IBR.v-index_IBR.nu))...
        -0.0001,xs2(index_IBR.ab-index_IBR.nu));
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
    splinepointhist(i,:)=[xs(index_IBR.s-index_IBR.nu),nextSplinePoints(:)'];
    
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
    splinepointhist2(i,:)=[xs2(index_IBR.s-index_IBR.nu),nextSplinePoints2(:)'];
    
    % parameters
    problem.all_parameters = repmat (getParameters_IBR(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs2(1),xs2(2),nextSplinePoints),...
        model.N ,1);
    
    problem.all_parameters(index_IBR.xComp:model.npar:end)=[Pos2(:,1);...
                                                        Pos2(end,1)];
    problem.all_parameters(index_IBR.yComp:model.npar:end)=[Pos2(:,2);...
                                                        Pos2(end,2)];
   
    problem.x0 = x0(:);
       
    % parameters
    problem2.all_parameters = repmat (getParameters_IBR(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs(1),xs(2),nextSplinePoints2),...
        model.N ,1);
    
    problem2.all_parameters(index_IBR.xComp:model.npar:end)=[Pos1(:,1);...
                                                         Pos1(end,1)];
    problem2.all_parameters(index_IBR.yComp:model.npar:end)=[Pos1(:,2);...
                                                         Pos1(end,2)];
    problem2.x0 = x02(:);
    
     
    %go kart 1
    [output,exitflag,info] = MPCPathFollowing_2v_IBR(problem);
    solvetimes(end+1)=info.solvetime;
    if(exitflag==0)
        a =a+ 1;
        IND=[IND;i];
    end
    if(exitflag~=1 && exitflag ~=0)
       keyboard
    end
    outputM = reshape(output.alldata,[model.nvar,model.N])';
    problem2.all_parameters(index_IBR.xComp:model.npar:end)=...
        outputM(:,index_IBR.x);
    problem2.all_parameters(index_IBR.yComp:model.npar:end)=...
        outputM(:,index_IBR.y);
    %go kart 2
    [output2,exitflag2,info2] = MPCPathFollowing_2v_IBR(problem2);
    solvetimes2(end+1)=info2.solvetime;
    if(exitflag2==0)
        a2 =a2+ 1;
        IND2=[IND2;i];
    end
    if(exitflag2~=1 && exitflag2 ~=0)
        keyboard           
    end

    outputM2 = reshape(output2.alldata,[model.nvar,model.N])';

    problem.all_parameters(index_IBR.xComp:model.npar:end)=...
        outputM2(:,index_IBR.x);
    problem.all_parameters(index_IBR.yComp:model.npar:end)=...
        outputM2(:,index_IBR.y);
    costT1(i)=info.pobj;
    costT2(i)=info2.pobj;
    %outputM = reshape(output.alldata,[model.nvar,model.N])';

    x0 = outputM';
    u = repmat(outputM(1,1:index_IBR.nu),eulersteps,1);
    [xhist,time] = euler(@(x,u)interstagedx_IBR(x,u),xs,u,...
        integrator_stepsize/eulersteps);
    xs = xhist(end,:);
    xs
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
    
    %outputM2 = reshape(output2.alldata,[model.nvar,model.N])';
    Pos2=[outputM2(2:end,index_IBR.x),outputM2(2:end,index_IBR.y)];
    x02 = outputM2';
    u2 = repmat(outputM2(1,1:index_IBR.nu),eulersteps,1);
    [xhist2,time2] = euler(@(x2,u2)interstagedx_IBR(x2,u2),...
        xs2,u2,integrator_stepsize/eulersteps);
    xs2 = xhist2(end,:);
    xs2
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
