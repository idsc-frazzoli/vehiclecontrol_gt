%% Initialization for simulation
global index_IBR

dis=1.5;
%% Initialization for simulation
fpoints = points(1:2,1:2);
pdir = diff(fpoints);
[pstartx,pstarty] = casadiDynamicBSPLINE(0.01,points);
pstart = [pstartx,pstarty];
pangle = atan2(pdir(2),pdir(1));
xs(index_IBR.x-index_IBR.nu)=pstart(1);
xs(index_IBR.y-index_IBR.nu)=pstart(2)-dis;
xs(index_IBR.theta-index_IBR.nu)=pangle;
xs(index_IBR.v-index_IBR.nu)=6;
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
xs2(index_IBR.x-index_IBR.nu)=pstart2(1)+dis;
xs2(index_IBR.y-index_IBR.nu)=pstart2(2);
xs2(index_IBR.theta-index_IBR.nu)=pangle2;
xs2(index_IBR.v-index_IBR.nu)=6;
xs2(index_IBR.ab-index_IBR.nu)=0;
xs2(index_IBR.beta-index_IBR.nu)=0;
xs2(index_IBR.s-index_IBR.nu)=0.01;

%% Initialization for simulation 3
fpoints3 = points3(1:2,1:2);
pdir3 = diff(fpoints3);
[pstartx3,pstarty3] = casadiDynamicBSPLINE(0.01,points3);
pstart3 = [pstartx3,pstarty3];
pangle3 = atan2(pdir3(2),pdir3(1));
xs3(index_IBR.x-index_IBR.nu)=pstart3(1)-dis;
xs3(index_IBR.y-index_IBR.nu)=pstart3(2);
xs3(index_IBR.theta-index_IBR.nu)=pangle3;
xs3(index_IBR.v-index_IBR.nu)=6;
xs3(index_IBR.ab-index_IBR.nu)=0;
xs3(index_IBR.beta-index_IBR.nu)=0;
xs3(index_IBR.s-index_IBR.nu)=0.01;
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
%% k2
history3 = zeros(tend*eulersteps,model.nvar+1);
splinepointhist3 = zeros(tend,pointsN*3+1);
plansx3 = [];
plansy3 = [];
planss3 = [];
targets3 = [];
planc3 = 10;

x03 = [zeros(model.N,index_IBR.nu),repmat(xs3,model.N,1)]';
%% Communal
a=0;
a2=0;
a3=0;
IND=[];
IND2=[];
IND3=[];
Pos1=repmat(pstart, model.N-1 ,1);
Pos2=repmat(pstart2, model.N-1 ,1);
Pos3=repmat(pstart3, model.N-1 ,1);
cost1 = zeros(tend,1);
cost2 = zeros(tend,1);
cost3 = zeros(tend,1);
Progress1 = zeros(tend,1);
Progress2 = zeros(tend,1);
Progress3 = zeros(tend,1);
costT1 = zeros(tend,1);
costT2 = zeros(tend,1);
costT3 = zeros(tend,1);
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
        if xs3(index_IBR.s-index_IBR.nu)>1
            %nextSplinePoints
            %spline step forward
            splinestart3 = splinestart3+1;
            xs3(index_IBR.s-index_IBR.nu)=xs3(index_IBR.s-index_IBR.nu)-1;
        end
    end

    xs(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs(index_IBR.v-index_IBR.nu))...
        -0.0001,xs(index_IBR.ab-index_IBR.nu));
    problem.xinit = xs';
    
    xs2(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs2(index_IBR.v-index_IBR.nu))...
        -0.0001,xs2(index_IBR.ab-index_IBR.nu));
    problem2.xinit = xs2';
    
    xs3(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs3(index_IBR.v-index_IBR.nu))...
        -0.0001,xs3(index_IBR.ab-index_IBR.nu));
    problem3.xinit = xs3';
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
    
    %go kart 3
    ip3 = splinestart3;
    [nkp3, ~] = size(points3);
    nextSplinePoints3 = zeros(pointsN,3);
    for ii=1:pointsN
       while ip3>nkp3
            ip3 = ip3 -nkp3;
       end
       nextSplinePoints3(ii,:)=points3(ip3,:);
       ip3 = ip3 + 1;
    end
    splinepointhist3(i,:)=[xs3(index_IBR.s-index_IBR.nu),nextSplinePoints3(:)'];
    
    % parameters
    problem.all_parameters = repmat (getParameters_IBR_3(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs2(1),xs2(2),xs3(1),xs3(2),...
        nextSplinePoints),model.N ,1);
    
    problem.all_parameters(index_IBR.xComp2:model.npar:end)=[Pos2(:,1);...
                                                        Pos2(end,1)];
    problem.all_parameters(index_IBR.yComp2:model.npar:end)=[Pos2(:,2);...
                                                        Pos2(end,2)];
    problem.all_parameters(index_IBR.xComp3:model.npar:end)=[Pos3(:,1);...
                                                        Pos3(end,1)];
    problem.all_parameters(index_IBR.yComp3:model.npar:end)=[Pos3(:,2);...
                                                        Pos3(end,2)];
   
    problem.x0 = x0(:);
       
    % parameters
    problem2.all_parameters = repmat (getParameters_IBR_3(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs(1),xs(2),xs3(1),xs3(2),...
        nextSplinePoints2),model.N ,1);
    
    problem2.all_parameters(index_IBR.xComp2:model.npar:end)=[Pos1(:,1);...
                                                         Pos1(end,1)];
    problem2.all_parameters(index_IBR.yComp2:model.npar:end)=[Pos1(:,2);...
                                                         Pos1(end,2)];
    problem2.all_parameters(index_IBR.xComp3:model.npar:end)=[Pos3(:,1);...
                                                         Pos3(end,1)];
    problem2.all_parameters(index_IBR.yComp3:model.npar:end)=[Pos3(:,2);...
                                                         Pos3(end,2)];
   
    problem2.x0 = x02(:);
    
    % parameters
    problem3.all_parameters = repmat (getParameters_IBR_3(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs(1),xs(2),xs2(1),xs2(2),nextSplinePoints3),...
        model.N ,1);
    
    problem3.all_parameters(index_IBR.xComp2:model.npar:end)=[Pos1(:,1);...
                                                         Pos1(end,1)];
    problem3.all_parameters(index_IBR.yComp2:model.npar:end)=[Pos1(:,2);...
                                                         Pos1(end,2)];
    problem3.all_parameters(index_IBR.xComp3:model.npar:end)=[Pos2(:,1);...
                                                         Pos2(end,1)];
    problem3.all_parameters(index_IBR.yComp3:model.npar:end)=[Pos2(:,2);...
                                                         Pos2(end,2)];
    problem3.x0 = x03(:);
    
    
            %go kart 3
    [output3,exitflag3,info3] = MPCPathFollowing_3v_IBR(problem3);
    solvetimes3(end+1)=info3.solvetime;
    if(exitflag3==0)
        a3 =a3+ 1;
        IND3=[IND3;i];
    end
    if(exitflag3~=1 && exitflag3 ~=0)
        keyboard           
    end

    outputM3 = reshape(output3.alldata,[model.nvar,model.N])';

    problem.all_parameters(index_IBR.xComp3:model.npar:end)=...
        outputM3(:,index_IBR.x);
    problem.all_parameters(index_IBR.yComp3:model.npar:end)=...
        outputM3(:,index_IBR.y);
    problem2.all_parameters(index_IBR.xComp3:model.npar:end)=...
        outputM3(:,index_IBR.x);
    problem2.all_parameters(index_IBR.yComp3:model.npar:end)=...
        outputM3(:,index_IBR.y);
    
   
    %go kart 1
    [output,exitflag,info] = MPCPathFollowing_3v_IBR(problem);
    solvetimes(end+1)=info.solvetime;
    if(exitflag==0)
        a =a+ 1;
        IND=[IND;i];
    end
    if(exitflag~=1 && exitflag ~=0)
       keyboard
    end
    outputM = reshape(output.alldata,[model.nvar,model.N])';
    problem2.all_parameters(index_IBR.xComp2:model.npar:end)=...
        outputM(:,index_IBR.x);
    problem2.all_parameters(index_IBR.yComp2:model.npar:end)=...
        outputM(:,index_IBR.y);
    problem3.all_parameters(index_IBR.xComp2:model.npar:end)=...
        outputM(:,index_IBR.x);
    problem3.all_parameters(index_IBR.yComp2:model.npar:end)=...
        outputM(:,index_IBR.y);
     %go kart 2
    [output2,exitflag2,info2] = MPCPathFollowing_3v_IBR(problem2);
    solvetimes2(end+1)=info2.solvetime;
    if(exitflag2==0)
        a2 =a2+ 1;
        IND2=[IND2;i];
    end
    if(exitflag2~=1 && exitflag2 ~=0)
        keyboard           
    end

    outputM2 = reshape(output2.alldata,[model.nvar,model.N])';

    problem.all_parameters(index_IBR.xComp2:model.npar:end)=...
        outputM2(:,index_IBR.x);
    problem.all_parameters(index_IBR.yComp2:model.npar:end)=...
        outputM2(:,index_IBR.y);
    problem3.all_parameters(index_IBR.xComp3:model.npar:end)=...
        outputM2(:,index_IBR.x);
    problem3.all_parameters(index_IBR.yComp3:model.npar:end)=...
        outputM2(:,index_IBR.y);
    
    costT1(i)=info.pobj;
    costT2(i)=info2.pobj;
    costT3(i)=info3.pobj;
    %outputM = reshape(output.alldata,[model.nvar,model.N])';

    x0 = outputM';
    u = repmat(outputM(1,1:index_IBR.nu),eulersteps,1);
    [xhist,time] = euler(@(x,u)interstagedx_IBR(x,u),xs,u,...
        integrator_stepsize/eulersteps);
    xs = xhist(end,:);
    %xs
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
    x02 = outputM2';
    u2 = repmat(outputM2(1,1:index_IBR.nu),eulersteps,1);
    [xhist2,time2] = euler(@(x2,u2)interstagedx_IBR(x2,u2),...
        xs2,u2,integrator_stepsize/eulersteps);
    xs2 = xhist2(end,:);
    %xs2
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
    Pos2=[outputM2(2:end,index_IBR.x),outputM2(2:end,index_IBR.y)];
    
    x03 = outputM3';
    u3 = repmat(outputM3(1,1:index_IBR.nu),eulersteps,1);
    [xhist3,time3] = euler(@(x3,u3)interstagedx_IBR(x3,u3),...
        xs3,u3,integrator_stepsize/eulersteps);
    xs3 = xhist3(end,:);
    %xs3
    history3((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
        [time3(1:end-1)+(tstart-1)*integrator_stepsize,u3,...
        xhist3(1:end-1,:)];
    planc3 = planc3 + 1;
    if(planc3>planintervall)
       planc3 = 1; 
       plansx3 = [plansx3; outputM3(:,index_IBR.x)'];
       plansy3 = [plansy3; outputM3(:,index_IBR.y)'];
       planss3 = [planss3; outputM3(:,index_IBR.s)'];
       [tx3,ty3]=casadiDynamicBSPLINE(outputM3(end,index_IBR.s),nextSplinePoints3);
       targets3 = [targets3;tx3,ty3];
    end
    cost1(i)=info.pobj;
    cost2(i)=info2.pobj;
    cost3(i)=info3.pobj;
    %costS(i)=costS;
    Progress1(i)=outputM(1,index_IBR.s);
    Progress2(i)=outputM2(1,index_IBR.s);
    Progress3(i)=outputM3(1,index_IBR.s);
    % check
    Pos3=[outputM3(2:end,index_IBR.x),outputM3(2:end,index_IBR.y)];
    
    distanceX=xs(1)-xs2(1);
    distanceY=xs(2)-xs2(2);
    distanceX2=xs(1)-xs3(1);
    distanceY2=xs(2)-xs3(2);
    distanceX3=xs2(1)-xs3(1);
    distanceY3=xs2(2)-xs3(2);
   
    squared_distance_array    = sqrt(distanceX.^2 + distanceY.^2);
    squared_distance_array2   = sqrt(distanceX2.^2 + distanceY2.^2);
    squared_distance_array3   = sqrt(distanceX3.^2 + distanceY3.^2);
    
    if squared_distance_array<=dist
        squared_distance_array
    end
    if squared_distance_array2<=dist
        squared_distance_array2
    end
    if squared_distance_array3<=dist
        squared_distance_array3
    end
end
%[t,ab,dotbeta,x,y,theta,v,beta,s]
if tend==1
        figure(4)
%         plot(pstart(1),pstart(2)-dis,'b*','Linewidth',1)
         hold on
%         plot(pstart2(1)+dis,pstart2(2),'r*','Linewidth',1) 
%         plot(pstart3(1)-dis,pstart3(2),'g*','Linewidth',1) 

        %grid on
        title('Trajectory')
        set(gca,'visible','off')
%         xlabel('X')
%         ylabel('Y')
        %axis equal
        
        figure(5)
        hold on
        grid on
        title('steer')
        ylabel('')

        figure(6)
        hold on
        grid on
        title('Speed')

        figure(4)
        I=imread('strada1.png');
        h=image([20 80],[20 80],I);
        maxxacc=max(abs(outputM(:,index_IBR.ab)));
        maxxacc2=max(abs(outputM2(:,index_IBR.ab)));
        maxxacc3=max(abs(outputM3(:,index_IBR.ab)));
        hold on
        
        for ii=1:length(outputM(1:60,index_IBR.x))-1
            vc = outputM(ii,index_IBR.ab)/maxxacc;
            vc2 = outputM2(ii,index_IBR.ab)/maxxacc2;
            vc3 = outputM3(ii,index_IBR.ab)/maxxacc3;
            next = ii+1;
            x = [outputM(ii,index_IBR.x),outputM(next,index_IBR.x)];
            y = [outputM(ii,index_IBR.y),outputM(next,index_IBR.y)];
            x2 = [outputM2(ii,index_IBR.x),outputM2(next,index_IBR.x)];
            y2 = [outputM2(ii,index_IBR.y),outputM2(next,index_IBR.y)];
            x3 = [outputM3(ii,index_IBR.x),outputM3(next,index_IBR.x)];
            y3 = [outputM3(ii,index_IBR.y),outputM3(next,index_IBR.y)];
            line(x,y,'Color',[0,0,0.5+0.5*vc],'Linewidth',2)
            line(x2,y2,'Color',[0.5+0.5*vc2,0,0],'Linewidth',2)
            line(x3,y3,'Color',[0,0.5+0.5*vc3,0],'Linewidth',2)
        end
        axis equal
        CP=0:0.01:2*pi;
        gklx = 1.5*cos(CP);
        gkly = 1.5*sin(CP);
        gklp = [gklx;gkly];
%         [leftline,middleline,rightline] = drawTrack(points(:,1:2),points(:,3));
%         [leftline2,middleline2,rightline2] = drawTrack(points2(:,1:2),points2(:,3));
%         hold on
%         plot(leftline(:,1),leftline(:,2),'k')
%         plot(middleline(:,1),middleline(:,2),'k--')
%         plot(rightline(:,1),rightline(:,2),'k')
%         plot(leftline2(:,1),leftline2(:,2),'k')
%         plot(middleline2(:,1),middleline2(:,2),'k--')
%         plot(rightline2(:,1),rightline2(:,2),'k')
%         plot([10,80],[pstarty-3.5,pstarty-3.5],'--k','Linewidth',1)
%         plot([10,80],[pstarty+3.5,pstarty+3.5],'--k','Linewidth',1)
%         plot([pstartx2-3.5,pstartx2-3.5],[20,80],'--k','Linewidth',1)
%         plot([pstartx2+3.5,pstartx2+3.5],[20,80],'--k','Linewidth',1)
        idx=[1,30,59];
        for jjj=1:length(idx)
            iff= idx(jjj);
            theta = atan2(outputM(iff+1,index_IBR.y)-outputM(iff,index_IBR.y),outputM(iff+1,index_IBR.x)-outputM(iff,index_IBR.x)); % to rotate 90 counterclockwise
            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            rgklp = [outputM(iff+1,index_IBR.x);outputM(iff+1,index_IBR.y)]+R*gklp;
            fill(rgklp(1,:),rgklp(2,:),'b');
        %     
            theta2 = atan2(outputM2(iff+1,index_IBR.y)-outputM2(iff,index_IBR.y),outputM2(iff+1,index_IBR.x)-outputM2(iff,index_IBR.x)); % to rotate 90 counterclockwise
            R = [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)];
            rgklp = [outputM2(iff+1,index_IBR.x);outputM2(iff+1,index_IBR.y)]+R*gklp;
            fill(rgklp(1,:),rgklp(2,:),'r');
        %     
            theta3 = atan2(outputM3(iff+1,index_IBR.y)-outputM3(iff,index_IBR.y),outputM3(iff+1,index_IBR.x)-outputM3(iff,index_IBR.x)); % to rotate 90 counterclockwise
            R = [cos(theta3) -sin(theta3); sin(theta3) cos(theta3)];
            rgklp = [outputM3(iff+1,index_IBR.x);outputM3(iff+1,index_IBR.y)]+R*gklp;
            fill(rgklp(1,:),rgklp(2,:),'g');
            
        end
        %legend ('Trajectory 1','T 2','T 3','Vehicle 1','V 2','V 3')
        set(gca,'FontSize',12)
        savefig('figures/3v_IBR_intersection_gbr')
        saveas(gcf,'figures/3v_IBR_intersection_gbr','epsc')
        figure(5)
        plot(0.1:0.1:length(outputM(:,1))*0.1,outputM(:,index_IBR.theta),'b.-','Linewidth',1)
        plot(0.1:0.1:length(outputM(:,1))*0.1,outputM2(:,index_IBR.theta),'r.-','Linewidth',1)
        plot(0.1:0.1:length(outputM(:,1))*0.1,outputM3(:,index_IBR.theta),'g.-','Linewidth',1)
        legend ('Vehicle 1','V 2','V 3')
        xlabel('Prediction horizon [s]')
        set(gca,'FontSize',12)
        figure(6)
        plot(0.1:0.1:length(outputM(:,1))*0.1,outputM(:,index_IBR.v),'b.-','Linewidth',1)
        plot(0.1:0.1:length(outputM(:,1))*0.1,outputM2(:,index_IBR.v),'r.-','Linewidth',1)
        plot(0.1:0.1:length(outputM(:,1))*0.1,outputM3(:,index_IBR.v),'g.-','Linewidth',1)
        legend ('Vehicle 1','V 2','V 3')
        xlabel('Prediction horizon [s]')
        ylabel('speed [m/s]')
        set(gca,'FontSize',12)
        savefig('figures/3v_IBR_speed_gbr')
        saveas(gcf,'figures/3v_IBR_speed_gbr','epsc')
        figure(7)
        hold on
        plot(cost1,'b*')
        plot(cost2,'r*')
        plot(cost3,'g*')
        plot(cost1+cost2+cost3,'c*')
        legend ('Vehicle 1','V 2','V 3','Tot')
        drawAnimation_P3_PH_IBR
else
    drawIBR3
    figure
    hold on
    plot(cost1,'b')
    plot(cost2,'r')
    plot(cost3,'g')
    plot(cost1+cost2+cost3,'c')
    legend ('Vehicle 1','V 2','V 3','Tot')
end


