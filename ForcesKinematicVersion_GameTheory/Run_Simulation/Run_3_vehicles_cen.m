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
%parameters_3_vehicles

%% Initialization for simulation
global index
%indexes_3_vehicles

dis=1.5;
%% Initialization for simulation
fpoints = points(1:2,1:2);
pdir = diff(fpoints);
[pstartx,pstarty] = casadiDynamicBSPLINE(0.01,points);
pstart = [pstartx,pstarty];
pangle = atan2(pdir(2),pdir(1));
xs(index.x-index.nu)=pstart(1);
xs(index.y-index.nu)=pstart(2)-dis;
xs(index.theta-index.nu)=pangle;
xs(index.v-index.nu)=8;
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
xs(index.x_k2-index.nu)=pstart2(1)+dis;
xs(index.y_k2-index.nu)=pstart2(2);
xs(index.theta_k2-index.nu)=pangle2;
xs(index.v_k2-index.nu)=8;
xs(index.ab_k2-index.nu)=0;
xs(index.beta_k2-index.nu)=0;
xs(index.s_k2-index.nu)=0.01;
plansx2 = [];
plansy2 = [];
planss2 = [];
targets2 = [];

%Go-kart 3 initialization
fpoints3 = points3(1:2,1:2);
pdir3 = diff(fpoints3);
[pstartx3,pstarty3] = casadiDynamicBSPLINE(0.01,points3);
pstart3 = [pstartx3,pstarty3];
pangle3 = atan2(pdir3(2),pdir3(1));
xs(index.x_k3-index.nu)=pstart3(1)-dis;
xs(index.y_k3-index.nu)=pstart3(2);
xs(index.theta_k3-index.nu)=pangle3;
xs(index.v_k3-index.nu)=8;
xs(index.ab_k3-index.nu)=0;
xs(index.beta_k3-index.nu)=0;
xs(index.s_k3-index.nu)=0.01;
plansx3 = [];
plansy3 = [];
planss3 = [];
targets3 = [];

planc = 10;
tstart = 1;
x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';
optA = 0;
optB = 0;
optC = 0;
opt =  0;
regABA=0;
regABB=0;
regABC=0;
regBetaA=0;
regBetaB=0;
regBetaC=0;
latcostA=0;
latcostB=0;
latcostC=0;
speedcostA=0;
speedcostB=0;
speedcostC=0;
%% Simulation
history = zeros(tend*eulersteps,model.nvar+1);
splinepointhist = zeros(tend,pointsN*3+pointsN2*3+pointsN3*3+1);
a=0;
for i =1:tend
    tstart = i;
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
        if xs(index.s_k3-index.nu)>1
            %nextSplinePoints_k2;
            %spline step forward
            splinestart3 = splinestart3+1;
            xs(index.s_k3-index.nu)=xs(index.s_k3-index.nu)-1;
        end
    end
    % go kart 1
    xs(index.ab-index.nu)=min(casadiGetMaxAcc(xs(index.v-index.nu))-0.0001,xs(index.ab-index.nu));
    % go kart 2
    xs(index.ab_k2-index.nu)=min(casadiGetMaxAcc(xs(index.v_k2-index.nu))-0.0001,xs(index.ab_k2-index.nu));
    % go kart 3
    xs(index.ab_k3-index.nu)=min(casadiGetMaxAcc(xs(index.v_k3-index.nu))-0.0001,xs(index.ab_k3-index.nu));
    problem.xinit = xs';
    
    ip = splinestart;
    ip2 = splinestart2;
    ip3 = splinestart3;
    [nkp, ~] = size(points);
    [nkp2, ~] = size(points2);
    [nkp3, ~] = size(points3);
    nextSplinePoints = zeros(pointsN,3);
    nextSplinePoints_k2 = zeros(pointsN2,3);
    nextSplinePoints_k3 = zeros(pointsN3,3);
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
    for jj=1:pointsN3
       while ip3>nkp3
            ip3 = ip3 -nkp3;
       end
       nextSplinePoints_k3(jj,:)=points3(ip3,:);
       ip3 = ip3 + 1;
    end
    splinepointhist(i,:)=[xs(index.s-index.nu),[nextSplinePoints(:);nextSplinePoints_k2(:);nextSplinePoints_k3(:)]'];
    
    % parameters
    problem.all_parameters = repmat(getParameters_PG3(maxSpeed,maxxacc,...
        maxyacc,latacclim,rotacceffect,torqueveceffect,brakeeffect,...
        plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,nextSplinePoints,nextSplinePoints_k2,nextSplinePoints_k3), model.N ,1);
    problem.x0 = x0(:);       
    % solve mpc
    [output,exitflag,info] = MPCPathFollowing_3v(problem);
    solvetimes(end+1)=info.solvetime;
    if(exitflag==0)
       a = a + 1; 
    end
    if(exitflag~=1 && exitflag ~=0)
        draw3
        keyboard
    end
    %nextSplinePoints
    %get output
    outputM = reshape(output.alldata,[model.nvar,model.N])';
    x0 = outputM';
    u = repmat(outputM(1,1:index.nu),eulersteps,1);
    [xhist,time] = euler(@(x,u)interstagedx_PG3(x,u),xs,u,integrator_stepsize/eulersteps);
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
        plansx3 = [plansx3; outputM(:,index.x_k3)'];
        plansy3 = [plansy3; outputM(:,index.y_k3)'];
        planss3 = [planss3; outputM(:,index.s_k3)'];
        [tx,ty]=casadiDynamicBSPLINE(outputM(end,index.s),nextSplinePoints);
        targets = [targets;tx,ty];
        [tx2,ty2]=casadiDynamicBSPLINE(outputM(end,index.s_k2),nextSplinePoints_k2);
        targets2 = [targets2;tx2,ty2];
        [tx3,ty3]=casadiDynamicBSPLINE(outputM(end,index.s_k3),nextSplinePoints_k3);
        targets3 = [targets3;tx3,ty3];
    end      
    if tend==1
    %% Evaluation cost function
    for jj=1:length(outputM)
        [lagcost,latcost,regAB,regBeta,latcost1,slack,speedcost,lagcost_k2,...
         latcost_k2,regAB_k2,regBeta_k2,latcost1_k2,slack_k2,speedcost_k2,...
         latcost_k3,regAB_k3,regBeta_k3,latcost1_k3,slack_k3,speedcost_k3,f,f1,f2,f3] =...
         objective_PG_Test3(outputM(jj,:),points,points2,points3,maxSpeed,plagerror,...
         platerror, pprog, pab, pdotbeta, pspeedcost,pslack,pslack2);
        
        regABA=regABA+regAB;
        regABB=regABB+regAB_k2;
        regABC=regABC+regAB_k3;
        regBetaA=regBetaA+regBeta;
        regBetaB=regBetaB+regBeta_k2;
        regBetaC=regBetaC+regBeta_k3;
        latcostA=latcostA+latcost+latcost1;
        latcostB=latcostA+latcost_k2+latcost1_k2;
        latcostC=latcostC+latcost_k3+latcost1_k3;
        speedcostA=speedcostA+speedcost;
        speedcostB=speedcostB+speedcost_k2;
        speedcostC=speedcostC+speedcost_k3;
        optA = optA+ f1;
        optB = optB+ f2;
        optC = optC+ f3;
        opt  = opt+ f;
    end
    end
    Percentage=i/tend*100
end

if tend==1
    figure(2)
%         plot(pstart(1),pstart(2)-dis,'b*','Linewidth',1)
        hold on
%         plot(pstart2(1)+dis,pstart2(2),'r*','Linewidth',1) 
%         plot(pstart3(1)-dis,pstart3(2),'g*','Linewidth',1) 
%         [leftline,middleline,rightline] = drawTrack(points(:,1:2),points(:,3));
%         [leftline2,middleline2,rightline2] = drawTrack(points2(:,1:2),points2(:,3));
%         hold on
%         plot(leftline(:,1),leftline(:,2),'k')
%         plot(middleline(:,1),middleline(:,2),'k-.')
%         plot(rightline(:,1),rightline(:,2),'k')
%         plot(leftline2(:,1),leftline2(:,2),'k')
%         plot(middleline2(:,1),middleline2(:,2),'k-.')
% %         plot(rightline2(:,1),rightline2(:,2),'k')
%         CP=0:0.01:2*pi;
%         gklx = 1.5*cos(CP);
%         gkly = 1.5*sin(CP);
%         gklp = [gklx;gkly];
%        I=imread('strada1.png');
%        h=image([20 80],[20 80],I);
%        grid on
        I=imread('strada1.png');
        h=image([20 80],[80 20],I);
        title('Trajectory')
%         xlabel('X')
%         ylabel('Y')

%         figure(2)
%         hold on
%         grid on
%         title('steer')
%         xlabel('step')
%         ylabel('')

        figure(3)
        hold on
        grid on
        xlabel('Time [s]')
        ylabel('speed [m/s]')

        figure(2)

        maxxacc=max(abs(outputM(:,index.ab)));
        maxxacc2=max(abs(outputM(:,index.ab_k2)));
        maxxacc3=max(abs(outputM(:,index.ab_k3)));
        hold on

        for ii=1:length(outputM(1:P_H_length,index.x))-1
            vc = outputM(ii,index.ab)/maxxacc;
            vc2 = outputM(ii,index.ab_k2)/maxxacc2;
            vc3 = outputM(ii,index.ab_k3)/maxxacc3;
            next = ii+1;
            x = [outputM(ii,index.x),outputM(next,index.x)];
            y = [outputM(ii,index.y),outputM(next,index.y)];
            x2 = [outputM(ii,index.x_k2),outputM(next,index.x_k2)];
            y2 = [outputM(ii,index.y_k2),outputM(next,index.y_k2)];
            x3 = [outputM(ii,index.x_k3),outputM(next,index.x_k3)];
            y3 = [outputM(ii,index.y_k3),outputM(next,index.y_k3)];
            line(x,y,'Color',[0,0,0.5+0.5*vc],'Linewidth',2)
            line(x2,y2,'Color',[0.5+0.5*vc2,0,0],'Linewidth',2)
            line(x3,y3,'Color',[0,0.5+0.5*vc3,0],'Linewidth',2)
        end
        set(gca,'visible','off')
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
        idx=[1,P_H_length/2,P_H_length-1];
        for jjj=1:length(idx)
            iff= idx(jjj);
            theta = atan2(outputM(iff+1,index.y)-outputM(iff,index.y),outputM(iff+1,index.x)-outputM(iff,index.x)); % to rotate 90 counterclockwise
            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            rgklp = [outputM(iff+1,index.x);outputM(iff+1,index.y)]+R*gklp;
            fill(rgklp(1,:),rgklp(2,:),'b');
        %     
            theta2 = atan2(outputM(iff+1,index.y_k2)-outputM(iff,index.y_k2),outputM(iff+1,index.x_k2)-outputM(iff,index.x_k2)); % to rotate 90 counterclockwise
            R = [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)];
            rgklp = [outputM(iff+1,index.x_k2);outputM(iff+1,index.y_k2)]+R*gklp;
            fill(rgklp(1,:),rgklp(2,:),'r');
        %     
            theta3 = atan2(outputM(iff+1,index.y_k3)-outputM(iff,index.y_k3),outputM(iff+1,index.x_k3)-outputM(iff,index.x_k3)); % to rotate 90 counterclockwise
            R = [cos(theta3) -sin(theta3); sin(theta3) cos(theta3)];
            rgklp = [outputM(iff+1,index.x_k3);outputM(iff+1,index.y_k3)]+R*gklp;
            fill(rgklp(1,:),rgklp(2,:),'g');
        end
        axis equal
        savefig('figures/3v_PG_intersection')
        saveas(gcf,'figures/3v_PG_intersection','epsc')
        int=integrator_stepsize;
%         figure(2)
%         plot(int:int:length(outputM(:,1))*int,outputM(:,index.theta),'b.-','Linewidth',1)
%         plot(int:int:length(outputM(:,1))*int,outputM(:,index.theta_k2),'r.-','Linewidth',1)
%         plot(int:int:length(outputM(:,1))*int,outputM(:,index.theta_k3),'g.-','Linewidth',1)
%         legend('Kart1','Kart2','Kart3')
        figure(3)
        plot(int:int:length(outputM(:,1))*int,outputM(:,index.v),'b.-','Linewidth',1)
        plot(int:int:length(outputM(:,1))*int,outputM(:,index.v_k2),'r.-','Linewidth',1)
        plot(int:int:length(outputM(:,1))*int,outputM(:,index.v_k3),'g.-','Linewidth',1)
        legend('Vehicle 1','V 2','V 3')
        set(gca,'FontSize',15)
        savefig('figures/3v_PG_speed')
        saveas(gcf,'figures/3v_PG_speed','epsc')
        % drawAnimation_P3_PH
        figure(1)
        hold on
        plot(optA,'b*','Linewidth',2)
        plot(optB,'r*','Linewidth',2)
        plot(optC,'g*','Linewidth',2)
        plot(opt/3,'c*','Linewidth',2)
        set(gca,'Fontsize',15)
else
    draw3
end