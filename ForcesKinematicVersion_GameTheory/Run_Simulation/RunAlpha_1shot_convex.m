global index index_IBR
close all
dis=1.5;
%% PG
% Initialization for simulation kart 1
fpoints = points(1:2,1:2);
pdir = diff(fpoints);
[pstartx,pstarty] = casadiDynamicBSPLINE(0.01,points);
pstart = [pstartx,pstarty];
pangle = atan2(pdir(2),pdir(1));
xs(index.x-index.nu)=pstart(1);
xs(index.y-index.nu)=pstart(2)-dis;
xs(index.theta-index.nu)=pangle;
xs(index.v-index.nu)=vel1;
xs(index.ab-index.nu)=0;
xs(index.beta-index.nu)=0;
xs(index.s-index.nu)=0.01;

%Go-kart 2 initialization
fpoints2 = points2(1:2,1:2);
pdir2 = diff(fpoints2);
[pstartx2,pstarty2] = casadiDynamicBSPLINE(0.01,points2);
pstart2 = [pstartx2,pstarty2];
pangle2 = atan2(pdir2(2),pdir2(1));
xs(index.x_k2-index.nu)=pstart2(1)+dis;
xs(index.y_k2-index.nu)=pstart2(2);
xs(index.theta_k2-index.nu)=pangle2;
xs(index.v_k2-index.nu)=vel2;
xs(index.ab_k2-index.nu)=0;
xs(index.beta_k2-index.nu)=0;
xs(index.s_k2-index.nu)=0.01;
x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';
% 
% %% Initialization for simulation
% fpoints = points(1:2,1:2);
% pdir = diff(fpoints);
% [pstartx,pstarty] = casadiDynamicBSPLINE(0.01,points);
% pstart = [pstartx,pstarty];
% pangle = atan2(pdir(2),pdir(1));
% xs(index.x-index.nu)=pstart(1);
% xs(index.y-index.nu)=pstart(2)-dis;
% xs(index.theta-index.nu)=pangle;
% xs(index.v-index.nu)=6;
% xs(index.ab-index.nu)=0;
% xs(index.beta-index.nu)=0;
% xs(index.s-index.nu)=0.01;
% plansx = [];
% plansy = [];
% planss = [];
% targets = [];
% %Go-kart 2 initialization
% fpoints2 = points2(1:2,1:2);
% pdir2 = diff(fpoints2);
% [pstartx2,pstarty2] = casadiDynamicBSPLINE(0.01,points2);
% pstart2 = [pstartx2,pstarty2];
% pangle2 = atan2(pdir2(2),pdir2(1));
% xs(index.x_k2-index.nu)=pstart2(1)+dis;
% xs(index.y_k2-index.nu)=pstart2(2);
% xs(index.theta_k2-index.nu)=pangle2;
% xs(index.v_k2-index.nu)=6;
% xs(index.ab_k2-index.nu)=0;
% xs(index.beta_k2-index.nu)=0;
% xs(index.s_k2-index.nu)=0.01;
% plansx2 = [];
% plansy2 = [];
% planss2 = [];
% targets2 = [];

%% IBR
% V1
xs_IBR(index_IBR.x-index_IBR.nu)=pstart(1);
xs_IBR(index_IBR.y-index_IBR.nu)=pstart(2)-dis;
xs_IBR(index_IBR.theta-index_IBR.nu)=pangle;
xs_IBR(index_IBR.v-index_IBR.nu)=vel1;
xs_IBR(index_IBR.ab-index_IBR.nu)=0;
xs_IBR(index_IBR.beta-index_IBR.nu)=0;
xs_IBR(index_IBR.s-index_IBR.nu)=0.01;
%V2
xs_IBR2(index_IBR.x-index_IBR.nu)=pstart2(1)+dis;
xs_IBR2(index_IBR.y-index_IBR.nu)=pstart2(2);
xs_IBR2(index_IBR.theta-index_IBR.nu)=pangle2;
xs_IBR2(index_IBR.v-index_IBR.nu)=vel2;
xs_IBR2(index_IBR.ab-index_IBR.nu)=0;
xs_IBR2(index_IBR.beta-index_IBR.nu)=0;
xs_IBR2(index_IBR.s-index_IBR.nu)=0.01;

x0_IBR = [zeros(model_IBR.N,index_IBR.nu),repmat(xs_IBR,model_IBR.N,1)]';
x0_IBR2 = [zeros(model_IBR.N,index_IBR.nu),repmat(xs_IBR2,model_IBR.N,1)]';

Pos1=repmat(pstart, model_IBR.N-1 ,1);
Pos2=repmat(pstart2, model_IBR.N-1 ,1);
%% Simulation
a=0;
a2=0;
alpha1=fliplr(0:0.1:1);
alpha2=1-alpha1;
optA = zeros(1,length(alpha1));
optB = zeros(1,length(alpha1));
opt = zeros(1,length(alpha1));
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

problem.xinit = [xs(1:index.ns/2),xs(index.ns/2+1:end)]';

%IBR
xs_IBR(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs_IBR(index_IBR.v-index_IBR.nu))...
        -0.0001,xs_IBR(index_IBR.ab-index_IBR.nu));
problem_IBR.xinit = xs_IBR';
    
xs_IBR2(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs_IBR2(index_IBR.v-index_IBR.nu))...
        -0.0001,xs_IBR2(index_IBR.ab-index_IBR.nu));
problem_IBR2.xinit = xs_IBR2';

problem.x0 = x0(:);   
problem_IBR.x0_IBR = x0_IBR(:);
problem_IBR2.x0_IBR = x0_IBR2(:);
tstart = 1;
tstart2 = 1;
for kk=1:length(alpha1)
    
    if kk==1
        figure(1)
        plot(pstart(1),pstart(2)-dis,'g*')
        hold on
        plot(pstart2(1)+dis,pstart2(2),'g*') 
        [leftline,middleline,rightline] = drawTrack(points(:,1:2),points(:,3));
        [leftline2,middleline2,rightline2] = drawTrack(points2(:,1:2),points2(:,3));
        I=imread('strada1.png');
        h=image([20 80],[20 80],I);
        CP=0:0.01:2*pi;
        gklx = 1.5*cos(CP);
        gkly = 1.5*sin(CP);
        gklp = [gklx;gkly];

%         plot(leftline(:,1),leftline(:,2),'k')
%         plot(middleline(:,1),middleline(:,2),'k--')
%         plot(rightline(:,1),rightline(:,2),'k')
%         plot(leftline2(:,1),leftline2(:,2),'k')
%         plot(middleline2(:,1),middleline2(:,2),'k--')
%         plot(rightline2(:,1),rightline2(:,2),'k')
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

    if alpha1(kk)==1
        % parameters
        problem_IBR.all_parameters = repmat (getParameters_IBR(maxSpeed,...
            maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
            brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
            pspeedcost,pslack,pslack2,dist,xs_IBR2(1),xs_IBR2(2),nextSplinePoints),...
            model_IBR.N ,1);
        problem_IBR.all_parameters(index_IBR.xComp:model_IBR.npar:end)=[Pos2(:,1);...
                                                            Pos2(end,1)];
        problem_IBR.all_parameters(index_IBR.yComp:model_IBR.npar:end)=[Pos2(:,2);...
                                                            Pos2(end,2)];
        problem_IBR.x0 = x0_IBR(:);
        %go kart 1
        [output,exitflag,info] = MPCPathFollowing_2v_IBR(problem_IBR);
        if(exitflag==0)
            a =a+ 1;
        end
        outputM = reshape(output.alldata,[model_IBR.nvar,model_IBR.N])';
        %x0_IBR=outputM';

        problem_IBR2.all_parameters = repmat (getParameters_IBR(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs_IBR(1),xs_IBR(2),nextSplinePoints_k2),...
        model_IBR.N ,1);

        problem_IBR2.all_parameters(index_IBR.xComp:model_IBR.npar:end)=...
            outputM(:,index_IBR.x);
        problem_IBR2.all_parameters(index_IBR.yComp:model_IBR.npar:end)=...
            outputM(:,index_IBR.y);
        problem_IBR2.x0 = x0_IBR2(:);

        %go kart 2
        [output2,exitflag2,info2] = MPCPathFollowing_2v_IBR(problem_IBR2);
        if(exitflag2==0)
            a2 =a2+ 1;
        end
        outputM2 = reshape(output2.alldata,[model_IBR.nvar,model_IBR.N])';
        %x0_IBR2=outputM2';
        cost1(kk)=info.pobj;
        optA(kk)=info.pobj;
        cost2(kk)=info2.pobj;
        optB(kk)=info2.pobj;

        opt(kk)=optA(kk)+optB(kk);
        Progress1(kk)=outputM(1,index_IBR.s);
        Progress2(kk)=outputM2(1,index_IBR.s);
        iff=1;
        figure(1)
        plot(outputM(:,index_IBR.x),outputM(:,index_IBR.y),'.-','Color',[0,alpha1(kk),1])
        plot(outputM2(:,index_IBR.x),outputM2(:,index_IBR.y),'.-','Color',[1,1-alpha1(kk),0])
        theta = atan2(outputM(iff+1,index_IBR.y)-outputM(iff,index_IBR.y),outputM(iff+1,index_IBR.x)-outputM(iff,index_IBR.x)); % to rotate 90 counterclockwise
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        rgklp = [outputM(iff+1,index_IBR.x);outputM(iff+1,index_IBR.y)]+R*gklp;
        fill(rgklp(1,:),rgklp(2,:),'b');
    %     
        theta2 = atan2(outputM2(iff+1,index_IBR.y)-outputM2(iff,index_IBR.y),outputM2(iff+1,index_IBR.x)-outputM2(iff,index_IBR.x)); % to rotate 90 counterclockwise
        R = [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)];
        rgklp = [outputM2(iff+1,index_IBR.x);outputM2(iff+1,index_IBR.y)]+R*gklp;
        fill(rgklp(1,:),rgklp(2,:),'r');
        figure(2)
        plot(outputM(:,index_IBR.theta),'.-','Color',[0,alpha1(kk),1])
        plot(outputM2(:,index_IBR.theta),'.-','Color',[1,1-alpha1(kk),0])

        figure(3)
        plot(outputM(:,index_IBR.v),'.-','Color',[0,alpha1(kk),1])
        plot(outputM2(:,index_IBR.v),'.-','Color',[1,1-alpha1(kk),0])
        x0_IBR = [zeros(model_IBR.N,index_IBR.nu),repmat(xs_IBR,model_IBR.N,1)]';
        x0_IBR2 = [zeros(model_IBR.N,index_IBR.nu),repmat(xs_IBR2,model_IBR.N,1)]';
    elseif alpha1(kk)==0
        problem_IBR2.all_parameters = repmat (getParameters_IBR(maxSpeed,...
            maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
            brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
            pspeedcost,pslack,pslack2,dist,xs_IBR(1),xs_IBR(2),nextSplinePoints_k2),...
            model_IBR.N ,1);

        problem_IBR2.all_parameters(index_IBR.xComp:model_IBR.npar:end)=[Pos1(:,1);...
                                                             Pos1(end,1)];
        problem_IBR2.all_parameters(index_IBR.yComp:model_IBR.npar:end)=[Pos1(:,2);...
                                                             Pos2(end,2)];
        problem_IBR2.x0 = x0_IBR2(:);

        %go kart 1
        [output2,exitflag2,info2] = MPCPathFollowing_2v_IBR(problem_IBR2);
        if(exitflag2==0)
            a2 =a2+ 1;
        end
        outputM2 = reshape(output2.alldata,[model_IBR.nvar,model_IBR.N])';
        %x0_IBR2=outputM2';
        problem_IBR.all_parameters = repmat (getParameters_IBR(maxSpeed,...
        maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
        brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
        pspeedcost,pslack,pslack2,dist,xs_IBR2(1),xs_IBR2(2),nextSplinePoints),...
        model_IBR.N ,1);

        problem_IBR.all_parameters(index_IBR.xComp:model_IBR.npar:end)=...
            outputM2(:,index_IBR.x);
        problem_IBR.all_parameters(index_IBR.yComp:model_IBR.npar:end)=...
            outputM2(:,index_IBR.y);
        problem_IBR.x0 = x0_IBR(:);
        %go kart 2
        [output,exitflag,info] = MPCPathFollowing_2v_IBR(problem_IBR);
        if(exitflag==0)
            a =a+ 1;
        end
        outputM = reshape(output.alldata,[model_IBR.nvar,model_IBR.N])';
        %x0_IBR=outputM';
        cost1(kk)=info.pobj;
        optA(kk)=info.pobj;
        cost2(kk)=info2.pobj;
        optB(kk)=info2.pobj;
        opt(kk)=optA(kk)+optB(kk);
        Progress1(kk)=outputM(1,index_IBR.s);
        Progress2(kk)=outputM2(1,index_IBR.s);
        figure(1)
        plot(outputM(:,index_IBR.x),outputM(:,index_IBR.y),'.-','Color',[0,alpha1(kk),1])
        plot(outputM2(:,index_IBR.x),outputM2(:,index_IBR.y),'.-','Color',[1,1-alpha1(kk),0])

        figure(2)
        plot(outputM(:,index_IBR.theta),'.-','Color',[0,alpha1(kk),1])
        plot(outputM2(:,index_IBR.theta),'.-','Color',[1,1-alpha1(kk),0])

        figure(3)
        plot(outputM(:,index_IBR.v),'.-','Color',[0,alpha1(kk),1])
        plot(outputM2(:,index_IBR.v),'.-','Color',[1,1-alpha1(kk),0])
        x0_IBR = [zeros(model_IBR.N,index_IBR.nu),repmat(xs_IBR,model_IBR.N,1)]';
        x0_IBR2 = [zeros(model_IBR.N,index_IBR.nu),repmat(xs_IBR2,model_IBR.N,1)]';
    else
        % parameters
        problem.all_parameters = repmat(getParameters_PG_alpha(maxSpeed,...
            maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
            brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
            pspeedcost,pslack,pslack2,dist,alpha1(kk),alpha2(kk),nextSplinePoints,...
            nextSplinePoints_k2), model.N ,1);
        problem.x0=x0(:);
        % solve mpc
        [output,exitflag,info] = MPCPathFollowing_2v_alpha(problem);
        outputM = reshape(output.alldata,[model.nvar,model.N])';
        %% Evaluation cost function
        for jj=1:length(outputM)
            [lagcost_A,latcost_A,reg_A,prog_A,slack_A,speedcost_A,lagcost_A_k2,...
            latcost_A_k2,reg_A_k2,prog_A_k2,slack_A_k2,speedcost_A_k2,f,f1,f2] =...
            objective_PG_TestAlpha(outputM(jj,:),nextSplinePoints,nextSplinePoints_k2,...
            maxSpeed,plagerror, platerror, pprog, pab, pdotbeta,...
            pspeedcost,pslack,pslack2,alpha1(kk),alpha2(kk));

            optA(kk)= optA(kk)+ f1;
            optB(kk)= optB(kk)+ f2;
            opt(kk) = opt(kk)+ 2*f;

        end

        cost1(kk)=info.pobj;

        Progress1(kk)=outputM(1,index.s);

        figure(1)
        plot(outputM(:,index.x),outputM(:,index.y),'.-','Color',[0,alpha1(kk),1])
        plot(outputM(:,index.x_k2),outputM(:,index.y_k2),'.-','Color',[1,1-alpha1(kk),0])
        axis equal
        figure(2)
        plot(outputM(:,index.theta),'.-','Color',[0,alpha1(kk),1])
        plot(outputM(:,index.theta_k2),'.-','Color',[1,1-alpha1(kk),0])

        figure(3)
        plot(outputM(:,index.v),'.-','Color',[0,alpha1(kk),1])
        plot(outputM(:,index.v_k2),'.-','Color',[1,1-alpha1(kk),0])
        x0 = [zeros(model.N,index.nu),repmat(xs,model.N,1)]';
    end
end

figure(4)
for kk=1:length(alpha1)
    plot(alpha1(kk),optA(kk),'*','Color',[0,alpha1(kk),1])
    plot(alpha1(kk),optB(kk),'*','Color',[1,1-alpha1(kk),0])
    plot(alpha1(kk),(optA(kk)+optB(kk))/2,'o','Color',[0,1,0])
end
legend('J1_{down}','J2_{left}','Tot/2')
xticklabels({'0,1','0.1,0.9','0.2,0.8','0.3,0.7','0.4,0.6',...
    '0.5,0.5','0.6,0.4','0.7,0.3','0.8,0.2',...
    '0.9,0.1','1,0'});

figure(5)
for kk=1:length(alpha1)
    plot(alpha1(kk),optA(kk)/(optA(kk)+optB(kk))*100,'*','Color',[0,alpha1(kk),1])
    plot(alpha1(kk),optB(kk)/(optA(kk)+optB(kk))*100,'*','Color',[1,1-alpha1(kk),0])
end
legend('V1_{down}','V2_{left}')
xticklabels({'0,1','0.1,0.9','0.2,0.8','0.3,0.7','0.4,0.6',...
    '0.5,0.5','0.6,0.4','0.7,0.3','0.8,0.2',...
    '0.9,0.1','1,0'});