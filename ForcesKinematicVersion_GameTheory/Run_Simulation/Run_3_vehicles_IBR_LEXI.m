%% Initialization for simulation
global index_IBR
Plotta=1;
Plotta1=0;
dis=1.75;

%% Simulation
history = zeros(tend*eulersteps,model.nvar+1);
splinepointhist = zeros(tend,pointsN*3+1);
plansx = [];
plansy = [];
planss = [];
targets = [];
planc = 10;

%% k2
history2 = zeros(tend*eulersteps,model.nvar+1);
splinepointhist2 = zeros(tend,pointsN*3+1);
plansx2 = [];
plansy2 = [];
planss2 = [];
targets2 = [];
planc2 = 10;

%x02 = [zeros(model.N,index_IBR.nu),repmat(xs2,model.N,1)]';
tstart = 1;
%% k2
history3 = zeros(tend*eulersteps,model.nvar+1);
splinepointhist3 = zeros(tend,pointsN*3+1);
plansx3 = [];
plansy3 = [];
planss3 = [];
targets3 = [];
planc3 = 10;

%x03 = [zeros(model.N,index_IBR.nu),repmat(xs3,model.N,1)]';
%% Communal
a=0;
a2=0;
a3=0;
IND=[];
IND2=[];
IND3=[];
config=[1,2,3;1,3,2;];%2,1,3;2,3,1;3,1,2;3,2,1
cost1 = zeros(length(config(:,1)),tend);
cost2 = zeros(length(config(:,1)),tend);
cost3 = zeros(length(config(:,1)),tend);
Progress1 = zeros(length(config(:,1)),tend);
Progress2 = zeros(length(config(:,1)),tend);
Progress3 = zeros(length(config(:,1)),tend);
Steer1 = zeros(length(config(:,1)),tend);
Steer2 = zeros(length(config(:,1)),tend);
Steer3 = zeros(length(config(:,1)),tend);
Speed1 = zeros(length(config(:,1)),tend);
Speed2 = zeros(length(config(:,1)),tend);
Speed3 = zeros(length(config(:,1)),tend);
Speed1_1 = zeros(length(config(:,1)),tend);
Speed2_1 = zeros(length(config(:,1)),tend);
Speed3_1 = zeros(length(config(:,1)),tend);
Jerk1 = zeros(length(config(:,1)),tend);
Jerk2 = zeros(length(config(:,1)),tend);
Jerk3 = zeros(length(config(:,1)),tend);

for jj=1:length(config(:,1))
    %% Initialization for simulation
    fpoints = points(1:2,1:2);
    pdir = diff(fpoints);
    [pstartx,pstarty] = casadiDynamicBSPLINE(0.01,points);
    pstart = [pstartx,pstarty];
    pangle = atan2(pdir(2),pdir(1));
    xs(index_IBR.x-index_IBR.nu)=pstart(1);
    xs(index_IBR.y-index_IBR.nu)=pstart(2)+dis;
    xs(index_IBR.theta-index_IBR.nu)=pangle;
    xs(index_IBR.v-index_IBR.nu)=targetSpeed;
    xs(index_IBR.ab-index_IBR.nu)=0;
    xs(index_IBR.beta-index_IBR.nu)=0;
    xs(index_IBR.s-index_IBR.nu)=0.01;
   
    plansx = [];
    plansy = [];
    planss = [];
    targets = [];
    planc = 10;

    %x0 = [zeros(model.N,index_IBR.nu),repmat(xs,model.N,1)]';

    %% Initialization for simulation 2
    fpoints2 = points2(1:2,1:2);
    pdir2 = diff(fpoints2);
    [pstartx2,pstarty2] = casadiDynamicBSPLINE(0.01,points2);
    pstart2 = [pstartx2,pstarty2];
    pangle2 = atan2(pdir2(2),pdir2(1));
    xs2(index_IBR.x-index_IBR.nu)=pstart2(1)+dis;
    xs2(index_IBR.y-index_IBR.nu)=pstart2(2);
    xs2(index_IBR.theta-index_IBR.nu)=pangle2;
    xs2(index_IBR.v-index_IBR.nu)=targetSpeed;
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
    xs3(index_IBR.v-index_IBR.nu)=targetSpeed;
    xs3(index_IBR.ab-index_IBR.nu)=0;
    xs3(index_IBR.beta-index_IBR.nu)=0;
    xs3(index_IBR.s-index_IBR.nu)=0.01;
    
    xs(index_IBR.laterror-index_IBR.nu)=0;
    xs2(index_IBR.laterror-index_IBR.nu)=0;
    xs3(index_IBR.laterror-index_IBR.nu)=0;
    
    Pos1=repmat(pstart, model.N-1 ,1);
    Pos2=repmat(pstart2, model.N-1 ,1);
    Pos3=repmat(pstart3, model.N-1 ,1);
    x0 = [zeros(model.N,index_IBR.nu),repmat(xs,model.N,1)]';
    x02 = [zeros(model.N,index_IBR.nu),repmat(xs2,model.N,1)]';
    x03 = [zeros(model.N,index_IBR.nu),repmat(xs3,model.N,1)]';
    order=config(jj,:);
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
        problem.xinit = xs(1:7)';

        xs2(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs2(index_IBR.v-index_IBR.nu))...
            -0.0001,xs2(index_IBR.ab-index_IBR.nu));
        problem2.xinit = xs2(1:7)';

        xs3(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs3(index_IBR.v-index_IBR.nu))...
            -0.0001,xs3(index_IBR.ab-index_IBR.nu));
        problem3.xinit = xs3(1:7)';
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
        problem.all_parameters = repmat (getParameters_IBR_3(targetSpeed,...
            maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
            brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
            pspeedcost,pslack,pslack2,dist,xs2(1),xs2(2),xs3(1),xs3(2),...
            nextSplinePoints),model.N ,1);

        problem.all_parameters(index_IBR.xComp2:model.npar:end)=[Pos2(:,1)*0;...
                                                            Pos2(end,1)*0];
        problem.all_parameters(index_IBR.yComp2:model.npar:end)=[Pos2(:,2)*0;...
                                                            Pos2(end,2)*0];
        problem.all_parameters(index_IBR.xComp3:model.npar:end)=[Pos3(:,1)*0;...
                                                            Pos3(end,1)*0];
        problem.all_parameters(index_IBR.yComp3:model.npar:end)=[Pos3(:,2)*0;...
                                                            Pos3(end,2)*0];
        x0_1=x0(1:11,:);
        problem.x0 = x0_1(:);

        % parameters
        problem2.all_parameters = repmat (getParameters_IBR_3(targetSpeed,...
            maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
            brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
            pspeedcost,pslack,pslack2,dist,xs(1),xs(2),xs3(1),xs3(2),...
            nextSplinePoints2),model.N ,1);

        problem2.all_parameters(index_IBR.xComp2:model.npar:end)=[Pos1(:,1)*0;...
                                                             Pos1(end,1)*0];
        problem2.all_parameters(index_IBR.yComp2:model.npar:end)=[Pos1(:,2)*0;...
                                                             Pos1(end,2)*0];
        problem2.all_parameters(index_IBR.xComp3:model.npar:end)=[Pos3(:,1)*0;...
                                                             Pos3(end,1)*0];
        problem2.all_parameters(index_IBR.yComp3:model.npar:end)=[Pos3(:,2)*0;...
                                                             Pos3(end,2)*0];
        x02_1=x02(1:11,:);
        problem2.x0 = x02_1(:);

        % parameters
        problem3.all_parameters = repmat (getParameters_IBR_3(targetSpeed,...
            maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect,...
            brakeeffect,plagerror,platerror,pprog,pab,pdotbeta,...
            pspeedcost,pslack,pslack2,dist,xs(1),xs(2),xs2(1),xs2(2),nextSplinePoints3),...
            model.N ,1);

        problem3.all_parameters(index_IBR.xComp2:model.npar:end)=[Pos1(:,1)*0;...
                                                             Pos1(end,1)*0];
        problem3.all_parameters(index_IBR.yComp2:model.npar:end)=[Pos1(:,2)*0;...
                                                             Pos1(end,2)*0];
        problem3.all_parameters(index_IBR.xComp3:model.npar:end)=[Pos2(:,1)*0;...
                                                             Pos2(end,1)*0];
        problem3.all_parameters(index_IBR.yComp3:model.npar:end)=[Pos2(:,2)*0;...
                                                             Pos2(end,2)*0];
        
        x03_1=x03(1:11,:);
        problem3.x0 = x03_1(:);
        %go kart 1
        [output,exitflag,info] = MPCPathFollowing_3v_IBR_ORIG(problem);
        solvetimes(end+1)=info.solvetime;
        if(exitflag==0)
            a =a+ 1;
            IND=[IND;i];
        end
        if(exitflag~=1 && exitflag ~=0)
          % keyboard
        end
        outputMA = reshape(output.alldata,[model.nvar-1,model.N])';
        x0 = outputMA';
      
        %go kart 2
        [output2,exitflag2,info2] = MPCPathFollowing_3v_IBR_ORIG(problem2);
        solvetimes2(end+1)=info2.solvetime;
        if(exitflag2==0)
            a2 =a2+ 1;
            IND2=[IND2;i];
        end
        if(exitflag2~=1 && exitflag2 ~=0)
        %    keyboard           
        end

        outputMB = reshape(output2.alldata,[model.nvar-1,model.N])';
        x02=outputMB';
        %go kart 3

        [output3,exitflag3,info3] = MPCPathFollowing_3v_IBR_ORIG(problem3);
        solvetimes3(end+1)=info3.solvetime;
        if(exitflag3==0)
            a3 =a3+ 1;
            IND3=[IND3;i];
        end
        if(exitflag3~=1 && exitflag3 ~=0)
        %    keyboard           
        end

        outputMC = reshape(output3.alldata,[model.nvar-1,model.N])';
        x03 = outputMC';

        problem2.all_parameters(index_IBR.xComp2:model.npar:end)=...
            outputMA(:,index_IBR.x);
        problem2.all_parameters(index_IBR.yComp2:model.npar:end)=...
            outputMA(:,index_IBR.y);
        problem3.all_parameters(index_IBR.xComp2:model.npar:end)=...
            outputMA(:,index_IBR.x);
        problem3.all_parameters(index_IBR.yComp2:model.npar:end)=...
            outputMA(:,index_IBR.y);

        problem.all_parameters(index_IBR.xComp2:model.npar:end)=...
            outputMB(:,index_IBR.x);
        problem.all_parameters(index_IBR.yComp2:model.npar:end)=...
            outputMB(:,index_IBR.y);
        problem3.all_parameters(index_IBR.xComp3:model.npar:end)=...
            outputMB(:,index_IBR.x);
        problem3.all_parameters(index_IBR.yComp3:model.npar:end)=...
            outputMB(:,index_IBR.y);

        problem.all_parameters(index_IBR.xComp3:model.npar:end)=...
            outputMC(:,index_IBR.x);
        problem.all_parameters(index_IBR.yComp3:model.npar:end)=...
            outputMC(:,index_IBR.y);
        problem2.all_parameters(index_IBR.xComp3:model.npar:end)=...
            outputMC(:,index_IBR.x);
        problem2.all_parameters(index_IBR.yComp3:model.npar:end)=...
            outputMC(:,index_IBR.y);
        %% initialization
        if jj==1 && Plotta==1
            figure(400)
            hold on
            I=imread('road06.png');
            h=image([20 80],[80 20],I);
            set(gca,'visible','off')
%            plot(outputMA(:,index_IBR.x),outputMA(:,index_IBR.y),'Color',[0,0,1],'Linewidth',3)
%            plot(outputMB(:,index_IBR.x),outputMB(:,index_IBR.y),'Color',[1,0,0],'Linewidth',3)
%            plot(outputMC(:,index_IBR.x),outputMC(:,index_IBR.y),'Color',[0,1,0],'Linewidth',3)
            maxxacc=max(abs(outputMA(:,index_IBR.ab)));
            maxxacc2=max(abs(outputMB(:,index_IBR.ab)));
            maxxacc3=max(abs(outputMC(:,index_IBR.ab)));
            hold on

            for ii=1:length(outputMA(1:P_H_length,index_IBR.x))-1
                vc = outputMA(ii,index_IBR.ab)/maxxacc;
                vc2 = outputMB(ii,index_IBR.ab)/maxxacc2;
                vc3 = outputMC(ii,index_IBR.ab)/maxxacc3;
                next = ii+1;
                x = [outputMA(ii,index_IBR.x),outputMA(next,index_IBR.x)];
                y = [outputMA(ii,index_IBR.y),outputMA(next,index_IBR.y)];
                x2 = [outputMB(ii,index_IBR.x),outputMB(next,index_IBR.x)];
                y2 = [outputMB(ii,index_IBR.y),outputMB(next,index_IBR.y)];
                x3 = [outputMC(ii,index_IBR.x),outputMC(next,index_IBR.x)];
                y3 = [outputMC(ii,index_IBR.y),outputMC(next,index_IBR.y)];
                line(x,y,'Color',[0,0,0.5+0.5*vc],'Linewidth',3)
                line(x2,y2,'Color',[0.5+0.5*vc2,0,0],'Linewidth',3)
                line(x3,y3,'Color',[0,0.5+0.5*vc3,0],'Linewidth',3)
            end
            Metric1.MaxACC(1,jj)=max(outputMA(:,index_IBR.ab));
            Metric1.MinACC(1,jj)=min(outputMA(:,index_IBR.ab));
            SteerEFF1=cumsum(abs(outputMA(:,index_IBR.dotbeta)));
            Metric1.SteerEff(1,jj)=SteerEFF1(end);
            Metric1.MaxACC(2,jj)=max(outputMB(:,index_IBR.ab));
            Metric1.MinACC(2,jj)=min(outputMB(:,index_IBR.ab));
            SteerEFF21=cumsum(abs(outputMB(:,index_IBR.dotbeta)));
            Metric1.SteerEff(2,jj)=SteerEFF21(end);
            Metric1.MaxACC(3,jj)=max(outputMC(:,index_IBR.ab));
            Metric1.MinACC(3,jj)=min(outputMC(:,index_IBR.ab));
            SteerEFF31=cumsum(abs(outputMC(:,index_IBR.dotbeta)));
            Metric1.SteerEff(3,jj)=SteerEFF31(end);
            axis equal
            B=imread('carb.png');
            b=image([pstart(1)-2.5,pstart(1)+2.5],[pstart(2)+dis-1.5,pstart(2)+dis+1.5],B);
            G=imread('carg.png');
            g=image([pstart3(1)-dis-1.5,pstart3(1)-dis+1.5],[pstart3(2)+2.5,pstart3(2)-2.5],G);
            R=imread('carr.png');
            r=image([pstart2(1)+dis-1.5,pstart2(1)+dis+1.5],[pstart2(2)+2.5,pstart2(2)-2.5],R);
            [K, map, alphachannel]=imread('cargray_1.png');
            k=image([53-1.5,53+1.5],[58+2.5,58-2.5],K,'AlphaData', alphachannel);
            [W, map1, alphachannel1]=imread('warning_1.png');
            w=image([54-1.5,54+1.5],[56+1.5,56-1.5],W,'AlphaData', alphachannel1);
            th = 0:pi/50:2*pi;
            xunit = dist * cos(th) + 53;
            yunit = dist * sin(th) + 58;
            plot(xunit, yunit,'k--');

            CP=0:0.01:2*pi;
            gklx = 1.5*cos(CP);
            gkly = 1.5*sin(CP);
            gklp = [gklx;gkly];
            idx=[P_H_length/2-3,P_H_length-1];
            for jjj=1:length(idx)

                iff= idx(jjj);
%                 vc = outputMA(iff,index_IBR.ab)/maxxacc;
%                 vc2 = outputMB(iff,index_IBR.ab)/maxxacc2;
%                 vc3 = outputMC(iff,index_IBR.ab)/maxxacc3;
                theta = atan2(outputMA(iff+1,index_IBR.y)-outputMA(iff,index_IBR.y),outputMA(iff+1,index_IBR.x)-outputMA(iff,index_IBR.x)); % to rotate 90 counterclockwise
                R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
                rgklp = [outputMA(iff+1,index_IBR.x);outputMA(iff+1,index_IBR.y)]+R*gklp;
                fill(rgklp(1,:),rgklp(2,:),[0,0,1]);
            %     
                theta2 = atan2(outputMB(iff+1,index_IBR.y)-outputMB(iff,index_IBR.y),outputMB(iff+1,index_IBR.x)-outputMB(iff,index_IBR.x)); % to rotate 90 counterclockwise
                R = [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)];
                rgklp = [outputMB(iff+1,index_IBR.x);outputMB(iff+1,index_IBR.y)]+R*gklp;
                fill(rgklp(1,:),rgklp(2,:),[1,0,0]);
            %     
                theta3 = atan2(outputMC(iff+1,index_IBR.y)-outputMC(iff,index_IBR.y),outputMC(iff+1,index_IBR.x)-outputMC(iff,index_IBR.x)); % to rotate 90 counterclockwise
                R = [cos(theta3) -sin(theta3); sin(theta3) cos(theta3)];
                rgklp = [outputMC(iff+1,index_IBR.x);outputMC(iff+1,index_IBR.y)]+R*gklp;
                fill(rgklp(1,:),rgklp(2,:),[0,1,0]);

            end

            hold off
            %legend ('Trajectory 1','T 2','T 3','Vehicle 1','V 2','V 3')
            set(gca,'FontSize',12)
            savefig('figures/3v_IBR_intersection_NC')
            saveas(gcf,'figures/3v_IBR_intersection_NC','epsc')
            int=integrator_stepsize;

            figure(401)
            hold on
            grid on
            ylim([7,9.2])
            plot(int:int:length(outputMA(:,1))*int,outputMA(:,index_IBR.v),'b','Linewidth',2)
            plot(int:int:length(outputMB(:,1))*int,outputMB(:,index_IBR.v),'r','Linewidth',2)
            plot(int:int:length(outputMC(:,1))*int,outputMC(:,index_IBR.v),'g','Linewidth',2)
            plot(int:int*3:length(outputMA(:,1))*int,outputMA(1:3:end,index_IBR.v),'bx','Linewidth',2)
            plot(int:int*3:length(outputMB(:,1))*int,outputMB(1:3:end,index_IBR.v),'r*','Linewidth',2)
            plot(int:int*3:length(outputMC(:,1))*int,outputMC(1:3:end,index_IBR.v),'go','Linewidth',2)
            
            line([0,6],[maxSpeed,maxSpeed],'Color',[0.2,0.2,0.2],'LineStyle','--','Linewidth',2)
            line([0,6],[targetSpeed,targetSpeed],'Color',[0.8,0.8,0],'LineStyle','--','Linewidth',2)

            hold off
            %legend ('Vehicle 1','V 2','V 3','Location','southeast')
            %xlabel('Time [s]')
            ylabel('Speed [m/s]','interpreter','latex')
            %set(gca,'yticklabel',[])
            grid on

            set(gca,'FontSize',18)
            savefig('figures/3v_IBR_speed_NC')
            saveas(gcf,'figures/3v_IBR_speed_NC','epsc')
        end
        %% IBR
        problem.xinit = xs';
        problem2.xinit = xs2';
        problem3.xinit = xs3';
        x0(12,:)=zeros(1,60);
        x02(12,:)=zeros(1,60);
        x03(12,:)=zeros(1,60);
        problem.x0=x0(:);
        problem2.x0 = x02(:);
        problem3.x0 = x03(:);
        iter=1; 
        outputMold=outputMA;
        outputMold2=outputMB;
        outputMold3=outputMC;
        while iter<=10
            iter=iter+1;
            for ii=1:length(config(1,:))
                if config(jj,ii)==1
                    %go kart 1
                    [output,exitflag,info] = MPCPathFollowing_3v_IBR(problem);
                    solvetimes(end+1)=info.solvetime;
                    if(exitflag==0)
                        a =a+ 1;
                        IND=[IND;i];
                    end
                    if(exitflag~=1 && exitflag ~=0)
                       iter
                       config(jj,ii)
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
                elseif  config(jj,ii)==2
                     %go kart 2
                    [output2,exitflag2,info2] = MPCPathFollowing_3v_IBR(problem2);
                    solvetimes2(end+1)=info2.solvetime;
                    if(exitflag2==0)
                        a2 =a2+ 1;
                        IND2=[IND2;i];
                    end
                    if(exitflag2~=1 && exitflag2 ~=0)
                        iter
                        config(jj,ii)
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
                elseif  config(jj,ii)==3
                    %go kart 3
                    [output3,exitflag3,info3] = MPCPathFollowing_3v_IBR(problem3);
                    solvetimes3(end+1)=info3.solvetime;
                    if(exitflag3==0)
                        a3 =a3+ 1;
                        IND3=[IND3;i];
                    end
                    if(exitflag3~=1 && exitflag3 ~=0)
                        iter
                        config(jj,ii)
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
                   
                end
            end
            distanceX=outputM(:,index_IBR.x)-outputMold(:,index_IBR.x);
            distanceY=outputM(:,index_IBR.y)-outputMold(:,index_IBR.y);
            distanceX2=outputM2(:,index_IBR.x)-outputMold2(:,index_IBR.x);
            distanceY2=outputM2(:,index_IBR.y)-outputMold2(:,index_IBR.y);
            distanceX3=outputM3(:,index_IBR.x)-outputMold3(:,index_IBR.x);
            distanceY3=outputM3(:,index_IBR.y)-outputMold3(:,index_IBR.y);

%                     distanceXold=outputMold(:,index_IBR.x)-outputMold(:,index_IBR.x);
%                     distanceYold=outputMold(:,index_IBR.y)-outputMold(:,index_IBR.y);
%                     distanceX2old=outputMold2(:,index_IBR.x)-outputMold2(:,index_IBR.x);
%                     distanceY2old=outputMold2(:,index_IBR.y)-outputMold2(:,index_IBR.y);
%                     distanceX3old=outputMold3(:,index_IBR.x)-outputMold3(:,index_IBR.x);
%                     distanceY3old=outputMold3(:,index_IBR.y)-outputMold3(:,index_IBR.y);
%                     
            squared_distance_arrayX    = sum((distanceX).^2);
            squared_distance_arrayY    = sum((distanceY).^2);
            squared_distance_arrayX2    = sum((distanceX2).^2);
            squared_distance_arrayY2    = sum((distanceY2).^2);
            squared_distance_arrayX3    = sum((distanceX3).^2);
            squared_distance_arrayY3    = sum((distanceY3).^2);
            par=0.05;
            if (squared_distance_arrayX<=par && squared_distance_arrayY<=par &&...
                squared_distance_arrayX<=par && squared_distance_arrayY<=par && ...
                squared_distance_arrayX<=par && squared_distance_arrayY<=par && ...
                exitflag3==1 && exitflag2==1 && exitflag==1)
                iter-1
                iter=11;
            else
                outputMold=outputM;
                outputMold2=outputM2;
                outputMold3=outputM3;
            end
        end
%         costT1(i)=info.pobj;
%         costT2(i)=info2.pobj;
%         costT3(i)=info3.pobj;
        %outputM = reshape(output.alldata,[model.nvar,model.N])';
               
        x0 = outputM';
        Metric.MaxACC(1,jj)=max(outputM(:,index_IBR.ab));
        Metric.MinACC(1,jj)=min(outputM(:,index_IBR.ab));
        SteerEFF=cumsum(abs(outputM(:,index_IBR.dotbeta)));
        Metric.SteerEff(1,jj)=SteerEFF(end);
        Metric.MaxACC(2,jj)=max(outputM2(:,index_IBR.ab));
        Metric.MinACC(2,jj)=min(outputM2(:,index_IBR.ab));
        SteerEFF2=cumsum(abs(outputM2(:,index_IBR.dotbeta)));
        Metric.SteerEff(2,jj)=SteerEFF2(end);
        Metric.MaxACC(3,jj)=max(outputM3(:,index_IBR.ab));
        Metric.MinACC(3,jj)=min(outputM3(:,index_IBR.ab));
        SteerEFF3=cumsum(abs(outputM3(:,index_IBR.dotbeta)));
        Metric.SteerEff(3,jj)=SteerEFF3(end);
        u = repmat(outputM(1,1:index_IBR.nu),eulersteps,1);
%         [xhist,time] = euler(@(x,u)interstagedx_IBR(x,u),xs,u,...
%             integrator_stepsize/eulersteps);
%         xs = xhist(end,:);
        %xs
%         history((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
%             [time(1:end-1)+(tstart-1)*integrator_stepsize,u,xhist(1:end-1,:)];
%         planc = planc + 1;
%         if(planc>planintervall)
%            planc = 1; 
%            plansx = [plansx; outputM(:,index_IBR.x)'];
%            plansy = [plansy; outputM(:,index_IBR.y)'];
%            planss = [planss; outputM(:,index_IBR.s)'];
%            [tx,ty]=casadiDynamicBSPLINE(outputM(end,index_IBR.s),nextSplinePoints);
%            targets = [targets;tx,ty];
%         end
        Pos1=[outputM(2:end,index_IBR.x),outputM(2:end,index_IBR.y)];

        %outputM2 = reshape(output2.alldata,[model.nvar,model.N])';
        x02 = outputM2';
        u2 = repmat(outputM2(1,1:index_IBR.nu),eulersteps,1);
%         [xhist2,time2] = euler(@(x2,u2)interstagedx_IBR(x2,u2),...
%             xs2,u2,integrator_stepsize/eulersteps);
%         xs2 = xhist2(end,:);
%         %xs2
%         history2((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
%             [time2(1:end-1)+(tstart-1)*integrator_stepsize,u2,...
%             xhist2(1:end-1,:)];
%         planc2 = planc2 + 1;
%         if(planc2>planintervall)
%            planc2 = 1; 
%            plansx2 = [plansx2; outputM2(:,index_IBR.x)'];
%            plansy2 = [plansy2; outputM2(:,index_IBR.y)'];
%            planss2 = [planss2; outputM2(:,index_IBR.s)'];
%            [tx2,ty2]=casadiDynamicBSPLINE(outputM2(end,index_IBR.s),nextSplinePoints2);
%            targets2 = [targets2;tx2,ty2];
%         end
        Pos2=[outputM2(2:end,index_IBR.x),outputM2(2:end,index_IBR.y)];

        x03 = outputM3';
        u3 = repmat(outputM3(1,1:index_IBR.nu),eulersteps,1);
%         [xhist3,time3] = euler(@(x3,u3)interstagedx_IBR(x3,u3),...
%             xs3,u3,integrator_stepsize/eulersteps);
%         xs3 = xhist3(end,:);
%         %xs3
%         history3((tstart-1)*eulersteps+1:(tstart)*eulersteps,:)=...
%             [time3(1:end-1)+(tstart-1)*integrator_stepsize,u3,...
%             xhist3(1:end-1,:)];
%         planc3 = planc3 + 1;
%         if(planc3>planintervall)
%            planc3 = 1; 
%            plansx3 = [plansx3; outputM3(:,index_IBR.x)'];
%            plansy3 = [plansy3; outputM3(:,index_IBR.y)'];
%            planss3 = [planss3; outputM3(:,index_IBR.s)'];
%            [tx3,ty3]=casadiDynamicBSPLINE(outputM3(end,index_IBR.s),nextSplinePoints3);
%            targets3 = [targets3;tx3,ty3];
%         end
        cost1(jj,i)=info.pobj;
        cost2(jj,i)=info2.pobj;
        cost3(jj,i)=info3.pobj;
        optA=0;
        regAB1=0;
        regBetaA=0;
        latcostA=0;
        lagcostA=0;
        optB=0;
        regAB2=0;
        regBetaB=0;
        latcostB=0;
        lagcostB=0;
        optC=0;
        regAB3=0;
        regBetaC=0;
        latcostC=0;
        lagcostC=0;
        slackA=0;
        slackB=0;
        slackC=0;
        speedcostA=0;
        speedcostA2=0;
        speedcostB=0;
        speedcostB2=0;
        speedcostC=0;
        speedcostC2=0;
        for uu=1:length(outputM)
            [lagcost,latcost,regAB,regBeta,slack2,speedcost,speedcost2,f] = objective_IBR_LE_Test(outputM(uu,:),points,targetSpeed, plagerror, platerror,...
                               pprog, pab, pdotbeta, pspeedcost,pslack,pslack2);

            regAB1=regAB1+regAB;
            regBetaA=regBetaA+regBeta;
            latcostA=latcostA+latcost;
            lagcostA=lagcostA+lagcost;
            speedcostA=speedcostA+speedcost;
            speedcostA2=speedcostA2+speedcost2;
            optA = optA+ f;
            %slackA=slackA+pslack*slack;
            slackA=slackA+pslack2*slack2;
        end
        Steer1(jj,i)=regBetaA;
        Jerk1(jj,i)=regAB1;
        Speed1(jj,i)=speedcostA;
        Speed1_1(jj,i)=speedcostA2;
        LatError1(jj,i)=latcostA;
        LagError1(jj,i)=lagcostA;
        
        for uu=1:length(outputM2)
            [lagcost,latcost,regAB,regBeta,slack2,speedcost,speedcost2,f] = objective_IBR_LE_Test(outputM2(uu,:),points2,targetSpeed, plagerror, platerror,...
                               pprog, pab, pdotbeta, pspeedcost,pslack,pslack2);

            regAB2=regAB2+regAB;
            regBetaB=regBetaB+regBeta;
            latcostB=latcostB+latcost;
            lagcostB=lagcostB+lagcost;
            speedcostB=speedcostB+speedcost;
            speedcostB2=speedcostB2+speedcost2;
            optB = optB+ f;
            %slackA=slackA+pslack*slack;
            slackB=slackB+pslack2*slack2;
        end
        Steer2(jj,i)=regBetaB;
        Jerk2(jj,i)=regAB2;
        Speed2(jj,i)=speedcostB;
        Speed2_1(jj,i)=speedcostB2;
        LatError2(jj,i)=latcostB;
        LagError2(jj,i)=lagcostB;
        for uu=1:length(outputM3)
            [lagcost,latcost,regAB,regBeta,slack2,speedcost,speedcost2,f] = objective_IBR_LE_Test(outputM3(uu,:),points3,targetSpeed, plagerror, platerror,...
                               pprog, pab, pdotbeta, pspeedcost,pslack,pslack2);

            regAB3=regAB3+regAB;
            regBetaC=regBetaC+regBeta;
            latcostC=latcostC+latcost;
            lagcostC=lagcostC+lagcost;
            speedcostC=speedcostC+speedcost;
            speedcostC2=speedcostC2+speedcost2;
            optC = optC+ f;
            %slackA=slackA+pslack*slack;
            slackC=slackC+pslack2*slack2;
        end
        Steer3(jj,i)=regBetaC; 
        Jerk3(jj,i)=regAB3;
        Speed3(jj,i)=speedcostC;
        Speed3_1(jj,i)=speedcostC2;
        LatError3(jj,i)=latcostC;
        LagError3(jj,i)=lagcostC;
        %costS(i)=costS;
        Progress1(jj,i)=outputM(1,index_IBR.s);
        Progress2(jj,i)=outputM2(1,index_IBR.s);
        Progress3(jj,i)=outputM3(1,index_IBR.s);
        % check
        Pos3=[outputM3(2:end,index_IBR.x),outputM3(2:end,index_IBR.y)];

%         distanceX=xs(1)-xs2(1);
%         distanceY=xs(2)-xs2(2);
%         distanceX2=xs(1)-xs3(1);
%         distanceY2=xs(2)-xs3(2);
%         distanceX3=xs2(1)-xs3(1);
%         distanceY3=xs2(2)-xs3(2);
% 
%         squared_distance_array    = sqrt(distanceX.^2 + distanceY.^2);
%         squared_distance_array2   = sqrt(distanceX2.^2 + distanceY2.^2);
%         squared_distance_array3   = sqrt(distanceX3.^2 + distanceY3.^2);
% 
%         if squared_distance_array<=dist
%             squared_distance_array
%         end
%         if squared_distance_array2<=dist
%             squared_distance_array2
%         end
%         if squared_distance_array3<=dist
%             squared_distance_array3
%         end
    end
    %[t,ab,dotbeta,x,y,theta,v,beta,s]
    if tend==1
        if Plotta==1
            figure(5+(jj-1)*2)
            hold on
            set(gca,'visible','off')

            figure(6+(jj-1)*2)
            hold on
            grid on
            ylim([7,9.2])
            xlim([0,6])

            figure(5+(jj-1)*2)
            I=imread('road06.png');
            h=image([20 80],[80 20],I);
            
            maxxacc=max(abs(outputM(:,index_IBR.ab)));
            maxxacc2=max(abs(outputM2(:,index_IBR.ab)));
            maxxacc3=max(abs(outputM3(:,index_IBR.ab)));


            for ii=1:length(outputM(1:P_H_length,index_IBR.x))-1
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
                line(x,y,'Color',[0,0,0.5+0.5*vc],'Linewidth',3)
                line(x2,y2,'Color',[0.5+0.5*vc2,0,0],'Linewidth',3)
                line(x3,y3,'Color',[0,0.5+0.5*vc3,0],'Linewidth',3)
            end
%             plot(outputM(:,index_IBR.x),outputM(:,index_IBR.y),'Color',[0,0,1],'Linewidth',3)
%             plot(outputM2(:,index_IBR.x),outputM2(:,index_IBR.y),'Color',[1,0,0],'Linewidth',3)
%             plot(outputM3(:,index_IBR.x),outputM3(:,index_IBR.y),'Color',[0,1,0],'Linewidth',3)
    

            B=imread('carb.png');
            b=image([pstart(1)-2.5,pstart(1)+2.5],[pstart(2)+dis-1.5,pstart(2)+dis+1.5],B);
            G=imread('carg.png');
            g=image([pstart3(1)-dis-1.5,pstart3(1)-dis+1.5],[pstart3(2)+2.5,pstart3(2)-2.5],G);
            R=imread('carr.png');
            r=image([pstart2(1)+dis-1.5,pstart2(1)+dis+1.5],[pstart2(2)+2.5,pstart2(2)-2.5],R);
            [K, map, alphachannel]=imread('cargray_1.png');
            k=image([53-1.5,53+1.5],[58+2.5,58-2.5],K,'AlphaData', alphachannel);
            [W, map1, alphachannel1]=imread('warning_1.png');
            w=image([54-1.5,54+1.5],[56+1.5,56-1.5],W,'AlphaData', alphachannel1);
            th = 0:pi/50:2*pi;
            xunit = dist * cos(th) + 53;
            yunit = dist * sin(th) + 58;
            plot(xunit, yunit,'k--');
            axis equal
            CP=0:0.01:2*pi;
            gklx = 1.5*cos(CP);
            gkly = 1.5*sin(CP);
            gklp = [gklx;gkly];
            idx=[P_H_length/2,P_H_length-1];
            for jjj=1:length(idx)

                iff= idx(jjj);
                theta = atan2(outputM(iff+1,index_IBR.y)-outputM(iff,index_IBR.y),outputM(iff+1,index_IBR.x)-outputM(iff,index_IBR.x)); % to rotate 90 counterclockwise
                R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
                rgklp = [outputM(iff+1,index_IBR.x);outputM(iff+1,index_IBR.y)]+R*gklp;
                fill(rgklp(1,:),rgklp(2,:),[0,0,1]);
            %     
                theta2 = atan2(outputM2(iff+1,index_IBR.y)-outputM2(iff,index_IBR.y),outputM2(iff+1,index_IBR.x)-outputM2(iff,index_IBR.x)); % to rotate 90 counterclockwise
                R = [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)];
                rgklp = [outputM2(iff+1,index_IBR.x);outputM2(iff+1,index_IBR.y)]+R*gklp;
                fill(rgklp(1,:),rgklp(2,:),[1,0,0]);
            %     
                theta3 = atan2(outputM3(iff+1,index_IBR.y)-outputM3(iff,index_IBR.y),outputM3(iff+1,index_IBR.x)-outputM3(iff,index_IBR.x)); % to rotate 90 counterclockwise
                R = [cos(theta3) -sin(theta3); sin(theta3) cos(theta3)];
                rgklp = [outputM3(iff+1,index_IBR.x);outputM3(iff+1,index_IBR.y)]+R*gklp;
                fill(rgklp(1,:),rgklp(2,:),[0,1,0]);

            end

            hold off
            %legend ('Trajectory 1','T 2','T 3','Vehicle 1','V 2','V 3')
            set(gca,'FontSize',12)
            if isequal(order,[1,2,3])
                savefig('figures/3v_IBR_intersection_brg')
                saveas(gcf,'figures/3v_IBR_intersection_brg','epsc')
            elseif isequal(order,[1,3,2])
                savefig('figures/3v_IBR_intersection_bgr')
                saveas(gcf,'figures/3v_IBR_intersection_bgr','epsc')
            elseif isequal(order,[2,1,3])
                savefig('figures/3v_IBR_intersection_rbg')
                saveas(gcf,'figures/3v_IBR_intersection_rbg','epsc')
            elseif isequal(order,[2,3,1])
                savefig('figures/3v_IBR_intersection_rgb')
                saveas(gcf,'figures/3v_IBR_intersection_rgb','epsc')
            elseif isequal(order,[3,1,2])
                savefig('figures/3v_IBR_intersection_gbr')
                saveas(gcf,'figures/3v_IBR_intersection_gbr','epsc')
            elseif isequal(order,[3,2,1])
                savefig('figures/3v_IBR_intersection_grb')
                saveas(gcf,'figures/3v_IBR_intersection_grb','epsc')
            end
            int=integrator_stepsize;

            figure(6+(jj-1)*2)
            plot(int:int:length(outputM(:,1))*int,outputM(:,index_IBR.v),'b','Linewidth',2)
            plot(int:int:length(outputM(:,1))*int,outputM2(:,index_IBR.v),'r','Linewidth',2)
            plot(int:int:length(outputM(:,1))*int,outputM3(:,index_IBR.v),'g','Linewidth',2)
            scatter(int:int*3:length(outputM(:,1))*int,outputM(1:3:end,index_IBR.v),'bx','Linewidth',2)
            scatter(int:int*3:length(outputM(:,1))*int,outputM2(1:3:end,index_IBR.v),'r*','Linewidth',2)
            scatter(int:int*3:length(outputM(:,1))*int,outputM3(1:3:end,index_IBR.v),'go','Linewidth',2)
            line([0,6],[maxSpeed,maxSpeed],'Color',[0.2,0.2,0.2],'LineStyle','--','Linewidth',2)
            line([0,6],[targetSpeed,targetSpeed],'Color',[0.8,0.8,0],'LineStyle','--','Linewidth',2)
            xlim([0,6])
            hold off
            %legend ('Vehicle 1','V 2','V 3','Location','southeast')
            %xlabel('Time [s]','interpreter','latex')
%             if jj==4
%                 ylabel('Speed [m/s]','interpreter','latex')
%             end
%             if jj~=4
               set(gca,'yticklabel',[])
               grid on
            %end

            set(gca,'FontSize',18)
            if isequal(order,[1,2,3])
                savefig('figures/3v_IBR_speed_brg')
                saveas(gcf,'figures/3v_IBR_speed_brg','epsc')
            elseif isequal(order,[1,3,2])
                savefig('figures/3v_IBR_speed_bgr')
                saveas(gcf,'figures/3v_IBR_speed_bgr','epsc')
            elseif isequal(order,[2,1,3])
                savefig('figures/3v_IBR_speed_rbg')
                saveas(gcf,'figures/3v_IBR_speed_rbg','epsc')
            elseif isequal(order,[2,3,1])
                savefig('figures/3v_IBR_speed_rgb')
                saveas(gcf,'figures/3v_IBR_speed_rgb','epsc')
            elseif isequal(order,[3,1,2])
                savefig('figures/3v_IBR_speed_gbr')
                saveas(gcf,'figures/3v_IBR_speed_gbr','epsc')
            elseif isequal(order,[3,2,1])
                savefig('figures/3v_IBR_speed_grb')
                saveas(gcf,'figures/3v_IBR_speed_grb','epsc')
            end
            pause(0.2)  
        end
%         
%         figure(1000)
%         if jj==1
%             
%             hold on
%             I=imread('road06.png');
%             h=image([20 80],[80 20],I);
%             set(gca,'visible','off')
%             axis equal
%         end
%         P1=plot(outputM(:,index_IBR.x),outputM(:,index_IBR.y),'Color',[0,0,0],'Linewidth',0.5);
%         P2=plot(outputM2(:,index_IBR.x),outputM2(:,index_IBR.y),'Color',[0,0,0],'Linewidth',0.5);
%         P3=plot(outputM3(:,index_IBR.x),outputM3(:,index_IBR.y),'Color',[0,0,0],'Linewidth',0.5);
%         P1.Color(4)=0.6;
%         P2.Color(4)=0.6;
%         P3.Color(4)=0.6;
%         if jj==6
%             B=imread('carb.png');
%             b=image([pstart(1)-2.5,pstart(1)+2.5],[pstart(2)-1.5,pstart(2)+1.5],B);
%             G=imread('carg.png');
%             g=image([pstart3(1)-1.5,pstart3(1)+1.5],[pstart3(2)+2.5,pstart3(2)-2.5],G);
%             R=imread('carr.png');
%             r=image([pstart2(1)-1.5,pstart2(1)+1.5],[pstart2(2)+2.5,pstart2(2)-2.5],R);
%             savefig('figures/3v_IBR_intall_v1')
%             saveas(gcf,'figures/3v_IBR_intall_v1','epsc')
%         end
        %drawAnimation_P3_PH_IBR
    else
        drawIBR3
        figure
        hold on
        plot(cost1,'b')
        plot(cost2,'r')
        plot(cost3,'g')
        plot(cost1+cost2+cost3,'c')
        legend ('Vehicle 1','V 2','V 3','Tot','interpreter','latex')
    end
    
end
%%

if Plotta==1 && Plotta1==1
    figure (100)
    load('PG.mat')
    COSTS=[[1:7]',[cost1,cost2,cost3;optA,optB,optC]];
    for ii=1:7
    COSTS(ii,5)=mean(COSTS(ii,2:4));
    end
    T = array2table(COSTS);
    T.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
    writetable(T,'Costs_IBR.txt','Delimiter',' ')
    scatter(1:length(COSTS),COSTS(:,2),'bx','Linewidth',2);
    %alpha(.5)  
    hold on
    scatter(1:length(COSTS),COSTS(:,3),'r*','Linewidth',2)
    scatter(1:length(COSTS),COSTS(:,4),'go','Linewidth',2)
    scatter(1:length(COSTS),COSTS(:,5),'k+','Linewidth',2)
    %line([0,8],[opt/3,opt/3],'Color',[0.8,0.8,0],'LineStyle','--','Linewidth',0.5)

    grid on
    %title('Costs')
    xlim([0,8])
    xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
    %legend ('V 1','V 2','V 3','Mean')
    hold off
    set(gca,'FontSize',15)
    savefig('figures/3v_IBR_costs')
    saveas(gcf,'figures/3v_IBR_costs','epsc')

    figure (200)
    SteerCost=[[1:7]',[Steer1,Steer2,Steer3;regBetaA,regBetaB,regBetaC]];
    for ii=1:7
    SteerCost(ii,5)=mean(SteerCost(ii,2:4));
    end
    T1 = array2table(SteerCost);
    T1.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
    writetable(T1,'SteerCosts_IBR.txt','Delimiter',' ')
    plot(1:length(SteerCost),SteerCost(:,2),'bx','Linewidth',2.5)
    hold on
    plot(1:length(SteerCost),SteerCost(:,3),'r*','Linewidth',2.5)
    plot(1:length(SteerCost),SteerCost(:,4),'go','Linewidth',2.5)
    plot(1:length(SteerCost),SteerCost(:,5),'k+','Linewidth',2.5)
    %plot(1:length(cost1),(cost1+cost2+cost3)/3,'c*','Linewidth',2)
    % plot(length(Steer1)+1,regBetaA,'bx','Linewidth',2.5)
    % plot(length(Steer1)+1,regBetaB,'r*','Linewidth',2.5)
    % plot(length(Steer1)+1,regBetaC,'go','Linewidth',2.5)
    %plot(length(cost1)+1,(1.9105)/3,'c*','Linewidth',2)
    grid on
    %title('Costs')
    xlim([0,8])
    xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
    %legend ('V 1','V 2','V 3')
    hold off
    set(gca,'FontSize',15)
    savefig('figures/3v_IBR_steer')
    saveas(gcf,'figures/3v_IBR_steer','epsc')

    figure (300)
    JerkCost=[[1:7]',[Jerk1,Jerk2,Jerk3;regABA,regABB,regABC]];
    for ii=1:7
    JerkCost(ii,5)=mean(JerkCost(ii,2:4));
    end
    T2 = array2table(JerkCost);
    T2.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
    writetable(T2,'JerkCost_IBR.txt','Delimiter',' ')
    plot(1:length(JerkCost),JerkCost(:,2),'bx','Linewidth',2.5)
    hold on
    plot(1:length(JerkCost),JerkCost(:,3),'r*','Linewidth',2.5)
    plot(1:length(JerkCost),JerkCost(:,4),'go','Linewidth',2.5)
    plot(1:length(JerkCost),JerkCost(:,5),'k+','Linewidth',2.5)

    grid on
    %title('Costs')
    xlim([0,8])
    xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
    %legend ('V 1','V 2','V 3')
    hold off
    set(gca,'FontSize',15)
    savefig('figures/3v_IBR_jerk')
    saveas(gcf,'figures/3v_IBR_jerk','epsc')

    figure (500)
    SpeedCost=[[1:7]',[Speed1+Speed1_1,Speed2+Speed2_1,Speed3+Speed3_1;speedcostA+speedcostA1,speedcostB+speedcostB1,speedcostC+speedcostC1]];
    for ii=1:7
    SpeedCost(ii,5)=mean(SpeedCost(ii,2:4));
    end
    T3 = array2table(SpeedCost);
    T3.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
    writetable(T3,'SpeedCost_IBR.txt','Delimiter',' ')
    plot(1:length(SpeedCost),SpeedCost(:,2),'bx','Linewidth',2.5)
    hold on
    plot(1:length(SpeedCost),SpeedCost(:,3),'r*','Linewidth',2.5)
    plot(1:length(SpeedCost),SpeedCost(:,4),'go','Linewidth',2.5)
    plot(1:length(SpeedCost),SpeedCost(:,5),'k+','Linewidth',2.5)

    grid on
    %title('Costs')
    xlim([0,8])
    xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
    %legend ('V 1','V 2','V 3')
    hold off
    set(gca,'FontSize',15)
    savefig('figures/3v_IBR_SpeedCost')
    saveas(gcf,'figures/3v_IBR_SpeedCost','epsc')

    figure (600)
    SpeedCost1=[[1:7]',[Speed1_1,Speed2_1,Speed3_1;speedcostA1,speedcostB1,speedcostC1]];
    for ii=1:7
    SpeedCost1(ii,5)=mean(SpeedCost1(ii,2:4));
    end
    T4 = array2table(SpeedCost1);
    T4.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
    writetable(T4,'SpeedCost1_IBR.txt','Delimiter',' ')
    plot(1:length(SpeedCost1),SpeedCost1(:,2),'bx','Linewidth',2.5)
    hold on
    plot(1:length(SpeedCost1),SpeedCost1(:,3),'r*','Linewidth',2.5)
    plot(1:length(SpeedCost1),SpeedCost1(:,4),'go','Linewidth',2.5)
    plot(1:length(SpeedCost1),SpeedCost1(:,5),'k+','Linewidth',2.5)

    grid on
    %title('Costs')
    xlim([0,8])
    xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
    %legend ('V 1','V 2','V 3')
    hold off
    set(gca,'FontSize',15)
    savefig('figures/3v_IBR_SpeedCost1')
    saveas(gcf,'figures/3v_IBR_SpeedCost1','epsc')

    figure (700)
    LatCost=[[1:7]',[LatError1,LatError2,LatError3;latcostA,latcostB,latcostC]];
    for ii=1:7
    LatCost(ii,5)=mean(LatCost(ii,2:4));
    end
    T5 = array2table(LatCost);
    T5.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
    writetable(T5,'LatCost_IBR.txt','Delimiter',' ')
    plot(1:length(LatCost),LatCost(:,2),'bx','Linewidth',2.5)
    hold on
    plot(1:length(LatCost),LatCost(:,3),'r*','Linewidth',2.5)
    plot(1:length(LatCost),LatCost(:,4),'go','Linewidth',2.5)
    plot(1:length(LatCost),LatCost(:,5),'k+','Linewidth',2.5)

    grid on
    %title('Costs')
    xlim([0,8])
    xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
    %legend ('V 1','V 2','V 3')
    hold off
    set(gca,'FontSize',15)
    savefig('figures/3v_IBR_LatCost')
    saveas(gcf,'figures/3v_IBR_LatCost1','epsc')

    figure (800)
    LagCost=[[1:7]',[LagError1,LagError2,LagError3;lagcostA,lagcostB,lagcostC]];
    for ii=1:7
    LagCost(ii,5)=mean(LagCost(ii,2:4));
    end
    T6 = array2table(LagCost);
    T6.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
    writetable(T6,'LatCost_IBR.txt','Delimiter',' ')
    plot(1:length(LagCost),LagCost(:,2),'bx','Linewidth',2.5)
    hold on
    plot(1:length(LagCost),LagCost(:,3),'r*','Linewidth',2.5)
    plot(1:length(LagCost),LagCost(:,4),'go','Linewidth',2.5)
    plot(1:length(LagCost),LagCost(:,5),'k+','Linewidth',2.5)

    grid on
    %title('Costs')
    xlim([0,8])
    xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
    %legend ('V 1','V 2','V 3')
    hold off
    set(gca,'FontSize',15)
    savefig('figures/3v_IBR_LagCost')
    saveas(gcf,'figures/3v_IBR_LagCost1','epsc')
    
    figure (900)
    TotCost=[[1:7]',[LagError1+LatError1,LagError2++LatError2,LagError3+LatError3;lagcostA+latcostA,lagcostB+latcostB,lagcostC+latcostC]];
    for ii=1:7
    TotCost(ii,5)=mean(TotCost(ii,2:4));
    end
    T7 = array2table(TotCost);
    T7.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
    writetable(T7,'TotCost_IBR.txt','Delimiter',' ')
    plot(1:length(TotCost),TotCost(:,2),'bx','Linewidth',2.5)
    hold on
    plot(1:length(TotCost),TotCost(:,3),'r*','Linewidth',2.5)
    plot(1:length(TotCost),TotCost(:,4),'go','Linewidth',2.5)
    plot(1:length(TotCost),TotCost(:,5),'k+','Linewidth',2.5)

    grid on
    %title('Costs')
    xlim([0,8])
    xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
    %legend ('V 1','V 2','V 3')
    hold off
    set(gca,'FontSize',15)
    savefig('figures/3v_IBR_TotCost')
    saveas(gcf,'figures/3v_IBR_TotCost1','epsc')
end

save('Metric.mat','Metric','Metric1')