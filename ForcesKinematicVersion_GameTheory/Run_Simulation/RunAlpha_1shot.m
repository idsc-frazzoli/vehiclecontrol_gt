global index index_IBR

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
xs(index.x_k2-index.nu)=pstart2(1);
xs(index.y_k2-index.nu)=pstart2(2);
xs(index.theta_k2-index.nu)=pangle2;
xs(index.v_k2-index.nu)=vel2;
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
xs2(index.v-index.nu)=vel1;
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
xs2(index.v_k2-index.nu)=vel2;
xs2(index.ab_k2-index.nu)=0;
xs2(index.beta_k2-index.nu)=0;
xs2(index.s_k2-index.nu)=0.01;

x02 = [zeros(model.N,index.nu),repmat(xs2,model.N,1)]';

% V1
xs_IBR(index_IBR.x-index_IBR.nu)=pstart(1);
xs_IBR(index_IBR.y-index_IBR.nu)=pstart(2);
xs_IBR(index_IBR.theta-index_IBR.nu)=pangle;
xs_IBR(index_IBR.v-index_IBR.nu)=vel1;
xs_IBR(index_IBR.ab-index_IBR.nu)=0;
xs_IBR(index_IBR.beta-index_IBR.nu)=0;
xs_IBR(index_IBR.s-index_IBR.nu)=0.01;
%V2
xs_IBR2(index_IBR.x-index_IBR.nu)=pstart2(1);
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
alpha1=0:0.1:1;
alpha2=alpha1;
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

%IBR
xs_IBR(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs_IBR(index_IBR.v-index_IBR.nu))...
        -0.0001,xs_IBR(index_IBR.ab-index_IBR.nu));
problem_IBR.xinit = xs_IBR';
    
xs_IBR2(index_IBR.ab-index_IBR.nu)=min(casadiGetMaxAcc(xs_IBR2(index_IBR.v-index_IBR.nu))...
        -0.0001,xs_IBR2(index_IBR.ab-index_IBR.nu));
problem_IBR2.xinit = xs_IBR2';

problem.x0 = x0(:);   
problem2.x0 = x02(:); 
problem_IBR.x0_IBR = x0_IBR(:);
problem_IBR2.x0_IBR = x0_IBR2(:);
tstart = 1;
tstart2 = 1;
for kk=1:length(alpha1)
    
    if(1)
        figure(1)
        plot(pstart(1),pstart(2),'g*')
        hold on
        plot(pstart2(1),pstart2(2),'g*') 
        plot([pstart(1)-2.5,pstart(1)-2.5],[pstart(2),70],'--k')
        plot([pstart(1)+2.5,pstart(1)+2.5],[pstart(2),70],'--k')
        plot([pstart2(1),70],[pstart2(2)-2.5,pstart2(2)-2.5],'--k')
        plot([pstart2(1),70],[pstart2(2)+2.5,pstart2(2)+2.5],'--k')
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
    if alpha1(kk)==0
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
        [output,exitflag,info] = MPCPathFollowing_IBR(problem_IBR);
        if(exitflag==0)
            a =a+ 1;
        end
        outputM = reshape(output.alldata,[model_IBR.nvar,model_IBR.N])';
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
        [output2,exitflag2,info2] = MPCPathFollowing_IBR(problem_IBR2);
        if(exitflag2==0)
            a2 =a2+ 1;
        end
        outputM2 = reshape(output2.alldata,[model_IBR.nvar,model_IBR.N])';
        cost1(kk)=info.pobj;
        optA2(kk)=info.pobj;
        cost2(kk)=info2.pobj;
        optB2(kk)=info2.pobj;
        opt(kk)=optA2(kk)+optB2(kk);
        Progress1(kk)=outputM(1,index_IBR.s);
        Progress2(kk)=outputM2(1,index_IBR.s);
        
        figure(1)
        plot(outputM(:,index_IBR.x),outputM(:,index_IBR.y),'.-','Color',[0,1-alpha1(kk)/max(alpha1)*0.5,1])
        plot(outputM2(:,index_IBR.x),outputM2(:,index_IBR.y),'.-','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])

        figure(2)
        plot(outputM(:,index_IBR.theta),'.-','Color',[0,1-alpha1(kk)/max(alpha1)*0.5,1])
        plot(outputM2(:,index_IBR.theta),'.-','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])

        figure(3)
        plot(outputM(:,index_IBR.v),'.-','Color',[0,1-alpha1(kk)/max(alpha1)*0.5,1])
        plot(outputM2(:,index_IBR.v),'.-','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])
        
        %alpha2=0
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
        [output2,exitflag2,info2] = MPCPathFollowing_IBR(problem_IBR2);
        if(exitflag2==0)
            a2 =a2+ 1;
        end
        outputM2 = reshape(output2.alldata,[model_IBR.nvar,model_IBR.N])';
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
        [output,exitflag,info] = MPCPathFollowing_IBR(problem_IBR);
        if(exitflag==0)
            a =a+ 1;
        end
        outputM = reshape(output.alldata,[model_IBR.nvar,model_IBR.N])';
        cost1(kk)=info.pobj;
        optA(kk)=info.pobj;
        cost2(kk)=info2.pobj;
        optB(kk)=info2.pobj;
        opt2(kk)=optA(kk)+optB(kk);
        Progress1(kk)=outputM(1,index_IBR.s);
        Progress2(kk)=outputM2(1,index_IBR.s);
        figure(1)
        plot(outputM(:,index_IBR.x),outputM(:,index_IBR.y),'.-','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
        plot(outputM2(:,index_IBR.x),outputM2(:,index_IBR.y),'.-','Color',[1,alpha2(kk)/max(alpha2)*0.5,0])

        figure(2)
        plot(outputM(:,index_IBR.theta),'.-','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
        plot(outputM2(:,index_IBR.theta),'.-','Color',[1,alpha2(kk)/max(alpha2)*0.5,0])

        figure(3)
        plot(outputM(:,index_IBR.v),'.-','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
        plot(outputM2(:,index_IBR.v),'.-','Color',[1,alpha2(kk)/max(alpha2)*0.5,0])
    else
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
        [output,exitflag,info] = MPCPathFollowing_2v_alpha(problem);
        [output2,exitflag2,info2] = MPCPathFollowing_2v_alpha(problem2);
        outputM = reshape(output.alldata,[model.nvar,model.N])';
        outputM2 = reshape(output2.alldata,[model.nvar,model.N])';
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
legend('J1_{down}','J2_{left}','Tot/2')
xticklabels({'0,1','0.2,1','0.4,1','0.6,1',...
    '0.8,1','1,1','1,0.8','1,0.6',...
    '1,0.4','1,0.2','1,0'});
% xticklabels({'0.2,1','0.4,1','0.6,1',...
%     '0.8,1','1,1','1,0.8','1,0.6',...
%     '1,0.4','1,0.2'});
figure(5)
for kk=1:length(alpha1)
    plot(alpha1(kk),optA(kk)/(optA(kk)+optB(kk))*100,'*','Color',[0,alpha1(kk)/max(alpha1)*0.5,1])
    plot(alpha1(kk),optB(kk)/(optA(kk)+optB(kk))*100,'*','Color',[1,alpha1(kk)/max(alpha1)*0.5,0])
    plot(2-alpha2(kk),optA2(kk)/(optA2(kk)+optB2(kk))*100,'*','Color',[0,1-alpha2(kk)/max(alpha2)*0.5,1])
    plot(2-alpha2(kk),optB2(kk)/(optA2(kk)+optB2(kk))*100,'*','Color',[1,1-alpha2(kk)/max(alpha2)*0.5,0])
end
legend('V1_{down}','V2_{left}')
xticklabels({'0,1','0.2,1','0.4,1','0.6,1',...
    '0.8,1','1,1','1,0.8','1,0.6',...
    '1,0.4','1,0.2','1,0'});
% xticklabels({'0.2,1','0.4,1','0.6,1',...
%     '0.8,1','1,1','1,0.8','1,0.6',...
%     '1,0.4','1,0.2'});
% xticklabels({'1,0.1','1,0.2','1,0.3','1,0.4','1,0.5','1,0.6','1,0.7',...
%     '1,0.8','1,0.9','1,1','0.9,1','0.8,1','0.7,1','0.6,1','0.5,1',...
%     '0.4,1','0.3,1','0.2,1','0.1,1'});