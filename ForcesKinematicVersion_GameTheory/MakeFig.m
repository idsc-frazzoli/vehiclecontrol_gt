function [] = MakeFig(jj,outputM,outputM2,outputM3,params)
global index_IBR
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

P_H_length=60;
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
dis=0;
pstart=[outputM(1,index_IBR.x),outputM(1,index_IBR.y)];
pstart2=[outputM2(1,index_IBR.x),outputM2(1,index_IBR.y)];
pstart3=[outputM3(1,index_IBR.x),outputM3(1,index_IBR.y)];
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
xunit = params.dist * cos(th) + 53;
yunit = params.dist * sin(th) + 58;
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
int=params.integrator_stepsize;

figure(6+(jj-1)*2)
plot(int:int:length(outputM(:,1))*int,outputM(:,index_IBR.v),'b','Linewidth',2)
plot(int:int:length(outputM(:,1))*int,outputM2(:,index_IBR.v),'r','Linewidth',2)
plot(int:int:length(outputM(:,1))*int,outputM3(:,index_IBR.v),'g','Linewidth',2)
scatter(int:int*3:length(outputM(:,1))*int,outputM(1:3:end,index_IBR.v),'bx','Linewidth',2)
scatter(int:int*3:length(outputM(:,1))*int,outputM2(1:3:end,index_IBR.v),'r*','Linewidth',2)
scatter(int:int*3:length(outputM(:,1))*int,outputM3(1:3:end,index_IBR.v),'go','Linewidth',2)
line([0,6],[params.maxSpeed,params.maxSpeed],'Color',[0.2,0.2,0.2],'LineStyle','--','Linewidth',2)
line([0,6],[params.targetSpeed,params.targetSpeed],'Color',[0.8,0.8,0],'LineStyle','--','Linewidth',2)
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



end

