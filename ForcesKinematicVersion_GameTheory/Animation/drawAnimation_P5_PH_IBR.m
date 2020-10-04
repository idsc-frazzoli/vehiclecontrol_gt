global index_IBR
frames = length(outputM(:,1));
vidfile = VideoWriter('P5_PH_IBR','Motion JPEG AVI');
vidfile.FrameRate = 10;
open(vidfile);
set(gcf,'position',[100,100,1000,800])
tracelength = 500;
for iff = 1:frames-1
    figure(1)
    clf
    daspect([1 1 1])
    hold on
    set(gca,'visible','off')
    if(1)
        I=imread('strada3.png');
        h=image([20 80],[20 80],I);
    %points = [36.2,52,57.2,53,55,47,41.8;44.933,58.2,53.8,49,44,43,38.33;1.8,1.8,1.8,0.2,0.2,0.2,1.8]';
    %points = [36.2,52,57.2,53,52,47,41.8;44.933,58.2,53.8,49,44,43,38.33;1.8,1.8,1.8,0.5,0.5,0.5,1.8]';
%        [leftline,middleline,rightline] = drawTrack(points(:,1:2),points(:,3)+0.5);
%        [rleftline,rmiddleline,rrightline] = drawTrack(points(:,1:2),points(:,3));
%        plot(leftline(:,1),leftline(:,2),'b')
%        plot(rightline(:,1),rightline(:,2),'b')
%        plot(rleftline(:,1),rleftline(:,2),'--b')
%        plot(rrightline(:,1),rrightline(:,2),'--b')
%        
%        [leftline2,middleline2,rightline2] = drawTrack(points2(:,1:2),points2(:,3)+0.5);
%        [rleftline2,rmiddleline2,rrightline2] = drawTrack(points2(:,1:2),points2(:,3));
%        plot(leftline2(:,1),leftline2(:,2),'r')
%        plot(rightline2(:,1),rightline2(:,2),'r')
%        plot(rleftline2(:,1),rleftline2(:,2),'--r')
%        plot(rrightline2(:,1),rrightline2(:,2),'--r')
%        
%        [leftline3,middleline3,rightline3] = drawTrack(points3(:,1:2),points3(:,3)+0.5);
%        [rleftline3,rmiddleline3,rrightline3] = drawTrack(points3(:,1:2),points3(:,3));
%        plot(leftline3(:,1),leftline3(:,2),'g')
%        plot(rightline3(:,1),rightline3(:,2),'g')
%        plot(rleftline3(:,1),rleftline3(:,2),'--g')
%        plot(rrightline3(:,1),rrightline3(:,2),'--g')
%        
%        [leftline4,middleline4,rightline4] = drawTrack(points4(:,1:2),points4(:,3)+0.5);
%        [rleftline4,rmiddleline4,rrightline4] = drawTrack(points4(:,1:2),points4(:,3));
%        plot(leftline4(:,1),leftline4(:,2),'c')
%        plot(rightline4(:,1),rightline4(:,2),'c')
%        plot(rleftline4(:,1),rleftline4(:,2),'--c')
%        plot(rrightline4(:,1),rrightline4(:,2),'--c')
%        
%        [leftline5,middleline5,rightline5] = drawTrack(points5(:,1:2),points5(:,3)+0.5);
%        [rleftline5,rmiddleline5,rrightline5] = drawTrack(points5(:,1:2),points5(:,3));
%        plot(leftline5(:,1),leftline5(:,2),'m')
%        plot(rightline5(:,1),rightline5(:,2),'m')
%        plot(rleftline5(:,1),rleftline5(:,2),'--m')
%        plot(rrightline5(:,1),rrightline5(:,2),'--m')
       
%        [leftline3,middleline3,rightline3] = drawTrack(points3(:,1:2),points3(:,3)+0.5);
%        [rleftline3,rmiddleline3,rrightline3] = drawTrack(points3(:,1:2),points3(:,3));
%        plot(leftline3(:,1),leftline3(:,2),'b')
%        plot(rightline3(:,1),rightline3(:,2),'b')
%        plot(rleftline3(:,1),rleftline3(:,2),'--b')
%        plot(rrightline3(:,1),rrightline3(:,2),'--b')
    end
%     if(1)
%        %plot control points
%        sph = splinepointhist(iff*planintervall,:);
%        sphx = sph(2:1+pointsN)';
%        sphy = sph(2+pointsN:1+2*pointsN)';
%        sphr = sph(2+2*pointsN:1+3*pointsN)';
%        [leftline,middleline,rightline]=drawTrack([sphx,sphy],sphr);
%        plot(leftline(1:800,1),leftline(1:800,2),'--b')
%        plot(rightline(1:800,1),rightline(1:800,2),'--b')
%        plot(sphx,sphy,'--s','color',[.7 .2 .2]);
%     end
    endind = iff*eulersteps*planintervall;
    for i=1:frames-1
        
       next = i+1;
       x = [outputM(i,index_IBR.x),outputM(next,index_IBR.x)];
       y = [outputM(i,index_IBR.y),outputM(next,index_IBR.y)];
       x2 = [outputM2(i,index_IBR.x),outputM2(next,index_IBR.x)];
       y2 = [outputM2(i,index_IBR.y),outputM2(next,index_IBR.y)];
       x3 = [outputM3(i,index_IBR.x),outputM3(next,index_IBR.x)];
       y3 = [outputM3(i,index_IBR.y),outputM3(next,index_IBR.y)];
       x4 = [outputM4(i,index_IBR.x),outputM4(next,index_IBR.x)];
       y4 = [outputM4(i,index_IBR.y),outputM4(next,index_IBR.y)];
       x5 = [outputM5(i,index_IBR.x),outputM5(next,index_IBR.x)];
       y5 = [outputM5(i,index_IBR.y),outputM5(next,index_IBR.y)];
       vc = abs(outputM(i,index_IBR.ab))/maxxacc;
       vc2 = abs(outputM2(i,index_IBR.ab))/maxxacc;
       vc3 = abs(outputM3(i,index_IBR.ab))/maxxacc;
       vc4 = abs(outputM4(i,index_IBR.ab))/maxxacc;
       vc5 = abs(outputM5(i,index_IBR.ab))/maxxacc;
       line(x,y,'Color',[0.5-0.5*vc,0.5+0.5*vc,0],'LineWidth',2);
       line(x2,y2,'Color',[0.5-0.5*vc2,0.5+0.5*vc2,0],'LineWidth',2);
       line(x3,y3,'Color',[0.5-0.5*vc3,0.5+0.5*vc3,0],'LineWidth',2);
       line(x4,y4,'Color',[0.5-0.5*vc4,0.5+0.5*vc4,0],'LineWidth',2);
       line(x5,y5,'Color',[0.5-0.5*vc5,0.5+0.5*vc5,0],'LineWidth',2);
    end
    iff
    
%     plot(plansx(iff+1,:),plansy(iff+1,:),'-k','LineWidth',6);
%     plot(plansx2(iff+1,:),plansy2(iff+1,:),'-k','LineWidth',6);
%     plot(plansx3(iff+1,:),plansy3(iff+1,:),'-k','LineWidth',6);
    
    CP=0:0.01:2*pi;
    gklx = 1.5*cos(CP);
    gkly = 1.5*sin(CP);
    gklp = [gklx;gkly];
    
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
    
    theta4 = atan2(outputM4(iff+1,index_IBR.y)-outputM4(iff,index_IBR.y),outputM4(iff+1,index_IBR.x)-outputM4(iff,index_IBR.x)); % to rotate 90 counterclockwise
    R = [cos(theta4) -sin(theta4); sin(theta4) cos(theta4)];
    rgklp = [outputM4(iff+1,index_IBR.x);outputM4(iff+1,index_IBR.y)]+R*gklp;
    fill(rgklp(1,:),rgklp(2,:),'c');
%     
    theta5 = atan2(outputM5(iff+1,index_IBR.y)-outputM5(iff,index_IBR.y),outputM5(iff+1,index_IBR.x)-outputM5(iff,index_IBR.x)); % to rotate 90 counterclockwise
    R = [cos(theta5) -sin(theta5); sin(theta5) cos(theta5)];
    rgklp = [outputM5(iff+1,index_IBR.x);outputM5(iff+1,index_IBR.y)]+R*gklp;
    fill(rgklp(1,:),rgklp(2,:),'m');
    drawnow
    F = getframe(gcf); 
    writeVideo(vidfile,F);
end
close(vidfile)