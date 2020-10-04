%close all

points = [20,30,35,40,45,50,55,60,65,70,75,80,85,90;...          %x
           50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
           3,3,3,3,3,3,3,3,3,3,3,3,3,3]';

points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
           80,75,70,65,60,55,50,45,40,35,30,25,20,15,10; ...    %y
           3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';

points3 = [80,75,70,65,60,52,50,50,50,48,45,40,35,20;...          %x
           60,60,60,60,60,60,57,55,52,50,50,50,50,50; ...    %y
           3,3,3,3,3,3,3,3,3,3,3,3,3,3]';
points3(:,2)=points3(:,2)-10;
points3=flip(points3);

points4 = [20,30,35,40,45,50,55,60,65,70,75,80,85,90;...          %x
          60,60,60,60,60,60,60,60,60,60,60,60,60,60; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
      
points5 = [80,70,65,60,55,50,45,40,35,30,25,20,15,10;...          %x
          40,40,40,40,40,40,40,40,40,40,40,40,40,40; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3]';

[leftline,middleline,rightline] = drawTrack(points(:,1:2),points(:,3));
[leftline2,middleline2,rightline2] = drawTrack(points2(:,1:2),points2(:,3));
[leftline3,middleline3,rightline3] = drawTrack(points3(:,1:2),points3(:,3));
[leftline4,middleline4,rightline4] = drawTrack(points4(:,1:2),points4(:,3));
[leftline5,middleline5,rightline5] = drawTrack(points5(:,1:2),points5(:,3));
figure
hold on
plot(leftline(:,1),leftline(:,2),'b')
plot(middleline(:,1),middleline(:,2),'b')
plot(rightline(:,1),rightline(:,2),'b')
pointsA = points;%points(1,1),points(1,2),points(1,3)];
plot(pointsA(:,1),pointsA(:,2),'b*')

plot(leftline2(:,1),leftline2(:,2),'g')
plot(middleline2(:,1),middleline2(:,2),'g')
plot(rightline2(:,1),rightline2(:,2),'g')
pointsB = points2;%points2(1,1),points2(1,2),points2(1,3)];
plot(pointsB(:,1),pointsB(:,2),'g*')

plot(leftline3(:,1),leftline3(:,2),'r')
plot(middleline3(:,1),middleline3(:,2),'r')
plot(rightline3(:,1),rightline3(:,2),'r')
pointsC = points3;%;points3(1,1),points3(1,2),points3(1,3)]
plot(pointsC(:,1),pointsC(:,2),'r*')

plot(leftline4(:,1),leftline4(:,2),'c')
plot(middleline4(:,1),middleline4(:,2),'c')
plot(rightline4(:,1),rightline4(:,2),'c')
pointsD = points4;%;points3(1,1),points3(1,2),points3(1,3)]
plot(pointsD(:,1),pointsD(:,2),'c*')

plot(leftline5(:,1),leftline5(:,2),'y')
plot(middleline5(:,1),middleline5(:,2),'y')
plot(rightline5(:,1),rightline5(:,2),'y')
pointsE = points5;%;points3(1,1),points3(1,2),points3(1,3)]
plot(pointsE(:,1),pointsE(:,2),'y*')
xlabel('x')
ylabel('y')
grid on
