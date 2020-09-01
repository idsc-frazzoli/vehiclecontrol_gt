%% Parameters Definitions
maxSpeed = 30;
maxxacc = 4;
maxyacc = 8;
latacclim = 6;
rotacceffect  = 2;
torqueveceffect = 3;
brakeeffect = 0; 
plagerror=1;
platerror=0.02;
pprog=0.4;
pab=0.0004;
pdotbeta=0.009;
pspeedcost=0.3;
pslack=10000000;
pslack2=10000000;
dist=2;

% Splines
pointsO = 18;
pointsN = 10;
pointsN2 = 10;
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;

vel1=5;
vel2=5;

% Simulation length
tend = 1;
eulersteps = 10;
planintervall = 1;

%% Spline Points
points = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
          20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110; ...    %y
          5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5]';
points2 = [20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110;...          %x
          50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
          5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5]';  

solvetimes = [];

solvetimes2 = [];

P_H_length=40;