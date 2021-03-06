%% Parameters Definitions
maxSpeed = 10;
maxxacc = 4;
maxyacc = 8;
latacclim = 6;
rotacceffect  = 2;
torqueveceffect = 3;
brakeeffect = 0; 
plagerror=1;
platerror=1;
pprog=0.4;
pab=0.0004;
pdotbeta=0.3;
pspeedcost=0.3;
pslack=10000000;
pslack2=10000000;
dist=4;

% Splines
pointsO = 18;
pointsN = 10;
pointsN2 = 10;
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;

vel1=8; % Down
vel2=8;   % left

% Simulation length
tend = 1;
eulersteps = 10;
planintervall = 1;

%% Spline Points

points = [20,25,30,35,40,45,50,55,60,65,70,75,80,85;...          %x
          50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
          20,25,30,35,40,45,50,55,60,65,70,75,80,85; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3]';
solvetimes = [];

solvetimes2 = [];

%% controller params %%RECOMPILE IF MODIFIED!
NUM_const=4; % number of nonlinear constraint
P_H_length=40;% prediction horizon length
MAX_IT= 400;  % N of max iterations

% ds constraint (delta progress)
ds_max=5;
ds_min=-1;

% beta constraint (steering angle)
beta_max=0.5;
beta_min=-0.5;