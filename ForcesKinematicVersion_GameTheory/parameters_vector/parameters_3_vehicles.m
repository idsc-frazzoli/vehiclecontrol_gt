% go kart parameters
maxSpeed = 7;%10;
maxxacc = 4;
maxyacc = 8;
latacclim = 6;
rotacceffect  = 2;
torqueveceffect = 3;
brakeeffect = 0; 

% cost function parameters
plagerror=1;       % proj error param cost
platerror=0.005;    % lateral error param cost
pprog=0.2;         % progress parameter cost
pab=0.0004;        % acc parameter cost
pdotbeta=0.03;      % steering velocity parameter cost
pspeedcost=1;    % parameter for cost that occurs when max_speed is exceeded
pslack=500;          % slack variable 
pslack2=1000000;        % collision cost
dist=3.5;            % min distance available

% alpha parameters for CF (only if alpha condition active, update pointsO)
alpha1=0.5;
alpha2=1;
alpha3=1;

% Splines
pointsO = 16;      % N gokart parameters + N cost function parameters
pointsN = 12;      % N spline points ahead considered kart 1
pointsN2 = 12;     % N spline points ahead considered kart 2
pointsN3 = 12;     % N spline points ahead considered kart 3 
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;
% Stepsize
integrator_stepsize = 0.1;

% Simulation length
tend = 1;
eulersteps = 10;
planintervall = 1;

%% controller params %%RECOMPILE IF MODIFIED!
NUM_const=12; % number of nonlinear constraint
P_H_length=60; % prediction horizon length
MAX_IT= 500; % N of max iterations

% ds constraint (delta progress)
ds_max=5;
ds_min=-1;

% beta constraint (steering angle)
beta_max=0.5;
beta_min=-0.5;
%% Spline Points

points = [25,30,35,40,45,50,55,60,65,70,75,90,100,110;...          %x,75,80,85,90,95
          50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y,50,50,50,50,50
          1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]'; %,5,5,5,5,5  
points(:,2)=points(:,2)-1.75;
points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50
          25,30,35,40,45,50,55,60,65,70,75,80,85,90; ...    %y,75,80,85,90,95
          1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]';  %,5,5,5,5,5
points2(:,1)=points2(:,1)+1.75;
points3 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50,50,65,65,50
           75,70,65,60,55,50,45,40,35,30,25,20,15,10; ...    %y,25,20,15,10,5,0,5,92,95
           1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]';%,5,5,5,5,4,3,2,2,3
points3(:,1)=points3(:,1)-1.75;  

solvetimes  = [];
solvetimes2 = [];
solvetimes3 = [];
trajectorytimestep = integrator_stepsize;
