% go kart parameters
params.maxSpeed = 9;%10;
params.maxSpeed_1 = 20;
params.targetSpeed=8.3;
params.posX4=53;

% cost function parameters
params.plagerror=1;       % proj error param cost
params.platerror=1.2;    % lateral error param cost
params.pprog=0.8;         % progress parameter cost
params.pab=0.006;        % acc parameter cost
params.pdotbeta=4;      % steering velocity parameter cost
params.pspeedcost=2;    % parameter for cost that occurs when max_speed is exceeded
params.pSpeedMax=3;
params.PprogMax=8;
params.pslack=2;          % slack variable 
params.pslack2=100;        % collision cost
params.dist=4;   

% Splines
pointsO = 16;      % N gokart parameters + N cost function parameters
pointsN = 14;      % N spline points ahead considered kart 1
pointsN2 = 14;     % N spline points ahead considered kart 2
pointsN3 = 14;     % N spline points ahead considered kart 3 
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;
% Stepsize
integrator_stepsize = 0.1;
params.integrator_stepsize=integrator_stepsize;
% Simulation length
tend = 1;
eulersteps = 10;
planintervall = 1;

%% controller params %%RECOMPILE IF MODIFIED!
P_H_length=60; % prediction horizon length

% ds constraint (delta progress)
ds_max=5;
ds_min=-1;

% beta constraint (steering angle)
beta_max=pi/3;
beta_min=-pi/3;
%% Spline Points
points = [10,15,20,25,30,35,40,45,50,55,60,65,70,75;...          %x,75,80,85,90,95
          50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y,50,50,50,50,50
          3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]'; %,5,5,5,5,5  
points=flip(points);
points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50
          25,30,35,40,45,50,55,60,65,70,75,80,85,90; ...    %y,75,80,85,90,95
          3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]';  %,5,5,5,5,5
 
points3 = [50,50,50,50,50,50,52,55,60,65,70,75,80,85;...          %x,50,50,50,50,50,50,65,65,50
           75,70,65,60,55,52,50,50,50,50,50,50,50,50; ...    %y,25,20,15,10,5,0,5,92,95
           3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]';%,5,5,5,5,4,3,2,2,3

solvetimes  = [];
solvetimes2 = [];
solvetimes3 = [];
trajectorytimestep = integrator_stepsize;