% go kart parameters
maxSpeed = 10;
maxxacc = 4;
maxyacc = 8;
latacclim = 6;
rotacceffect  = 2;
torqueveceffect = 3;
brakeeffect = 0; 

% cost function parameters
plagerror=1;       % proj error param cost
platerror=0.01;    % lateral error param cost
pprog=0.2;         % progress parameter cost
pab=0.0004;        % acc parameter cost
pdotbeta=0.1;      % steering velocity parameter cost
pspeedcost=0.2;    % parameter for cost that occurs when max_speed is exceeded
pslack=5;          % slack variable 

% Splines
pointsO = 14;      % N gokart parameters + N cost function parameters
pointsN = 10;      % N spline points ahead considered
splinestart = 1;

% Stepsize
integrator_stepsize = 0.1;

% Simulation length
tend = 100;
eulersteps = 10;
planintervall = 1;

%% controller params %%RECOMPILE IF MODIFIED!
NUM_const=3; % number of nonlinear constraint
P_H_length=31; % prediction horizon length
MAX_IT= 200; % N of max iterations

% ds constraint (delta progress)
ds_max=5;
ds_min=-1;

% beta constraint (steering angle)
beta_max=0.5;
beta_min=-0.5;

%% Spline Points
    
points = [36.2,52,57.2,53,52,47,41.8;...
          44.933,58.2,53.8,49,44,43,38.33;...
          1.8,1.8,1.8,0.5,0.5,0.5,1.8]';
points(:,3)=points(:,3)-0.2;

trajectorytimestep = integrator_stepsize;
solvetimes = [];