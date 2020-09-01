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
pslack=1000;          % slack variable 
pslack2=1000;        % collision cost
dist=2;            % min distance available
% alpha parameters for CF (only if alpha condition active, update pointsO)
alpha1=0.5;
alpha2=1;

% Splines
pointsO = 16;      % N gokart parameters + N cost function parameters
pointsN = 10;      % N spline points ahead considered kart 1
pointsN2 = 10;     % N spline points ahead considered kart 2
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;
% Stepsize
integrator_stepsize = 0.1;

% Simulation length
tend = 90;
eulersteps = 10;
planintervall = 1;

%% controller params %%RECOMPILE IF MODIFIED!
NUM_const=13; % number of nonlinear constraint
P_H_length=40;% prediction horizon length
MAX_IT= 400;  % N of max iterations

% ds constraint (delta progress)
ds_max=5;
ds_min=-1;

% beta constraint (steering angle)
beta_max=0.5;
beta_min=-0.5;
%% Spline Points

points = [10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95;...          %x
          50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
          10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
      
% points2 = [5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110;...          %x
%           50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
%           3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';  
% points = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
%           5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110; ...    %y
%           3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';
solvetimes = [];
solvetimes2 = [];
trajectorytimestep = integrator_stepsize;
