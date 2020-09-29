% go kart parameters
maxSpeed = 7;
maxxacc = 4;
maxyacc = 8;
latacclim = 6;
rotacceffect  = 2;
torqueveceffect = 3;
brakeeffect = 0; 

% cost function parameters
plagerror=1;       % proj error param cost
platerror=2;    % lateral error param cost
pprog=0.2;         % progress parameter cost
pab=0.0004;        % acc parameter cost
pdotbeta=0.4;      % steering velocity parameter cost
pspeedcost=0.08;    % parameter for cost that occurs when max_speed is exceeded
pslack=500;          % slack variable 
pslack2=1000000;        % collision cost
dist=3;            % min distance available

% Splines
pointsO = 23;      % N gokart parameters + N cost function parameters
pointsN = 10;      % N spline points ahead considered kart 1
pointsN2 = 10;     % N spline points ahead considered kart 2
pointsN3 = 10;     % N spline points ahead considered kart 3 
pointsN4 = 10;     % N spline points ahead considered kart 3
pointsN5 = 10;     % N spline points ahead considered kart 3 
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;
splinestart5 = 1;
% Stepsize
integrator_stepsize = 0.1;

% Simulation length
tend = 1;
eulersteps = 10;
planintervall = 1;

%% controller params %%RECOMPILE IF MODIFIED!
NUM_const=7; % number of nonlinear constraint
P_H_length=70; % prediction horizon length
MAX_IT= 500; % N of max iterations

% ds constraint (delta progress)
ds_max=5;
ds_min=-1;

% beta constraint (steering angle)
beta_max=0.5;
beta_min=-0.5;
%% Spline Points
points3 = [30,35,40,45,50,55,60,65,70,75,80,85,90;...          %x
           50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y
           3,3,3,3,3,3,3,3,3,3,3,3,3]';

points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x
           80,75,70,65,60,55,50,45,40,35,30,25,20,15,10; ...    %y
           3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]';

points = [50,50,50,50,50,50,50,50,50,48,45,40,35,30;...          %x
           90,85,80,75,70,65,60,55,52,50,50,50,50,50; ...    %y
           3,3,3,3,3,3,3,3,3,3,3,3,3,3]';
points(:,2)=points(:,2)-10;
points=flip(points);

points4 = [30,35,40,45,50,55,60,65,70,75,80,85,90;...          %x
          60,60,60,60,60,60,60,60,60,60,60,60,60; ...    %y
          3,3,3,3,3,3,3,3,3,3,3,3,3]';  
      
points5 = [70,65,60,55,50,45,40,35,30,25,20;...          %x
          40,40,40,40,40,40,40,40,40,40,40; ...    %y
          3,3,3,3,3,3,3,3,3,3,3]';
      

       

%        
solvetimes  = [];
solvetimes2 = [];
solvetimes3 = [];
solvetimes4 = [];
solvetimes5 = [];
trajectorytimestep = integrator_stepsize;
