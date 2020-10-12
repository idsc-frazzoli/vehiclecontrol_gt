% go kart parameters
maxSpeed = 9;%10;
targetSpeed=8.3;
maxxacc = 4;
maxyacc = 8;
latacclim = 6;
rotacceffect  = 2;
torqueveceffect = 3;
brakeeffect = 0; 

% cost function parameters
plagerror=1;       % proj error param cost
platerror=1.8;    % lateral error param cost
pprog=0.8;         % progress parameter cost
pab=0.006;        % acc parameter cost
pdotbeta=5;      % steering velocity parameter cost
pspeedcost=2;    % parameter for cost that occurs when max_speed is exceeded
pslack=500;          % slack variable 
pslack2=10000;        % collision cost
dist=4;   
% 
% if Stack==1
%     plagerror=1;       % proj error param cost
%     platerror=2.4;    % lateral error param cost
%     pprog=0.8;         % progress parameter cost
%     pab=0.006;        % acc parameter cost
%     pdotbeta=5;      % steering velocity parameter cost
%     pspeedcost=2;    % parameter for cost that occurs when max_speed is exceeded
%     pslack=500;          % slack variable 
%     pslack2=1000000;        % collision cost
%     dist=4;   
% end
% platerror=5;    % lateral error param cost
% pprog=0;         % progress parameter cost
% pab=0.000004;        % acc parameter cost
% pdotbeta=2;      % steering velocity parameter cost
% pspeedcost=0.08;    % parameter for cost that occurs when max_speed is exceeded
% pslack=10000;          % slack variable 
% pslack2=10000;        % collision cost
% dist=4;            % min distance available

% alpha parameters for CF (only if alpha condition active, update pointsO)
alpha1=0.5;
alpha2=1;
alpha3=1;

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
beta_max=0.5;
beta_min=-0.5;
%% Spline Points

points = [10,15,20,25,30,35,40,45,50,55,60,65,70,75;...          %x,75,80,85,90,95
          50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y,50,50,50,50,50
          1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]'; %,5,5,5,5,5  
points(:,2)=points(:,2)+1.75;
points=flip(points);
points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50
          25,30,35,40,45,50,55,60,65,70,75,80,85,90; ...    %y,75,80,85,90,95
          1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]';  %,5,5,5,5,5
points2(:,1)=points2(:,1)+1.75;
% points3 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50,50,65,65,50
%            75,70,65,60,55,50,45,40,35,30,25,20,15,10; ...    %y,25,20,15,10,5,0,5,92,95
%            1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]';%,5,5,5,5,4,3,2,2,3
% points3(:,1)=points3(:,1)-1.75;  
 
points3 = [48.25,48.25,48.25,48.25,48.25,48.25,52,55,60,65,70,75,80,85;...          %x,50,50,50,50,50,50,65,65,50
           75,70,65,60,55,52,48.25,48.25,48.25,48.25,48.25,48.25,48.25,48.25; ...    %y,25,20,15,10,5,0,5,92,95
           1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]';%,5,5,5,5,4,3,2,2,3
%points3(:,1)=points3(:,1)-1.75;  
solvetimes  = [];
solvetimes2 = [];
solvetimes3 = [];
trajectorytimestep = integrator_stepsize;
