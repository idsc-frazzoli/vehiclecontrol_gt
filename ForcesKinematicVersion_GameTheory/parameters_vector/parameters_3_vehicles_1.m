% % go kart parameters
% maxSpeed = 20;%10;
% % cost function parameters
% plagerror=1;       % proj error param cost
% platerror=0;    % lateral error param cost
% pprog=0;         % progress parameter cost
% pab=0;        % acc parameter cost
% pdotbeta=0;      % steering velocity parameter cost
% pspeedcost=0;    % parameter for cost that occurs when max_speed is exceeded
% pslack=0.1;          % slack variable 
% pslack2=10000;        % collision cost
% dist=4;   
% % cost function parameters
% plagerror_1=1;       % proj error param cost
% platerror_1=1;    % lateral error param cost
% pprog_1=0.8;         % progress parameter cost
% pab_1=0.006;        % acc parameter cost
% pdotbeta_1=4;      % steering velocity parameter cost
% pspeedcost_1=2;    % parameter for cost that occurs when max_speed is exceeded
% pslack_1=0.1;          % slack variable 
% pslack2_1=10000;        % collision cost
% go kart parameters
params.maxSpeed = 9;%10;
params.maxSpeed_1 = 20;
params.targetSpeed=8.3;
params.posX4=53;

% cost function parameters
params.plagerror=0.3;       % proj error param cost
params.platerror=2;    % lateral error param cost
params.pprog=0.8;         % progress parameter cost
params.pab=0.06;        % acc parameter cost
params.pdotbeta=4;      % steering velocity parameter cost
params.pspeedcost=2;    % parameter for cost that occurs when max_speed is exceeded
params.pSpeedMax=1;
params.PprogMax=8;
params.pslack=2;          % slack variable 
params.pslack2=100;        % collision cost
params.dist=4;   

% Splines
pointsO = 16;      % N gokart parameters + N cost function parameters
pointsN = 14;      % N spline points ahead considered kart 1
pointsN2 = 14;     % N spline points ahead considered kart 2
pointsN3 = 14;     % N spline points ahead considered kart 3 


%% Spline Points
points = [10,15,20,25,30,35,40,45,50,55,60,65,70,75;...          %x,75,80,85,90,95
          50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y,50,50,50,50,50
          3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]'; %,5,5,5,5,5  
%points(:,2)=points(:,2)+1.75;
points=flip(points);
points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50
          25,30,35,40,45,50,55,60,65,70,75,80,85,90; ...    %y,75,80,85,90,95
          3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]';  %,5,5,5,5,5
%points2(:,1)=points2(:,1)+1.75;
% points3 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50,50,65,65,50
%            75,70,65,60,55,50,45,40,35,30,25,20,15,10; ...    %y,25,20,15,10,5,0,5,92,95
%            1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]';%,5,5,5,5,4,3,2,2,3
% points3(:,1)=points3(:,1)-1.75;  
 
points3 = [50,50,50,50,50,50,52,55,60,65,70,75,80,85;...          %x,50,50,50,50,50,50,65,65,50
           75,70,65,60,55,52,50,50,50,50,50,50,50,50; ...    %y,25,20,15,10,5,0,5,92,95
           3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]';%,5,5,5,5,4,3,2,2,3
%points3(:,1)=points3(:,1)-1.75;  

% % go kart parameters
% maxSpeed_1 = 9;%10;
% targetSpeed=8.3;
% maxxacc = 4;
% maxyacc = 8;
% latacclim = 6;
% rotacceffect  = 2;
% torqueveceffect = 3;
% brakeeffect = 0; 
% 
% % Splines
% pointsO_1 = 16;      % N gokart parameters + N cost function parameters
% pointsN_1 = 14;      % N spline points ahead considered kart 1
% pointsN2_1 = 14;     % N spline points ahead considered kart 2
% pointsN3_1 = 14;     % N spline points ahead considered kart 3 
splinestart = 1;
splinestart2 = 1;
splinestart3 = 1;
splinestart4 = 1;
% % Stepsize
integrator_stepsize = 0.1;
params.integrator_stepsize=integrator_stepsize;
% 
% % Simulation length
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

solvetimes  = [];
solvetimes2 = [];
solvetimes3 = [];
trajectorytimestep = integrator_stepsize;
