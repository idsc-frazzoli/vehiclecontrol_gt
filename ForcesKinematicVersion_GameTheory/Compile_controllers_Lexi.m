%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Game Theory Controller Generation and Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% code by em
% This code allows to represent Receding Horizon IBR and Potential Games,
% applied to N vehicles running in the same track, (intersections or 
% others) streets in without any constraints on collisions

%add paths (change that for yourself, remember to add Forces)
addpath('..');
userDir = getuserdir;
addpath('casadi');
addpath('models');  
addpath('draw_files');
addpath('parameters_vector');
addpath('objective_function');
addpath('constraints');
addpath('index_script');
addpath('Run_Simulation');
addpath('Animation');
addpath('ReadSVG');
%addpath('svgread');
clear model
clear problem
clear all
close all

% configuration
NUM_Vehicles = 3; %1,2,3,5
Compiled    = 'yes'; % 'yes' or 'no', yes if code has already been compiled
ok=1;
ok1=1;
ok2=1;

Simulation  = 'yes';% 'yes' or 'no', no if you don't want to run simulation
TestAlpha1shot='no';% 'yes' or 'no', yes if you want to test alpha. 
                    % Simulation must be no, it requires compiled IBR and
                    % PG+alpha
LEPunisher  = 'yes'; % 'yes' or 'no' % Lateral Error Punisher (It Penalizes
                                    % only the left side of the centerline)
Condition   = 'cen'; % 'cen','dec'; 'dec' for 2 vehicles only
Game        = 'IBR'; % 'PG'; 'IBR';
Alpha       = 'no'; % 'yes' (2 vehicles, 'cen' condition and PG only), 'no';
Stack       = 0;
if (strcmp(Alpha,'yes') || strcmp(TestAlpha1shot,'yes')) && NUM_Vehicles~=2
    warning('Configuration not supported, change NUM_Vehicles or Alpha')
    return
end
if strcmp(Condition,'dec') && NUM_Vehicles~=2
    warning('Configuration not supported, change NUM_Vehicles or Condition')
    return
end
if strcmp(Simulation,'yes') && strcmp(TestAlpha1shot,'yes')
    warning('Configuration not supported, change Simulation or TestAlpha1shot')
    return
end
%% Parameters Definitions (parameters_vector folder)
switch NUM_Vehicles
    case 3
        %parameters_3_vehicles_1
        switch Game
            case 'PG'
                parameters_3_vehicles
                pointsO = 16; 
                NUM_const=15; % number of nonlinear constraint
                MAX_IT= 1000; % N of max iterations
            case 'IBR'
                parameters_3_vehicles
                pointsO = 20; 
                NUM_const=6; % number of nonlinear constraint
                MAX_IT= 500; % N of max iterations
                %plagerror=1;       % proj error param cost
        end
    otherwise
        error('Change NUM_Vehicle')
end

%% State and Input Definitions 
global index index_IBR 
if strcmp(TestAlpha1shot,'no')
    switch NUM_Vehicles
        case 3
            switch Game
                case 'PG'
                    indexes_3_vehicles
                   % indexes_3_vehicles_1
                case 'IBR'
                    indexes_3_vehicles_IBR 
                    %indexes_3_vehicles_IBR_1
            end
    end
else
    index_2_vehicles_TestAlpha
end

if strcmp(Compiled,'no')
%% Model Definition (models folder)
% if you change interstagedx file, remember to change also the file in
% simulation!!!!
    switch NUM_Vehicles
        case 3
            switch Game
                case 'PG'
                    model.eq = @(z,p) RK4(z(index.sb:end), z(1:index.nu),...
                               @(x,u,p)interstagedx_PG3(x,u,p),integrator_stepsize,p);
                case 'IBR'
                    model.eq = @(z,p) RK4(z(index_IBR.sb:end), z(1:index_IBR.nu), ...
                               @(x,u,p)interstagedx_IBR(x,u,p),integrator_stepsize,p);
            end
    end
    
    if strcmp(Game,'IBR')
        model.E = [zeros(index_IBR.ns,index_IBR.nu), eye(index_IBR.ns)];
%         model1.E = [zeros(index_IBR.ns,index_IBR.nu), eye(index_IBR.ns)];
%         model2.E = [zeros(index_IBR_1.ns,index_IBR_1.nu), eye(index_IBR_1.ns)];
    else
        model.E = [zeros(index.ns,index.nu), eye(index.ns)];
%         model1.E = [zeros(index.ns,index.nu), eye(index.ns)];
%         model2.E = [zeros(index1.ns,index1.nu), eye(index1.ns)];
    end
    
%% Objective Function (objective_function folder)
    switch NUM_Vehicles
        case 3
            switch Game
                case 'PG'
                    switch LEPunisher
                        case 'no'
                            for i=1:model.N
                                model.objective{i} = @(z,p)objective_PG3_1(z,...
                                    getPointsFromParameters(p, pointsO, pointsN),...
                                    getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                    getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                                    p(index.ps),...
                                    p(index.plag),...
                                    p(index.plat),...
                                    p(index.pprog),...
                                    p(index.pab),...
                                    p(index.pdotbeta),...
                                    p(index.pspeedcost),...
                                    p(index.pslack),...
                                    p(index.pslack2));
                            end
                        case 'yes'
                            for i=1:model.N
                                model.objective{i} = @(z,p)objective_PG3_LE_1(z,...
                                    getPointsFromParameters(p, pointsO, pointsN),...
                                    getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                    getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                                    p(index.SpeedMax),...
                                    p(index.ps),...
                                    p(index.plag),...
                                    p(index.plat),...
                                    p(index.pprog),...
                                    p(index.pab),...
                                    p(index.pdotbeta),...
                                    p(index.pspeedcost),...
                                    p(index.pSpeedMax),...
                                    p(index.pslack),...
                                    p(index.pslack2));
                            end

                    end
                case 'IBR'
                    switch LEPunisher
                        case 'no'
                            for i=1:model.N
                                model.objective{i} = @(z,p)objective_IBR(z,...
                                    getPointsFromParameters(p, pointsO, pointsN),...
                                    p(index_IBR.ps),...
                                    p(index_IBR.plag),...
                                    p(index_IBR.plat),...
                                    p(index_IBR.pprog),...
                                    p(index_IBR.pab),...
                                    p(index_IBR.pdotbeta),...
                                    p(index_IBR.pspeedcost),...
                                    p(index_IBR.pslack),...
                                    p(index_IBR.pslack2));
                            end
                        case 'yes'
                            for i=1:model.N
                                model.objective{i} = @(z,p)objective_IBR_LE_1(z,...
                                    getPointsFromParameters(p, pointsO, pointsN),...
                                    p(index_IBR.SpeedMax),...
                                    p(index_IBR.ps),...
                                    p(index_IBR.plag),...
                                    p(index_IBR.plat),...
                                    p(index_IBR.pprog),...
                                    p(index_IBR.pab),...
                                    p(index_IBR.pdotbeta),...
                                    p(index_IBR.pspeedcost),...
                                    p(index_IBR.pSpeedMax),...
                                    p(index_IBR.pslack),...
                                    p(index_IBR.pslack2));
                            end
%                             
                    end
            end
    end
    
    %% Linear and NON-Linear Constraints
    if strcmp(Game,'IBR') && NUM_Vehicles>=2
        model.xinitidx = index_IBR.sb:index_IBR.nv;

        % initialization
        model.ub = ones(1,index_IBR.nv)*inf;
        model.lb = -ones(1,index_IBR.nv)*inf;

        %Delta path progress
        model.ub(index_IBR.ds)=ds_max;
        model.lb(index_IBR.ds)=ds_min;
        model.ub(index_IBR.dotab)=0.5;
        model.lb(index_IBR.dotab)=-1;
        % Acceleration
        model.lb(index_IBR.ab)=-inf;

        % Slack
        %model.lb(index_IBR.slack)=0;

        % Velocity
        model.lb(index_IBR.v)=0;
        model.ub(index_IBR.v)=params.maxSpeed_1;%maxSpeed
        % Steering Angle
        model.ub(index_IBR.beta)=beta_max;
        model.lb(index_IBR.beta)=beta_min;

        %Path Progress
        model.ub(index_IBR.s)=pointsN;
        model.lb(index_IBR.s)=0;
        % Slack
        model.lb(index_IBR.slack2)=0;
        
    else
        model.xinitidx = index.sb:index.nv;
        
        % initialization
        model.ub = ones(1,index.nv)*inf;
        model.lb = -ones(1,index.nv)*inf;
        
        % Delta path progress
        model.ub(index.ds)=ds_max;
        model.lb(index.ds)=ds_min;
        model.ub(index.dotab)=0.5;
        model.lb(index.dotab)=-1;
        % Acceleration
        model.lb(index.ab)=-inf;

        % Slack
        %model.lb(index.slack)=0;

        % Velocity
        model.lb(index.v)=0;
        model.ub(index.v)=params.maxSpeed_1;
        % Steering Angle
        model.ub(index.beta)=beta_max;
        model.lb(index.beta)=beta_min;

        %Path Progress
        model.ub(index.s)=pointsN;
        model.lb(index.s)=0;
        
    end
    switch NUM_Vehicles
       case 3
            switch Game
                case 'PG'
                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k2)=ds_max;
                    model.lb(index.ds_k2)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k2)=-inf;

                    % Slack Variables Constraint (input)
                    %model.lb(index.slack_k2)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k2)=0;
                    model.ub(index.v_k2)=params.maxSpeed_1;
                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k2)=beta_max;
                    model.lb(index.beta_k2)=beta_min;
                    model.ub(index.dotab_k2)=0.5;
                    model.lb(index.dotab_k2)=-1;
                    % Path Progress Constraint (input)
                    model.ub(index.s_k2)=pointsN2;
                    model.lb(index.s_k2)=0;

                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k3)=ds_max;
                    model.lb(index.ds_k3)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k3)=-inf;

                    % Slack Variables Constraint (input)
                    %model.lb(index.slack_k3)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k3)=0;
                    model.ub(index.v_k3)=params.maxSpeed_1;
                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k3)=beta_max;
                    model.lb(index.beta_k3)=beta_min;
                    model.ub(index.dotab_k3)=0.5;
                    model.lb(index.dotab_k3)=-1;
                    % Path Progress Constraint (input)
                    model.ub(index.s_k3)=pointsN3;
                    model.lb(index.s_k3)=0;

                    model.lb(index.slack2)=0;
                    model.lb(index.slack3)=0;
                    model.lb(index.slack4)=0;

                    %limit lateral acceleration
                    model.nh = NUM_const; 
                    model.ineq = @(z,p) nlconst_PG3(z,p);
                    model.hu = [0;0;0;...
                                0;0;0;...
                                0;0;0;...
                                0;0;0;0;0;0];%
                    model.hl = [-inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;-inf;-inf;];%
                    model2=model;
                    model3=model;
                    
                    model.nhN = NUM_const+3; 
                    model.ineqN= @(z,p) nlconst_PG3_N(z,p);
                    model.huN = [0;0;0;0;...
                                 0;0;0;0;...
                                 0;0;0;0;...;
                                 0;0;0;0;0;0];%
                    model.hlN = [-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;-inf;-inf];%
                            
                    model2.nhN = NUM_const+4; 
                    model2.ineqN= @(z,p) nlconst_PG3_N_1(z,p);
                    model2.huN = [0;0;0;0;0;...
                                 0;0;0;0;...
                                 0;0;0;0;...;
                                 0;0;0;0;0;0];%
                    model2.hlN = [-inf;-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;-inf;-inf];%
                            
                    model3.nhN = NUM_const+5; 
                    model3.ineqN= @(z,p) nlconst_PG3_N_2(z,p);
                    model3.huN = [0;0;0;0;0;0;...
                                 0;0;0;0;...
                                 0;0;0;0;...;
                                 0;0;0;0;0;0];%
                    model3.hlN = [-inf;-inf;-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;-inf;-inf];%         
                            
            case 'IBR'
                
                
                %% First                              
                model.nh = NUM_const; 
                model.ineq = @(z,p) nlconst_IBR_3(z,p);
                model.hu = [0;0;0;0;0;0];%
                model.hl = [-inf;-inf;-inf;-inf;-inf;-inf];%
                
                model2=model;
                model3=model;

                model.nhN = NUM_const+1; 
                model.ineqN= @(z,p) nlconst_IBR_3_N(z,p);
                model.huN = [0;0;0;0;0;0;0];%
                model.hlN = [-inf;-inf;-inf;-inf;-inf;-inf;-inf];%
%                 
                model2.nhN = NUM_const+2; 
                model2.ineqN= @(z,p) nlconst_IBR_3_N_1(z,p);
                model2.huN = [0;0;0;0;0;0;0;0];%
                model2.hlN = [-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;];%
               
                model3.nhN = NUM_const+3; 
                model3.ineqN= @(z,p) nlconst_IBR_3_N_2(z,p);
                model3.huN = [0;0;0;0;0;0;0;0;0];%
                model3.hlN = [-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf];% 
                
            end
    end

    %% Solver
    switch NUM_Vehicles
        case 3
            switch Game
                case 'PG'
                    codeoptions = getOptions('MPCPathFollowing_3v');
                    codeoptions2 = getOptions('MPCPathFollowing_3v_2');
                    codeoptions3 = getOptions('MPCPathFollowing_3v_3');
                    codeoptions.maxit = MAX_IT;    % Maximum number of iterations
                    codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
                    codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
                    codeoptions.cleanup = false;
                    codeoptions.timing = 1;
                    codeoptions.BuildSimulinkBlock = 0;
                    codeoptions.overwrite = 1;
                    codeoptions.noVariableElimination = 1; 
                    output = newOutput('alldata', 1:model.N, 1:model.nvar);
                    if ok==1
                    FORCES_NLP(model, codeoptions,output);
                    end
                    codeoptions2.maxit = MAX_IT;    % Maximum number of iterations
                    codeoptions2.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
                    codeoptions2.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
                    codeoptions2.cleanup = false;
                    codeoptions2.timing = 1;
                    codeoptions2.BuildSimulinkBlock = 0;
                    codeoptions2.overwrite = 1;
                    codeoptions2.noVariableElimination = 1;
                    output2 = newOutput('alldata', 1:model2.N, 1:model2.nvar);
                    if ok1==1
                    FORCES_NLP(model2, codeoptions2,output2);
                    end
                    codeoptions3.maxit = MAX_IT;    % Maximum number of iterations
                    codeoptions3.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
                    codeoptions3.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
                    codeoptions3.cleanup = false;
                    codeoptions3.timing = 1;
                    codeoptions3.BuildSimulinkBlock = 0;
                    codeoptions3.overwrite = 1;
                    codeoptions3.noVariableElimination = 1;
                    output3 = newOutput('alldata', 1:model3.N, 1:model3.nvar);
                    if ok2==1
                    FORCES_NLP(model3, codeoptions3,output3);
                    end
                    
                case 'IBR'
                    codeoptions = getOptions('MPCPathFollowing_3v_IBR');
                    codeoptions2 = getOptions('MPCPathFollowing_3v_IBR_2');
                    codeoptions3 = getOptions('MPCPathFollowing_3v_IBR_3');

                    codeoptions.maxit = MAX_IT; % Maximum number of iterations
                    codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
                    codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
                    codeoptions.cleanup = false;
                    codeoptions.timing = 1;
                    codeoptions.BuildSimulinkBlock = 0;
                    codeoptions.overwrite = 1;
                    codeoptions.noVariableElimination = 1; 
                    output = newOutput('alldata', 1:model.N, 1:model.nvar);
                    if ok==1
                    FORCES_NLP(model, codeoptions,output);
                    end
                    codeoptions2.maxit = MAX_IT; % Maximum number of iterations
                    codeoptions2.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
                    codeoptions2.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
                    codeoptions2.cleanup = false;
                    codeoptions2.timing = 1; 
                    codeoptions2.BuildSimulinkBlock = 0;
                    codeoptions2.overwrite = 1;
                    codeoptions2.noVariableElimination = 1; 
                    output2 = newOutput('alldata', 1:model2.N, 1:model2.nvar);
                    if ok1==1
                    FORCES_NLP(model2, codeoptions2,output2);
                    end
                    codeoptions3.maxit = MAX_IT; % Maximum number of iterations
                    codeoptions3.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
                    codeoptions3.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
                    codeoptions3.cleanup = false;
                    codeoptions3.timing = 1;
                    codeoptions3.BuildSimulinkBlock = 0;
                    codeoptions3.overwrite = 1;
                    codeoptions3.noVariableElimination = 1; 
                    output3 = newOutput('alldata', 1:model3.N, 1:model3.nvar);
                    if ok2==1
                    FORCES_NLP(model3, codeoptions3,output3);
                    end
            end
    end
    
   
end

%% Run simulation 

switch NUM_Vehicles
    case 3
        switch Game
            case 'PG'
                %Run_3_vehicles_cen_1
                Run_3_vehicles_cen_Lexi;%_1 ;%
            case 'IBR'
                if Stack==1
                    Run_3_vehicles_IBR_st
                else
                    %Run_3_vehicles_IBR
                    Run_3_vehicles_IBR_LEXI
                end
        end
end