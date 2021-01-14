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
%close all

% configuration
NUM_Vehicles = 3; %1,2,3,5
Compiled    = 'no'; % 'yes' or 'no', yes if code has already been compiled
ok=0;
ok1=0;
ok2=1;
Simulation  = 'yes';% 'yes' or 'no', no if you don't want to run simulation
TestAlpha1shot='no';% 'yes' or 'no', yes if you want to test alpha. 
                    % Simulation must be no, it requires compiled IBR and
                    % PG+alpha
LEPunisher  = 'yes'; % 'yes' or 'no' % Lateral Error Punisher (It Penalizes
                                    % only the left side of the centerline)
Condition   = 'cen'; % 'cen','dec'; 'dec' for 2 vehicles only
Game        = 'PG'; % 'PG'; 'IBR';
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
        parameters_3_vehicles_1
        switch Game
            case 'PG'
                %parameters_3_vehicles_1
                pointsO = 16; 
                NUM_const=12; % number of nonlinear constraint
                MAX_IT= 1000; % N of max iterations
            case 'IBR'
                pointsO = 20; 
                NUM_const=5; % number of nonlinear constraint
                MAX_IT= 500; % N of max iterations
        end
    otherwise
        error('Change NUM_Vehicle')
end

%% State and Input Definitions 
global index index1 index_IBR
if strcmp(TestAlpha1shot,'no')
    switch NUM_Vehicles
        case 3
            switch Game
                case 'PG'
                    indexes_3_vehicles
                    indexes_3_vehicles_1
                case 'IBR'
                    indexes_3_vehicles_IBR 
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
                               @(x,u,p)interstagedx_PG3(x,u),integrator_stepsize,p);
                    model1.eq = @(z,p) RK4(z(index1.sb:end), z(1:index1.nu),...
                               @(x,u,p)interstagedx_PG3_1(x,u),integrator_stepsize,p);
                case 'IBR'
                    model.eq = @(z,p) RK4(z(index_IBR.sb:end), z(1:index_IBR.nu),...
                               @(x,u,p)interstagedx_IBR(x,u),integrator_stepsize,p);
            end
    end
    
    model.E = [zeros(index.ns,index.nu), eye(index.ns)];
    model1.E = [zeros(index1.ns,index1.nu), eye(index1.ns)];
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
                            for i=1:model1.N
                                model1.objective{i} = @(z,p)objective_PG3_LE_1_T(z,...
                                    getPointsFromParameters(p, pointsO, pointsN),...
                                    getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                    getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                                    p(index1.ps),...
                                    p(index1.plag),...
                                    p(index1.plat),...
                                    p(index1.pprog),...
                                    p(index1.pab),...
                                    p(index1.pdotbeta),...
                                    p(index1.pspeedcost),...
                                    p(index1.pslack),...
                                    p(index1.pslack2));
                            end
%                             model.objectiveN = @(z,p) objective_PG3_LE_N(z,...
%                                     getPointsFromParameters(p, pointsO, pointsN),...
%                                     getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
%                                     getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
%                                     p(index.ps),...
%                                     p(index.plag),...
%                                     p(index.plat),...
%                                     p(index.pprog),...
%                                     p(index.pab),...
%                                     p(index.pdotbeta),...
%                                     p(index.pspeedcost),...
%                                     p(index.pslack),...
%                                     p(index.pslack2));
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
                                model.objective{i} = @(z,p)objective_IBR_LE(z,...
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

        % Acceleration
        model.lb(index_IBR.ab)=-inf;

        % Slack
        %model.lb(index_IBR.slack)=0;

        % Velocity
        model.lb(index_IBR.v)=0;
        model.ub(index_IBR.v)=maxSpeed;
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

        % Acceleration
        model.lb(index.ab)=-inf;

        % Slack
        %model.lb(index.slack)=0;

        % Velocity
        model.lb(index.v)=0;
        model.ub(index.v)=maxSpeed;
        % Steering Angle
        model.ub(index.beta)=beta_max;
        model.lb(index.beta)=beta_min;

        %Path Progress
        model.ub(index.s)=pointsN;
        model.lb(index.s)=0;
        
        
        model1.xinitidx = index1.sb:index1.nv;

        % initialization
        model1.ub = ones(1,index1.nv)*inf;
        model1.lb = -ones(1,index1.nv)*inf;

        % Delta path progress
        model1.ub(index1.ds)=ds_max;
        model1.lb(index1.ds)=ds_min;

        % Acceleration
        model1.lb(index1.ab)=-inf;

        % Slack
        %model.lb(index.slack)=0;

        % Velocity
        model1.lb(index1.v)=0;
        model1.ub(index1.v)=maxSpeed_1;
        % Steering Angle
        model1.ub(index1.beta)=beta_max;
        model1.lb(index1.beta)=beta_min;

        %Path Progress
        model1.ub(index1.s)=pointsN;
        model1.lb(index1.s)=0;
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
                    model.ub(index.v_k2)=maxSpeed;
                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k2)=beta_max;
                    model.lb(index.beta_k2)=beta_min;

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
                    model.ub(index.v_k3)=maxSpeed;
                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k3)=beta_max;
                    model.lb(index.beta_k3)=beta_min;

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
                                0;0;0];%
                    model.hl = [-inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf;-inf;-inf];%
                    model.nhN = NUM_const+3; 
                    model.ineqN= @(z,p) nlconst_PG3_N(z,p);
                    model.huN = [0;30;0;0;...
                                 0;+inf;0;0;...
                                 0;+inf;0;0;...;
                                 0;0;0];%
                    model.hlN = [-inf;-inf;-inf;-inf;...
                                 -inf;70;-inf;-inf;...
                                 -inf;70;-inf;-inf;...
                                 -inf;-inf;-inf];%
                    %%Controller 2
                    model1.ub(index1.ds_k2)=ds_max;
                    model1.lb(index1.ds_k2)=ds_min;

                    % Acceleration Constraint (input)
                    model1.lb(index1.ab_k2)=-inf;

                    % Slack Variables Constraint (input)
                    %model.lb(index.slack_k2)=0;

                    % Speed Constraint (state)
                    model1.lb(index1.v_k2)=0;
                    model1.ub(index1.v_k2)=maxSpeed_1;
                    % Steering Angle Constraint (input)
                    model1.ub(index1.beta_k2)=beta_max;
                    model1.lb(index1.beta_k2)=beta_min;

                    % Path Progress Constraint (input)
                    model1.ub(index1.s_k2)=pointsN2;
                    model1.lb(index1.s_k2)=0;

                    % Path Progress rate Constraint (input)
                    model1.ub(index1.ds_k3)=ds_max;
                    model1.lb(index1.ds_k3)=ds_min;

                    % Acceleration Constraint (input)
                    model1.lb(index1.ab_k3)=-inf;

                    % Slack Variables Constraint (input)
                    %model.lb(index.slack_k3)=0;

                    % Speed Constraint (state)
                    model1.lb(index1.v_k3)=0;
                    model1.ub(index1.v_k3)=maxSpeed_1;
                    % Steering Angle Constraint (input)
                    model1.ub(index1.beta_k3)=beta_max;
                    model1.lb(index1.beta_k3)=beta_min;

                    % Path Progress Constraint (input)
                    model1.ub(index1.s_k3)=pointsN3;
                    model1.lb(index1.s_k3)=0;

                    model1.lb(index1.slack2)=0;
                    model1.lb(index1.slack3)=0;
                    model1.lb(index1.slack4)=0;

                    %limit lateral acceleration
                    model1.nh = NUM_const+1; 
                    model1.ineq = @(z,p) nlconst_PG3_1(z,p);
                    model1.hu = [0;0;0;0;...
                                0;0;0;...
                                0;0;0;...
                                0;0;0];%
                    model1.hl = [-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf;-inf;-inf];%
                    model1.nhN = NUM_const+4; 
                    model1.ineqN= @(z,p) nlconst_PG3_N_1(z,p);
                    model1.huN = [0;0;35;0;0;...
                                 0;+inf;0;0;...
                                 0;+inf;0;0;...;
                                 0;0;0];%
                    model1.hlN = [-inf;-inf;-inf;-inf;-inf;...
                                 -inf;60;-inf;-inf;...
                                 -inf;60;-inf;-inf;...
                                 -inf;-inf;-inf];%
                             
                    %% Third Controller
                    model2=model1;
                    model2.nh = NUM_const+2; 
                    model2.ineq = @(z,p) nlconst_PG3_2(z,p);
                    model2.hu = [0;0;0;0;0;...
                                0;0;0;...
                                0;0;0;...
                                0;0;0];%
                    model2.hl = [-inf;-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf;-inf;-inf];%
                    model2.nhN = NUM_const+5; 
                    model2.ineqN= @(z,p) nlconst_PG3_N_2(z,p);
                    model2.huN = [0;0;0;35;0;0;...
                                 0;+inf;0;0;...
                                 0;+inf;0;0;...;
                                 0;0;0];%
                    model2.hlN = [-inf;-inf;-inf;-inf;-inf;-inf;...
                                 -inf;60;-inf;-inf;...
                                 -inf;60;-inf;-inf;...
                                 -inf;-inf;-inf];%
                             
            end
    end

    %% Solver
    switch NUM_Vehicles
        case 3
            switch Game
                case 'PG'
                    codeoptions = getOptions('MPCPathFollowing_3v');
                    codeoptions1 = getOptions('MPCPathFollowing_3v_1');
                    codeoptions2 = getOptions('MPCPathFollowing_3v_2');
                case 'IBR'
                    codeoptions = getOptions('MPCPathFollowing_3v_IBR');
            end
    end
    
    codeoptions.maxit = MAX_IT;    % Maximum number of iterations
    codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
    codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
    codeoptions.cleanup = false;
    codeoptions.timing = 1;
    output = newOutput('alldata', 1:model.N, 1:model.nvar);
    if ok==1
    FORCES_NLP(model, codeoptions,output);
    end
    codeoptions1.maxit = MAX_IT;    % Maximum number of iterations
    codeoptions1.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
    codeoptions1.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
    codeoptions1.cleanup = false;
    codeoptions1.timing = 1;
    output1 = newOutput('alldata', 1:model1.N, 1:model1.nvar);
    if ok1==1
    FORCES_NLP(model1, codeoptions1,output1);
    end
    codeoptions2.maxit = MAX_IT;    % Maximum number of iterations
    codeoptions2.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
    codeoptions2.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
    codeoptions2.cleanup = false;
    codeoptions2.timing = 1;
    output2 = newOutput('alldata', 1:model1.N, 1:model1.nvar);
    if ok2==1
    FORCES_NLP(model2, codeoptions2,output2);
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
                    Run_3_vehicles_IBR
                end
        end
end