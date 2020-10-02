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
clear model
clear problem
clear all
%close all

% configuration
NUM_Vehicles = 5; %1,2,3,5
Compiled    = 'no'; % 'yes' or 'no', yes if code has already been compiled
Simulation  = 'yes';% 'yes' or 'no', no if you don't want to run simulation
TestAlpha1shot='no';% 'yes' or 'no', yes if you want to test alpha. 
                    % Simulation must be no, it requires compiled IBR and
                    % PG+alpha
LEPunisher  = 'yes'; % 'yes' or 'no' % Lateral Error Punisher (It Penalizes
                                    % only the left side of the centerline)
Condition   = 'cen'; % 'cen','dec'; 'dec' for 2 vehicles only
Game        = 'PG'; % 'PG'; 'IBR' has simulation for 'dec' only.
Alpha       = 'no'; % 'yes' (2 vehicles, 'cen' condition and PG only), 'no';

if (strcmp(Alpha,'yes') || strcmp(TestAlpha1shot,'yes')) && NUM_Vehicles~=2
    warning('Configuration not supported, change NUM_Vehicles or Alpha')
    return
end
if strcmp(Condition,'dec') && NUM_Vehicles~=2
    warning('Configuration not supported, change NUM_Vehicles or Condition')
    return
end
if strcmp(Condition,'cen') && strcmp(Game,'IBR') && NUM_Vehicles==2
    warning('Configuration not supported, change NUM_Vehicles or Condition or Game')
    return
end
%% Parameters Definitions (parameters_vector folder)
switch NUM_Vehicles
    case 1
        parameters_1_vehicle

    case 2
        if strcmp(TestAlpha1shot,'no')
            parameters_2_vehicles
            % for alpha case
            if strcmp(Alpha,'yes')
                pointsO=18;
            end
            if strcmp(Game,'IBR')
                pointsO=18;
                NUM_const=4;
            end
        else
            parameters_2_vehicles_TestAlpha
        end
    case 3
        switch Game
            case 'PG'
                parameters_3_vehicles
            case 'IBR'
                parameters_3_vehicles_IBR
        end
    case 5
        switch Game
            case 'PG'
                parameters_5_vehicles
            case 'IBR'
                parameters_5_vehicles_IBR
        end
    otherwise
        error('Change NUM_Vehicle')
end

%% State and Input Definitions 
global index index_IBR
switch NUM_Vehicles
    case 1
        indexes_1_vehicle
    case 2
        switch Game
            case 'IBR'
                indexes_2_vehicles_IBR
            case 'PG'
                switch Alpha
                    case 'no'
                        indexes_2_vehicles
                    case 'yes'
                        if strcmp(TestAlpha1shot,'no')
                            indexes_2_vehicles_alpha
                        else
                            index_2_vehicles_TestAlpha
                            
                        end
                    otherwise
                        error('Change Alpha')
                end
            otherwise
                error('Change Game')
        end
    case 3
        switch Game
            case 'PG'
                indexes_3_vehicles
            case 'IBR'
                indexes_3_vehicles_IBR 
        end
    case 5
        switch Game
            case 'PG'
                indexes_5_vehicles
            case 'IBR'
                indexes_5_vehicles_IBR
       end
end

if strcmp(Compiled,'no')
%% Model Definition (models folder)
% if you change interstagedx file, remember to change also the file in
% simulation!!!!
    switch NUM_Vehicles
        case 1
            model.eq = @(z,p) RK4(z(index.sb:end), z(1:index.nu),...
                       @(x,u,p)interstagedx(x,u), integrator_stepsize,p);

        case 2
            switch Game
                case 'PG'
                        model.eq = @(z,p) RK4(z(index.sb:end), z(1:index.nu),...
                               @(x,u,p)interstagedx_PG(x,u),integrator_stepsize,p);
                case 'IBR'
                        model.eq = @(z,p) RK4(z(index_IBR.sb:end), z(1:index_IBR.nu),...
                               @(x,u,p)interstagedx_IBR(x,u),integrator_stepsize,p);
                otherwise
                    error('Change Game')
            end

        case 3
            switch Game
                case 'PG'
                    model.eq = @(z,p) RK4(z(index.sb:end), z(1:index.nu),...
                               @(x,u,p)interstagedx_PG3(x,u),integrator_stepsize,p);
                case 'IBR'
                    model.eq = @(z,p) RK4(z(index_IBR.sb:end), z(1:index_IBR.nu),...
                               @(x,u,p)interstagedx_IBR(x,u),integrator_stepsize,p);
                
            end
        case 5
            switch Game
                case 'PG'
                    model.eq = @(z,p) RK4(z(index.sb:end), z(1:index.nu),...
                               @(x,u,p)interstagedx_PG5(x,u),integrator_stepsize,p);
                case 'IBR'
                    model.eq = @(z,p) RK4(z(index_IBR.sb:end), z(1:index_IBR.nu),...
                               @(x,u,p)interstagedx_IBR(x,u),integrator_stepsize,p);
            end
    end
    
    if strcmp(Game,'IBR') && NUM_Vehicles>=2
        model.E = [zeros(index_IBR.ns,index_IBR.nu), eye(index_IBR.ns)];
    else
        model.E = [zeros(index.ns,index.nu), eye(index.ns)];
    end
    
%% Objective Function (objective_function folder)
    switch NUM_Vehicles
        case 1
            switch LEPunisher
                case 'no'
                    for i=1:model.N
                        model.objective{i} = @(z,p)objective(z,...
                            getPointsFromParameters(p, pointsO, pointsN),...
                            p(index.ps),...
                            p(index.plag),...
                            p(index.plat),...
                            p(index.pprog),...
                            p(index.pab),...
                            p(index.pdotbeta),...
                            p(index.pspeedcost),...
                            p(index.pslack));
                    end
                case 'yes'
                    for i=1:model.N
                        model.objective{i} = @(z,p)objective_LE(z,...
                            getPointsFromParameters(p, pointsO, pointsN),...
                            p(index.ps),...
                            p(index.plag),...
                            p(index.plat),...
                            p(index.pprog),...
                            p(index.pab),...
                            p(index.pdotbeta),...
                            p(index.pspeedcost),...
                            p(index.pslack));
                    end
            end
        case 2
            switch Alpha
                case 'no'
                    switch Game
                        case 'PG'
                            switch LEPunisher
                                case 'no'
                                    for i=1:model.N
                                        model.objective{i} = @(z,p)objective_PG(z,...
                                        getPointsFromParameters(p, pointsO, pointsN),...
                                        getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
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
                                        model.objective{i} = @(z,p)objective_PG_LE(z,...
                                        getPointsFromParameters(p, pointsO, pointsN),...
                                        getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
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
                case 'yes'
                    switch Game
                        case 'PG'
                            switch LEPunisher
                                case 'no'
                                    for i=1:model.N
                                        model.objective{i} = @(z,p)objective_PG_alpha(z,... % objective_PG_alpha
                                            getPointsFromParameters(p, pointsO, pointsN),...
                                            getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                            p(index.ps),...
                                            p(index.plag),...
                                            p(index.plat),...
                                            p(index.pprog),...
                                            p(index.pab),...
                                            p(index.pdotbeta),...
                                            p(index.pspeedcost),...
                                            p(index.pslack),...
                                            p(index.pslack2),...
                                            p(index.alpha1),...
                                            p(index.alpha2));
                                    end
                                case 'yes'
                                    for i=1:model.N
                                        model.objective{i} = @(z,p)objective_PG_alpha_LE(z,... % objective_PG_alpha
                                            getPointsFromParameters(p, pointsO, pointsN),...
                                            getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                            p(index.ps),...
                                            p(index.plag),...
                                            p(index.plat),...
                                            p(index.pprog),...
                                            p(index.pab),...
                                            p(index.pdotbeta),...
                                            p(index.pspeedcost),...
                                            p(index.pslack),...
                                            p(index.pslack2),...
                                            p(index.alpha1),...
                                            p(index.alpha2));
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
        case 3
            switch Game
                case 'PG'
                    switch LEPunisher
                        case 'no'
                            for i=1:model.N
                                model.objective{i} = @(z,p)objective_PG3(z,...
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
                                model.objective{i} = @(z,p)objective_PG3_LE(z,...
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
        case 5
            switch Game
                case 'PG'
                    switch LEPunisher
                        case 'no'
                            for i=1:model.N
                                model.objective{i} = @(z,p)objective_PG5(z,...
                                getPointsFromParameters(p, pointsO, pointsN),...
                                getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                                getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2 + 3*pointsN3, pointsN4),...
                                getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2 + 3*pointsN3 + 3*pointsN4, pointsN5),...
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
                                model.objective{i} = @(z,p)objective_PG5_LE(z,...
                                getPointsFromParameters(p, pointsO, pointsN),...
                                getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                                getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2 + 3*pointsN3, pointsN4),...
                                getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2 + 3*pointsN3 + 3*pointsN4, pointsN5),...
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

        % Delta path progress
        model.ub(index_IBR.ds)=ds_max;
        model.lb(index_IBR.ds)=ds_min;

        % Acceleration
        model.lb(index_IBR.ab)=-inf;

        % Slack
        model.lb(index_IBR.slack)=0;

        % Velocity
        model.lb(index_IBR.v)=0;

        % Steering Angle
        model.ub(index_IBR.beta)=beta_max;
        model.lb(index_IBR.beta)=beta_min;

        %Path Progress
        model.ub(index_IBR.s)=pointsN-2;
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
        model.lb(index.slack)=0;

        % Velocity
        model.lb(index.v)=0;

        % Steering Angle
        model.ub(index.beta)=beta_max;
        model.lb(index.beta)=beta_min;

        %Path Progress
        model.ub(index.s)=pointsN-2;
        model.lb(index.s)=0;
    end
    switch NUM_Vehicles
        case 1
            %limit lateral acceleration
            model.nh = NUM_const; 
            model.ineq = @(z,p) nlconst(z,p);
            model.hu = [0;0;0];
            model.hl = [-inf;-inf;-inf];
        case 2
            switch Game
                case 'PG'
                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k2)=ds_max;
                    model.lb(index.ds_k2)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k2)=-inf;

                    % Slack Variables Constraint (input)
                    model.lb(index.slack_k2)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k2)=0;

                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k2)=beta_max;
                    model.lb(index.beta_k2)=beta_min;

                    % Path Progress Constraint (input)
                    model.ub(index.s_k2)=pointsN2-2;
                    model.lb(index.s_k2)=0;

                    % collision
                    model.lb(index.slack2)=0;
                    
                    model.nh = NUM_const; 
                    model.ineq = @(z,p) nlconst_PG(z,p);
                    model.hu = [0;0;0;...
                                0;0;0;...
                                0];%
                    model.hl = [-inf;-inf;-inf;...
                                -inf;-inf;-inf;...
                                -inf];%
                case 'IBR'
                    model.nh = 4; 
                    model.ineq = @(z,p) nlconst_IBR(z,p);
                    model.hu = [0;0;0;0];
                    model.hl = [-inf;-inf;-inf;-inf];
             end
        case 3
            switch Game
                case 'PG'
                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k2)=ds_max;
                    model.lb(index.ds_k2)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k2)=-inf;

                    % Slack Variables Constraint (input)
                    model.lb(index.slack_k2)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k2)=0;

                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k2)=beta_max;
                    model.lb(index.beta_k2)=beta_min;

                    % Path Progress Constraint (input)
                    model.ub(index.s_k2)=pointsN2-2;
                    model.lb(index.s_k2)=0;

                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k3)=ds_max;
                    model.lb(index.ds_k3)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k3)=-inf;

                    % Slack Variables Constraint (input)
                    model.lb(index.slack_k3)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k3)=0;

                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k3)=beta_max;
                    model.lb(index.beta_k3)=beta_min;

                    % Path Progress Constraint (input)
                    model.ub(index.s_k3)=pointsN3-2;
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
                case 'IBR'
                    model.nh = 5; 
                    model.ineq = @(z,p) nlconst_IBR_3(z,p);
                    model.hu = [0;0;0;0;0];
                    model.hl = [-inf;-inf;-inf;-inf;-inf];
            end
        case 5
            switch Game
                case 'PG'
                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k2)=ds_max;
                    model.lb(index.ds_k2)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k2)=-inf;

                    % Slack Variables Constraint (input)
                    model.lb(index.slack_k2)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k2)=0;

                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k2)=beta_max;
                    model.lb(index.beta_k2)=beta_min;

                    % Path Progress Constraint (input)
                    model.ub(index.s_k2)=pointsN2-2;
                    model.lb(index.s_k2)=0;

                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k3)=ds_max;
                    model.lb(index.ds_k3)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k3)=-inf;

                    % Slack Variables Constraint (input)
                    model.lb(index.slack_k3)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k3)=0;

                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k3)=beta_max;
                    model.lb(index.beta_k3)=beta_min;

                    % Path Progress Constraint (input)
                    model.ub(index.s_k3)=pointsN3-2;
                    model.lb(index.s_k3)=0;
                    %4)
                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k4)=ds_max;
                    model.lb(index.ds_k4)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k4)=-inf;

                    % Slack Variables Constraint (input)
                    model.lb(index.slack_k4)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k4)=0;

                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k4)=beta_max;
                    model.lb(index.beta_k4)=beta_min;

                    % Path Progress Constraint (input)
                    model.ub(index.s_k4)=pointsN4-2;
                    model.lb(index.s_k4)=0;
                    %5)
                    % Path Progress rate Constraint (input)
                    model.ub(index.ds_k5)=ds_max;
                    model.lb(index.ds_k5)=ds_min;

                    % Acceleration Constraint (input)
                    model.lb(index.ab_k5)=-inf;

                    % Slack Variables Constraint (input)
                    model.lb(index.slack_k5)=0;

                    % Speed Constraint (state)
                    model.lb(index.v_k5)=0;

                    % Steering Angle Constraint (input)
                    model.ub(index.beta_k5)=beta_max;
                    model.lb(index.beta_k5)=beta_min;

                    % Path Progress Constraint (input)
                    model.ub(index.s_k5)=pointsN5-2;
                    model.lb(index.s_k5)=0;
                    
                    model.lb(index.slack2)=0;
                    model.lb(index.slack3)=0;
                    model.lb(index.slack4)=0;

                    %limit lateral acceleration
                    model.nh = NUM_const; 
                    model.ineq = @(z,p) nlconst_PG5(z,p);
                    model.hu = [0;0;0;0;0;0;...
                                0;0;0;0;0;0;...
                                0;0;0;0;0;0;...
                                0;0;0;0;0;0;...
                                0];%];%
                    model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;-inf;-inf;...
                                -inf];%
                case 'IBR'
                    model.nh = 7; 
                    model.ineq = @(z,p) nlconst_IBR_5(z,p);
                    model.hu = [0;0;0;0;0;0;0];
                    model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;-inf];
            end
    end

    %% Solver
    switch NUM_Vehicles
        case 1
            codeoptions = getOptions('MPCPathFollowing_1v');
        case 2
            switch Game
                case 'PG'
                    switch Alpha
                        case 'no'
                        	codeoptions = getOptions('MPCPathFollowing_2v');
                        case 'yes'
                            codeoptions = getOptions('MPCPathFollowing_2v_alpha');
                    end
                case 'IBR'
                    codeoptions = getOptions('MPCPathFollowing_2v_IBR');
           end
        case 3
            switch Game
                case 'PG'
                    codeoptions = getOptions('MPCPathFollowing_3v');
                case 'IBR'
                    codeoptions = getOptions('MPCPathFollowing_3v_IBR');
            end
        case 5
            switch Game
                case 'PG'
                    codeoptions = getOptions('MPCPathFollowing_5v');
                case 'IBR'
                    codeoptions = getOptions('MPCPathFollowing_5v_IBR');
            end
    end

    codeoptions.maxit = MAX_IT;    % Maximum number of iterations
    codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
    codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
    codeoptions.cleanup = false;
    codeoptions.timing = 1;
    output = newOutput('alldata', 1:model.N, 1:model.nvar);
    FORCES_NLP(model, codeoptions,output);
end

%% Run simulation 
if strcmp(Simulation, 'yes')
    switch NUM_Vehicles
        case 1
            Run_1_vehicle
        case 2
            switch Condition
                case 'cen'
                    switch Alpha
                        case 'no'
                            Run_2_vehicles_cen
                        case 'yes'
                            Run_2_vehicles_cen_alpha
                    end
                case 'dec'
                    switch Alpha
                        case 'no'
                            switch Game
                                case 'PG'
                                    Run_2_vehicles_dec
                                case 'IBR'
                                    Run_2_vehicles_dec_IBR
                            end
                        case 'yes'
                            Run_2_vehicles_dec_alpha
                    end
                otherwise
                    error('Change Condition')
            end
        case 3
            switch Game
                case 'PG'
                    Run_3_vehicles_cen
                case 'IBR'
                    Run_3_vehicles_IBR
            end
        case 5
            switch Game
                case 'PG'
                    Run_5_vehicles_cen
               case 'IBR'
                    Run_5_vehicles_IBR
           end
    end
elseif strcmp(TestAlpha1shot, 'yes')
    RunAlpha_1shot_convex % RunAlpha_1shot
end