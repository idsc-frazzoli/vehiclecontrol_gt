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
addpath('Run_Simulation')
clear model
clear problem
clear all
close all

% configuration
NUM_Vehicles = 2; %1,2,3
Compiled    = 'no'; % 'yes' or 'no', yes if code has already been compiled
Simulation  = 'yes';% 'yes' or 'no', no if you don't want to run simulation
TestAlpha1shot='no';% 'yes' or 'no', yes if you want to test alpha. 
                    % Simulation must be no
LEPunisher  = 'no'; % 'yes' or 'no' % Lateral Error Punisher (It Penalizes
                                    % only the left side of the centerline)
% NUM Vehicles=2
Condition   = 'cen'; % 'cen' 'dec';
Game        = 'PG'; % IBR, PG; 'IBR' for 'dec' only, without alpha
Alpha       = 'no'; % yes , no; yes for 'cen' condition and 'PG' Game only

%% Parameters Definitions (parameters_vector folder)
switch NUM_Vehicles
    case 1
        parameters_1_vehicle

    case 2
        parameters_2_vehicles
        % for alpha case
        if strcmp(Alpha,'yes')
            pointsO=18;
        end
        if strcmp(Game,'IBR')
            pointsO=18;
            NUM_const=7;
        end
    case 3
        parameters_3_vehicles
    otherwise
        error('Change NUM_Vehicle')
end

%% State and Input Definitions 
global index index_IBR
switch NUM_Vehicles
    case 1
        indexes_1_vehicle
    case 2
        switch Alpha
            case 'no'
                switch Game
                    case 'PG'
                        indexes_2_vehicles
                    case 'IBR'
                        indexes_2_vehicles_IBR
                    otherwise
                        error('Change Game')
                end
            case 'yes'
                indexes_2_vehicles_alpha
            otherwise
                error('Change Alpha')
        end
    case 3
        indexes_3_vehicles
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
                           @(x,u,p)interstagedx_IBR_1s(x,u), integrator_stepsize,p);
                otherwise
                    error('Change Game')
            end

        case 3
            model.eq = @(z,p) RK4(z(index.sb:end), z(1:index.nu),...
                       @(x,u,p)interstagedx_PG3(x,u),integrator_stepsize,p);
    end
    
    if strcmp(Game,'IBR')
        model.E = [zeros(index_IBR.ns,index_IBR.nu), eye(index_IBR.ns)];
    else
        model.E = [zeros(index.ns,index.nu), eye(index.ns)];
    end
    %% Objective Function (objective_function folder)
    switch NUM_Vehicles
        case 1
            for i=1:model.N
                model.objective{i} = @(z,p)objective(z,...
                    getPointsFromParameters(p, pointsO, pointsN),...
                    getRadiiFromParameters(p, pointsO, pointsN),...
                    p(index.ps),...
                    p(index.pax),...
                    p(index.pay),...
                    p(index.pll),...
                    p(index.prae),...
                    p(index.ptve),...
                    p(index.pbre),...
                    p(index.plag),...
                    p(index.plat),...
                    p(index.pprog),...
                    p(index.pab),...
                    p(index.pdotbeta),...
                    p(index.pspeedcost),...
                    p(index.pslack));
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
                                        getRadiiFromParameters(p, pointsO, pointsN),...
                                        getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                        getRadiiFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                        p(index.ps),...
                                        p(index.pax),...
                                        p(index.pay),...
                                        p(index.pll),...
                                        p(index.prae),...
                                        p(index.ptve),...
                                        p(index.pbre),...
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
                                        getRadiiFromParameters(p, pointsO, pointsN),...
                                        getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                        getRadiiFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                                        p(index.ps),...
                                        p(index.pax),...
                                        p(index.pay),...
                                        p(index.pll),...
                                        p(index.prae),...
                                        p(index.ptve),...
                                        p(index.pbre),...
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
                            for i=1:model.N
                                model.objective{i} = @(z,p)objective_IBR(z,...
                                getPointsFromParameters(p, pointsO, pointsN),...
                                getRadiiFromParameters(p, pointsO, pointsN),...
                                p(index_IBR.ps),...
                                p(index_IBR.pax),...
                                p(index_IBR.pay),...
                                p(index_IBR.pll),...
                                p(index_IBR.prae),...
                                p(index_IBR.ptve),...
                                p(index_IBR.pbre),...
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
                case 'yes'
                    for i=1:model.N
                        model.objective{i} = @(z,p)objective_PG_alpha(z,...
                            getPointsFromParameters(p, pointsO, pointsN),...
                            getRadiiFromParameters(p, pointsO, pointsN),...
                            getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                            getRadiiFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                            p(index.ps),...
                            p(index.pax),...
                            p(index.pay),...
                            p(index.pll),...
                            p(index.prae),...
                            p(index.ptve),...
                            p(index.pbre),...
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
        case 3
            switch LEPunisher
                case 'no'
                    for i=1:model.N
                        model.objective{i} = @(z,p)objective_PG3(z,...
                        getPointsFromParameters(p, pointsO, pointsN),...
                        getRadiiFromParameters(p, pointsO, pointsN),...
                        getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                        getRadiiFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                        getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                        getRadiiFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                        p(index.ps),...
                        p(index.pax),...
                        p(index.pay),...
                        p(index.pll),...
                        p(index.prae),...
                        p(index.ptve),...
                        p(index.pbre),...
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
                        getRadiiFromParameters(p, pointsO, pointsN),...
                        getPointsFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                        getRadiiFromParameters(p, pointsO + 3*pointsN, pointsN2),...
                        getPointsFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                        getRadiiFromParameters(p, pointsO + 3*pointsN + 3*pointsN2, pointsN3),...
                        p(index.ps),...
                        p(index.pax),...
                        p(index.pay),...
                        p(index.pll),...
                        p(index.prae),...
                        p(index.ptve),...
                        p(index.pbre),...
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
                    
    end

    %% Linear and NON-Linear Constraints 
    if strcmp(Game,'IBR')
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
            model.hu = [0;1;0;0;0;0];
            model.hl = [-inf;-inf;-inf;-inf;-inf;-inf];
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

                    %non linear
                    model.nh = NUM_const; 
                    model.ineq = @(z,p) nlconst_PG(z,p);
                    model.hu = [0;1;0;0;0;0;...
                                0;1;0;0;0;0;...
                                0];%
                    model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;...
                                -inf;-inf;-inf;-inf;-inf;-inf;...
                                -inf];%
                case 'IBR'
                    
                    %limit lateral acceleration
                    model.nh = NUM_const; 
                    model.ineq = @(z,p) nlconst_IBR(z,p);
                    model.hu = [0;1;0;0;0;0;...
                                0];
                    model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;...
                                -inf];
            end
        case 3
            % Path Progress rate Constraint (input)
            model.ub(index.ds_k2)=5;
            model.lb(index.ds_k2)=-1;

            % Acceleration Constraint (input)
            model.lb(index.ab_k2)=-inf;

            % Slack Variables Constraint (input)
            model.lb(index.slack_k2)=0;

            % Speed Constraint (state)
            model.lb(index.v_k2)=0;

            % Steering Angle Constraint (input)
            model.ub(index.beta_k2)=0.5;
            model.lb(index.beta_k2)=-0.5;

            % Path Progress Constraint (input)
            model.ub(index.s_k2)=pointsN2-2;
            model.lb(index.s_k2)=0;

            % Path Progress rate Constraint (input)
            model.ub(index.ds_k3)=5;
            model.lb(index.ds_k3)=-1;

            % Acceleration Constraint (input)
            model.lb(index.ab_k3)=-inf;

            % Slack Variables Constraint (input)
            model.lb(index.slack_k3)=0;

            % Speed Constraint (state)
            model.lb(index.v_k3)=0;

            % Steering Angle Constraint (input)
            model.ub(index.beta_k3)=0.5;
            model.lb(index.beta_k3)=-0.5;

            % Path Progress Constraint (input)
            model.ub(index.s_k3)=pointsN3-2;
            model.lb(index.s_k3)=0;

            model.lb(index.slack2)=0;
            model.lb(index.slack3)=0;
            model.lb(index.slack4)=0;

            %limit lateral acceleration
            model.nh = NUM_const; 
            model.ineq = @(z,p) nlconst_PG3(z,p);
            model.hu = [0;1;0;0;0;0;...
                        0;1;0;0;0;0;...
                        0;1;0;0;0;0;...
                        0;0;0];%
            model.hl = [-inf;-inf;-inf;-inf;-inf;-inf;...
                        -inf;-inf;-inf;-inf;-inf;-inf;...
                        -inf;-inf;-inf;-inf;-inf;-inf;...
                        -inf;-inf;-inf];%
    end

    %% Solver
    switch NUM_Vehicles
        case 1
            codeoptions = getOptions('MPCPathFollowing_1v');
        case 2
            switch Alpha
                case 'no'
                    switch Game
                        case 'PG'
                            codeoptions = getOptions('MPCPathFollowing_2v');
                        case 'IBR'
                            codeoptions = getOptions('MPCPathFollowing_IBR');
                    end
                case 'yes'
                    codeoptions = getOptions('MPCPathFollowing_2v_alpha');
            end
        case 3
            codeoptions = getOptions('MPCPathFollowing_3v');
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
            Run_3_vehicles_cen
    end
elseif strcmp(TestAlpha1shot, 'yes')
    RunAlpha_1shot
end