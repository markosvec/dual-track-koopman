function output = MPCController(currentX, totalRef, totalSteer, t, T, nSignal, nRef)
% MPC CONTROLLER
%   The function solves the MPC problem and returns control input,
%   diagnostics information and slack variable values. Optimization problem
%   is solved using YALMIP toolbox (https://yalmip.github.io/).

persistent oldU oldX uout vehicle controller N slipAngMax slipAngMin

start = tic;

if (t == 0)
    
    nx = 3; % Number of states
    nu = 5; % Number of inputs
    no = 2;
    nSlipAng = 8;
    N = 20; % Prediction horizon

    vehicle = LoadVehicleParameters();

    % Define data for the MPC controller
    % Input and input rate constraints
    slipAngMax = 5/180*pi;
    slipAngMin = -5/180*pi;

    % Define data for the MPC controller
    % Input and input rate constraints
    lowerInputLim = [-45/180*pi; -0.05; -0.05; -0.05; -0.05];
    upperInputLim = -lowerInputLim;
    lowerInputRateLim = [-180/180*pi; -0.0075; -0.0075; -0.0075; -0.0075]/T;
    upperInputRateLim = -lowerInputRateLim;

    Q = [1 0; 
         0 100];
    p = 1e8;
    C = [1 0 0;
         0 0 1];
 
    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear');

    % Setup the optimization problem
    u = sdpvar(repmat(nu,1,N-1), ones(1,N-1));
    x = sdpvar(repmat(nx,1,N), ones(1,N));
    At = sdpvar(nx, (N-1)*nx);
    Bt = sdpvar(nx, (N-1)*nu);
    d = sdpvar(nx, N-1);
    u0 = sdpvar(nu,1);
    r = sdpvar(repmat(no,1,N), ones(1,N));
    steerVect = sdpvar(N,1);
    G = sdpvar(nSlipAng, (N-1)*nx);
    eps = sdpvar(repmat(nSlipAng,1,N-1), ones(1,N-1));

    % Initialize persistent variable values
    if (t == 0)
        [oldU{1,1:N-1}] = deal(zeros(nu,1));
        [oldX{1,1:N-1}] = deal(currentX);
        uout = zeros(nu,1);
    end

    constraints = [];
    objective = 0;
    for k = 1:N-1

        if (k == 1)
            pastU = u0;
        else
            pastU = u{k-1};
        end

        constraints = [constraints, lowerInputLim <= u{k} <= upperInputLim];
        constraints = [constraints, lowerInputRateLim <= (u{k}-pastU)/T <= upperInputRateLim];
        
        objective = objective + (C*x{k}-r{k})'*Q*(C*x{k}-r{k}) + p*eps{k}'*eps{k};
        constraints = [constraints, x{k+1} == At(:,1:nx)*x{k} + Bt(:,1:nu)*u{k} + d(:,k)];
        constraints = [constraints, G(:,(k-1)*nx+1:k*nx)*x{k+1} <= eps{k}, eps{k} >= 0];

        constraints = [constraints, u{k}(1) == steerVect(k)]; % steering reference
        %constraints = [constraints, u{k}(2) == 0, u{k}(2) == u{k}(3), u{k}(2) == u{k}(4), u{k}(2) == u{k}(5)];
    end
    objective = objective + (C*x{N}-r{N})'*Q*(C*x{N}-r{N}); % Terminal cost
    
     % Setup the optimization solver
    ops = sdpsettings('solver','gurobi','verbose',0);

    parameters_in = {x{1}, [r{:}], d, steerVect, G, At, Bt, u0};
    solutions_out = {[u{:}], [x{:}], [eps{:}], objective};
    controller = optimizer(constraints, objective, ops, parameters_in, solutions_out);
end

[ref, steer] = LTVReference(totalRef, totalSteer, nSignal, nRef, N, T, t); % Extract reference and steering data
[Acell, Bcell] = LinearizedMatricesCell(oldX, oldU, T); % Calculate linearized system dynamics
delta = DeltaSignal(oldX, oldU, Acell, Bcell, N-1, T); % Calculate delta signal
Gnum = SlipAngleConstraints(steer(2:end), vehicle, slipAngMax, slipAngMin);

ref = ref(:,[1,3]); % transform ref vector
inputs = {currentX, ref', [delta{:}], steer, Gnum, [Acell{:}], [Bcell{:}], uout};
[solutions, diagnostics] = controller{inputs};  

% Output
uout = solutions{1}(:,1);
objective = solutions{4};

duration = toc(start);

% Save persistent variables

for i = 1:N-1
    if(i == N-1)
        oldU{i} = solutions{1}(:,i);
    else
        oldU{i} = solutions{1}(:,i+1);
    end
    oldX{i} = solutions{2}(:,i+1); 
end

output = [uout; objective; diagnostics; duration];
disp(['Feasible: ',num2str(diagnostics),' Time: ', num2str(t), ' Duration: ', num2str(duration)]);
end