function output = MPCControllerKoop(currentX, totalRef, totalSteer, t, T, nSignal, nRef)
% MPC CONTROLLER
%   The function solves the MPC problem and returns control input,
%   diagnostics information and slack variable values. Optimization problem
%   is solved using YALMIP toolbox (https://yalmip.github.io/).

persistent uout sysKoop slipAngMax slipAngMin controller vehicle N

start = tic; % start the timer

% Initialize persistent variable values
if (t == 0)
    load('koopData.mat');
    vehicle = LoadVehicleParameters();
    
    Akoop = sysKoop.A;
    Bkoop = sysKoop.B;
    Ckoop = sysKoop.C;
    uout = zeros(size(Bkoop,2),1);
    
    nz = size(Akoop,1); % Number of states
    nx = size(Ckoop,1);
    nu = size(Bkoop,2); % Number of inputs
    nSlipAng = 4;
    N = 20; % Prediction horizon
   
    % Define data for the MPC controller
    % Input and input rate constraints
    slipAngMax = 5/180*pi;
    slipAngMin = -5/180*pi;
    
    lowerInputLim = [-0.05*vehicle.Cx; -vehicle.Cy; -0.05*vehicle.Cx; -vehicle.Cy; -0.05*vehicle.Cx; -0.05*vehicle.Cx];
    upperInputLim = -lowerInputLim;
    lowerInputRateLim = [-0.0075*vehicle.Cx; -vehicle.Cy; -0.0075*vehicle.Cx; -vehicle.Cy; -0.0075*vehicle.Cx; -0.0075*vehicle.Cx]/T;
    upperInputRateLim = -lowerInputRateLim;
    
    costNorm = 1;
    Q = [1 0 0; 
         0 0 0; 
         0 0 100]*costNorm;
    p = 1e8*costNorm;

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear');

    % Setup the optimization problem
    u = sdpvar(repmat(nu,1,N-1), ones(1,N-1));
    z = sdpvar(repmat(nz,1,N), ones(1,N));
    u0 = sdpvar(nu,1);
    r = sdpvar(repmat(nx,1,N), ones(1,N));
    epsLow = sdpvar(repmat(nSlipAng,1,N-1), ones(1,N-1));
    epsUpp = sdpvar(repmat(nSlipAng,1,N-1), ones(1,N-1));
    rotMatrix = sdpvar(nu, nu*(N-1));
    dRotMatrix = sdpvar(nu, nu*(N-1));
    steerVect = sdpvar(N,1);
    
    constraints = [];
    objective = 0;
    for k = 1:N-1

        if (k == 1)
            pastU = u0;
        else
            pastU = u{k-1};
        end
            
        rotU = rotMatrix(:,(k-1)*nu+1:k*nu)*u{k}; % rotated input forces
        constraints = [constraints, rotU(2) ==  -vehicle.Cy*(z{k}(end-3) - steerVect(k))]; % front left wheel lateral force
        constraints = [constraints, rotU(4) ==  -vehicle.Cy*(z{k}(end-2) - steerVect(k))]; % front right wheel lateral force

        constraints = [constraints, (lowerInputLim <= rotU) & (rotU <= upperInputLim)];
        constraints = [constraints, lowerInputRateLim <= rotMatrix(:,(k-1)*nu+1:k*nu)*(u{k}-pastU)/T + dRotMatrix(:,(k-1)*nu+1:k*nu)*u{k} <= upperInputRateLim];

        objective = objective + (Ckoop*z{k}-r{k})'*Q*(Ckoop*z{k}-r{k}) + p*epsUpp{k}'*epsUpp{k} + p*epsLow{k}'*epsLow{k};
        constraints = [constraints, z{k+1} == Akoop*z{k} + Bkoop*u{k}];
        
        % Constraints for slip angles
        slipAngVector = z{k+1}(end-3:end) - [steerVect(k+1); steerVect(k+1); 0; 0];
        constraints = [constraints, slipAngVector <= slipAngMax + epsUpp{k}, epsUpp{k} >= 0]; % upper slip angle soft constraint
        constraints = [constraints, slipAngVector >= slipAngMin - epsLow{k}, epsLow{k} >= 0]; % lower slip angle soft constraint
    end
    objective = objective + (Ckoop*z{N}-r{N})'*Q*(Ckoop*z{N}-r{N}); % Terminal cost

    % Setup the optimization solver
    ops = sdpsettings('solver','gurobi','verbose',0);
    
    parameters_in = {z{1}, [r{:}], steerVect, rotMatrix, dRotMatrix, u0};
    solutions_out = {[u{:}], [z{:}], [epsLow{:}], [epsUpp{:}], objective};
    controller = optimizer(constraints, objective, ops, parameters_in, solutions_out);
end

% Extract reference and steering data
[ref, steer] = LTVReference(totalRef, totalSteer, nSignal, nRef, N, T, t);
[rotMatrixNum, dRotMatrixNum] = ForceRotationMatrices(steer, N-1, T);

currentZ = sysKoop.Psi(currentX);
inputs = {currentZ, ref', steer, rotMatrixNum, dRotMatrixNum, uout};
[solutions, diagnostics] = controller{inputs};  

% Input extraction
uout = solutions{1}(:,1);
% Output
rotatedForces = rotMatrixNum(:,1:6)*uout;
slipVector = rotatedForces([1 3 5 6])/vehicle.Cx;
objective = solutions{5};

duration = toc(start);

output = [steer(1); slipVector; objective; diagnostics; duration];
disp(['Feasible: ',num2str(diagnostics),' Time: ', num2str(t), ' Duration: ', num2str(duration)]);
end