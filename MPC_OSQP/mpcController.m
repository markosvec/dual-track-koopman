function output = MPCController(currentX, totalRef, totalSteer, t, T, nSignal, nRef)
% MPC CONTROLLER
%   The function solves the MPC problem and returns control input,
%   diagnostics information and slack variable values. Optimization problem
%   is solved using YALMIP toolbox (https://yalmip.github.io/).

persistent uout vehicle prob

start = tic; % start the counter

nx = 3; % Number of states
nu = 5; % Number of inputs
N = 20;

if (t==0)
    vehicle = LoadVehicleParameters();
    uout = zeros(nu,1);
end

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

% Cost matrices
costNorm = 1;
Q = [1 0;
     0 100]*costNorm;
p = 1e8*costNorm;

% Perform precalculation
[ref, steer] = LTVReference(totalRef, totalSteer, nSignal, nRef, N, T, t); % Extract reference and steering data
[Ad, Bd, Cd] = LinearizedMatrices(currentX, uout, T); % Calculate linearized system dynamics
delta = DeltaSignal(currentX, uout, Ad, Bd, N-1, T); % Calculate delta signal
Gnum = SlipAngleConstraints(steer(2:end), vehicle, slipAngMax, slipAngMin);

% create problem formulation
uLim = [lowerInputLim, upperInputLim];
dULim = [lowerInputRateLim, upperInputRateLim];
refVect = reshape(ref(2:end,:)',[2*(N-1),1]);
deltaVect = reshape([delta{:}],[nx*(N-1),1]);
[Px, q, Ax, low, upp] = QPMatricesDense(Ad, Bd, Cd, Q, p, N-1, T, refVect, steer, currentX, uout, deltaVect, Gnum, uLim, dULim, t);

if(t == 0)
    prob = osqp;
    prob.setup(sparse(Px+Px'), q, sparse(Ax), low, upp, 'adaptive_rho_interval', 25, 'verbose', 0);%,'max_iter',1e10);
    prob.update('Ax', nonzeros(Ax));
else
    prob.update('Px', nonzeros(triu(Px+Px')), 'Ax', nonzeros(Ax));
    prob.update('q', q, 'l', low, 'u', upp);
end

res = prob.solve();
xOpt = res.x;

if(res.info.status_val ~= 1)
   disp('problem roðo'); 
end

% Output
uout = xOpt(nu+1:2*nu);
status_val = res.info.status_val;
objective = res.info.obj_val;
duration = toc(start);

output = [uout; objective; status_val; duration];
disp(['Feasible: ',res.info.status,' Time: ', num2str(t), ' Duration: ', num2str(duration)]);
end