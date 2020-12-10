function output = MPCControllerKoop(currentX, totalRef, totalSteer, t, T, nSignal, nRef)
% MPC CONTROLLER
%   The function solves the MPC problem and returns control input,
%   diagnostics information and slack variable values. Optimization problem
%   is solved using YALMIP toolbox (https://yalmip.github.io/).

persistent uout vehicle prob Akoop Bkoop Ckoop sysKoop

start = tic; % start the counter

% Initialize persistent variable values
if (t == 0)
    load('koopData.mat');

    Akoop = sysKoop.A;
    Bkoop = sysKoop.B;
    Ckoop = [1 0 0; 0 0 1]*sysKoop.C;
    
    vehicle = LoadVehicleParameters();
    uout = zeros(size(sysKoop.B,2),1);
end

nz = size(Akoop,1); % Number of states
nx = size(Ckoop,1);
nu = size(Bkoop,2); % Number of inputs
N = 20; % Prediction horizon

% Define data for the MPC controller
% Input and input rate constraints
slipAngMax = 5/180*pi;
slipAngMin = -5/180*pi;

lowerInputLim = [-0.05*vehicle.Cx; -vehicle.Cy; -0.05*vehicle.Cx; -vehicle.Cy; -0.05*vehicle.Cx; -0.05*vehicle.Cx];
upperInputLim = -lowerInputLim;
lowerInputRateLim = [-0.0075*vehicle.Cx; -vehicle.Cy; -0.0075*vehicle.Cx; -vehicle.Cy; -0.0075*vehicle.Cx; -0.0075*vehicle.Cx]/T;
upperInputRateLim = -lowerInputRateLim;

% Cost matrices
costNorm = 1e15;
Q = [1 0;
     0 100]*costNorm;
p = 1e8*costNorm;
  
% Extract reference and steering data
[ref, steer] = LTVReference(totalRef, totalSteer, nSignal, nRef, N, T, t);
[rotMatrixNum, dRotMatrixNum] = ForceRotationMatrices(steer, N-1, T);
currentZ = sysKoop.Psi(currentX);

% create problem formulation
uLim = [lowerInputLim, upperInputLim];
dULim = [lowerInputRateLim, upperInputRateLim];
slipLim = [slipAngMin, slipAngMax];
refVect = reshape(ref(2:end,:)',[nx*(N-1),1]);
[Px, q, Ax, low, upp] = QPMatricesDenseKoop(Akoop, Bkoop, Ckoop, rotMatrixNum, dRotMatrixNum, Q, p, N-1, T, refVect, steer, currentZ, uout, uLim, dULim, slipLim, vehicle.Cy, t);

% Set the problem
if(t == 0)
    prob = osqp;
    prob.setup(sparse(Px + Px'), q, sparse(ones(size(Ax))), low, upp, 'adaptive_rho_interval', 25, 'verbose', 0);
    prob.update('Ax', Ax(:));
else
    prob.update('Px', nonzeros(triu(Px+Px')), 'Ax', Ax(:));
    prob.update('q', q, 'l', low, 'u', upp);
end
% Input extraction
res = prob.solve();
xOpt = res.x;


% Output
uout = xOpt(nu+1:2*nu);
diagnostics = res.info.status_val;
objective = res.info.obj_val;

% Output
rotatedForces = rotMatrixNum(:,:,1)*uout;
slipVector = rotatedForces([1 3 5 6])/vehicle.Cx;
duration = toc(start);

output = [steer(1); slipVector; objective; diagnostics; duration];
disp(['Feasible: ',num2str(diagnostics),' Time: ', num2str(t), ' Duration: ', num2str(duration)]);
end