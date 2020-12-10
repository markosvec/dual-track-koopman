function [Px,q, Ax, l, u] = QPMatricesDenseKoop(A, B, C, rotMatrix, dRotMatrix, Q, p, Np, T, ref, steer, x0, u0, uLim, dULim, slipLim, Cy, t)
%#codegen
%QP MATRICES DENSE
% The fuNption calculates matrices Px and Ax and vectors q, l and u. 

% Define persistant variables
persistent PxTemp F QExt PHIHat M inputRateLowLimVector inputRateUppLimVector ...
    prevInputMatrix inputLowLimVector inputUppLimVector latForceMatHat ...
    Fslip0 Fslip PHIslip0Hat PHIslipHat

% Extract system dimenstions
[nStates, nInputs] = size(B);
nOutputs = size(C,1);

nSlack = 8;

% Extract input and input rate constraints
lowerInputLim = uLim(:,1);
upperInputLim = uLim(:,2);
lowerInputRateLim = dULim(:,1);
upperInputRateLim = dULim(:,2);

% Extract slip limits
slipAngMin = slipLim(1)*ones(nSlack/2*Np,1);
slipAngMax = slipLim(2)*ones(nSlack/2*Np,1);

if (t==0)
    %% OBJECTIVE FUNCTION
    % Calculate system dynamics matrices
    [F, PHI, ~] = PredictionMatrices(A, B, C, Np);
    % Extend matrix PHI for augumented state (previous input measurement u_hat(k-1))
    PHIHat =  [zeros(nOutputs*Np, nInputs), PHI];

    % Diagonalize weight matrices
    QExt = DiagonalizeMatrix(Q,Np);

    % INPUT RATE WEIGHT MATRIX
    % M is a matrix which, when multiplied with vector [u_hat(k-1); U], calculates 
    % vector containing delta U signals. It subtracts u(k) from u(k+1).
    % dU= [du(0) du(1) du(2) ...] = M*[u_hat(k-1); U].
    % Input vector has a dimension Np+1, and a result has a dimension Np.
    MZero = zeros(Np*nInputs,nInputs);
    MEye = eye(Np*nInputs);
    M = [MZero, MEye] + [-MEye, MZero];

    % OUTPUT WEIGHT MATRIX
    % Create diagonal matrix QHat and vector qHat suitable for optimizer [u_hat(k-1); U; eps]
    QHat = blkdiag(PHIHat'*QExt*PHIHat, zeros(nSlack*Np)); % Quadratic

    % SLACK WEIGHT MATRIX
    pHat = blkdiag(zeros(nInputs*(Np+1)), p*eye(nSlack*Np));

    PxTemp = QHat + pHat;
    
    %% CONSTRAINTS
    % PREVIOUS INPUT CONSTRAINT
    prevInputMatrix = [eye(nInputs), zeros(nInputs,nInputs*Np+nSlack*Np)];

    inputLowLimVector = lowerInputLim.*ones(nInputs,Np);
    inputUppLimVector = upperInputLim.*ones(nInputs,Np);

    inputRateLowLimVector= lowerInputRateLim.*ones(nInputs,Np);
    inputRateUppLimVector = upperInputRateLim.*ones(nInputs,Np);

    %% SLIP ANGLE PREDICTION
    % Calculate system dynamics matrices
    Cslip = zeros(4,nStates);
    Cslip(1,end-3) = 1;
    Cslip(2,end-2) = 1;
    Cslip(3,end-1) = 1;
    Cslip(4,end) = 1;
    [Fslip, PHIslip, ~] = PredictionMatrices(A, B, Cslip, Np);
    PHIslipHat =  [zeros(4*Np, nInputs), PHIslip];
    
    % Slip for steering
    H = [1 0 0 0; 
         0 1 0 0];
    Hhat = DiagonalizeMatrix(H, Np);

    % Incorporate matrices to calculate [z0 z1 z2 ...]'
    Fslip0 = Hhat*[Cslip; Fslip(1:end-4,:)];
    PHIslip0 = Hhat*[zeros(4,nInputs*Np); PHIslip(1:end-4,:)];
    PHIslip0Hat =  [zeros(2*Np, nInputs), PHIslip0]; % Extend matrix PHI for augumented state (previous input measurement u_hat(k-1))

    latForceMat = [0 1 0 0 0 0;
                   0 0 0 1 0 0];
    latForceMatHat = DiagonalizeMatrix(latForceMat, Np);
end
 
% Create Px and q matrices
Px = PxTemp;
qHat = [2*(F*x0 - ref)'*QExt*PHIHat, zeros(1,nSlack*Np)]'; % Linear
q = qHat;

%% CONSTRAINTS

% INPUT CONSTRAINTS
rotMatrixHat = DiagonalizeMatrix(rotMatrix, Np);
inputMatrix = [zeros(nInputs*Np, nInputs), rotMatrixHat, zeros(nInputs*Np, nSlack*Np)];

% INPUT RATE CONSTRAINTS
dRotMatrixHat = DiagonalizeMatrix(dRotMatrix, Np);
dRotMatrixExt = [zeros(nInputs*Np, nInputs), dRotMatrixHat];
inputRateMatrix = [rotMatrixHat*M/T + dRotMatrixExt, zeros(nInputs*Np, nSlack*Np)];

%% SLIP ANGLE PREDICTION
latForceMatExt = [zeros(2*Np, nInputs), latForceMatHat*rotMatrixHat];
steeringMatrix = [latForceMatExt + Cy*PHIslip0Hat, zeros(2*Np, nSlack*Np)];

steerAngVector = DiagonalizeMatrix([1 1]', Np)*steer(1:end-1);
steeringVector = Cy*(steerAngVector - Fslip0*x0);

% Slip angle constraint
slipMatrixOne = [PHIslipHat, -eye(nSlack/2*Np), zeros(nSlack/2*Np)];
slipMatrixTwo = [-PHIslipHat, zeros(nSlack/2*Np), -eye(nSlack/2*Np)];

steerVectorAux = [steer(2:end)'; steer(2:end)'; zeros(2,Np)];
slipVectorLowOne = -1e10*ones(nSlack/2*Np,1);
slipVectorUppOne = slipAngMax - Fslip*x0 + steerVectorAux(:);
slipVectorLowTwo = -1e10*ones(nSlack/2*Np,1);
slipVectorUppTwo = -(slipAngMin - Fslip*x0 + steerVectorAux(:));

% Slack variable positivity constraint
slackMatrix = [zeros(nSlack*Np, (Np+1)*nInputs), eye(nSlack*Np)];
slackVectorLow = zeros(nSlack*Np,1);
slackVectorUpp = 1e10*ones(nSlack*Np,1);

% Create matrix Ax and limit vectors l and u
Ax = [prevInputMatrix; inputMatrix; inputRateMatrix; steeringMatrix; slipMatrixOne; slipMatrixTwo; slackMatrix];
l =  [u0; inputLowLimVector(:); inputRateLowLimVector(:); steeringVector; slipVectorLowOne; slipVectorLowTwo; slackVectorLow];
u =  [u0; inputUppLimVector(:); inputRateUppLimVector(:); steeringVector; slipVectorUppOne; slipVectorUppTwo; slackVectorUpp];
end

