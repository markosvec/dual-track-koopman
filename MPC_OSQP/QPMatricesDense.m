function [Px,q, Ax, l, u] = QPMatricesDense(A, B, C, Q, p, Np, T, ref, steer, x0, u0, d, G, uLim, dULim, t)
%QP MATRICES DENSE
% The fuNption calculates matrices Px and Ax and vectors q, l and u. 

% Define persistant variables
persistent QExt M pHat inputRateMatrix  inputRateLowLimVector inputRateUppLimVector ...
    prevInputMatrix inputMatrix inputLowLimVector inputUppLimVector

% Extract system dimenstions
[nStates, nInputs] = size(B);
nOutputs = size(C,1);
nSlack = size(G,1);

% Extract input and input rate constraints
lowerInputLim = uLim(:,1);
upperInputLim = uLim(:,2);
lowerInputRateLim = dULim(:,1);
upperInputRateLim = dULim(:,2);

if (t == 0)
    QExt = DiagonalizeMatrix(Q,Np); % Diagonalize weight matrices

    % INPUT RATE WEIGHT MATRIX
    % M is a matrix which, when multiplied with vector [u_hat(k-1); U], calculates 
    % vector containing delta U signals. It subtracts u(k) from u(k+1).
    % dU= [du(0) du(1) du(2) ...] = M*[u_hat(k-1); U].
    % Input vector has a dimension Np+1, and a result has a dimension Np.
    MZero = zeros(Np*nInputs,nInputs);
    MEye = eye(Np*nInputs);
    M = [MZero, MEye] + [-MEye, MZero];

    % SLACK WEIGHT MATRIX
    pHat = blkdiag(zeros(nInputs*(Np+1)), p*eye(nSlack*Np));

    % INPUT RATE CONSTRAINTS
    inputRateMatrix = [M/T, zeros(nInputs*Np, nSlack*Np)];
    inputRateLowLimVector= lowerInputRateLim.*ones(nInputs,Np);
    inputRateUppLimVector = upperInputRateLim.*ones(nInputs,Np);

    % PREVIOUS INPUT CONSTRAINT
    prevInputMatrix = [eye(nInputs), zeros(nInputs,nInputs*Np+nSlack*Np)];

    % INPUT CONSTRAINTS
    inputMatrix = [zeros(nInputs*Np, nInputs), eye(nInputs*Np), zeros(nInputs*Np, nSlack*Np)];
    inputLowLimVector = lowerInputLim.*ones(nInputs,Np);
    inputUppLimVector = upperInputLim.*ones(nInputs,Np);
end
 
%% OBJECTIVE FUNpTION
% Calculate system dynamics matrices
[F, PHI, THETA] = PredictionMatrices(A, B, C, Np);
% Extend matrix PHI for augumented state (previous input measurement u_hat(k-1))
PHIHat =  [zeros(nOutputs*Np, nInputs), PHI];

% OUTPUT WEIGHT MATRIX
% Create diagonal matrix QHat and vector qHat suitable for optimizer [u_hat(k-1); U; eps]
QHat = blkdiag(PHIHat'*QExt*PHIHat, zeros(nSlack*Np)); % Quadratic
qHat = [2*(F*x0 + THETA*d - ref)'*QExt*PHIHat, zeros(1,nSlack*Np)]'; % Linear

% Create Px and q matrices
Px = QHat + pHat;
q = qHat;

%% Update steering angle constraint
inputLowLimVector(1,:) = steer(1:end-1)';
inputUppLimVector(1,:) = steer(1:end-1)';

% SLIP RATIO AND SLIP ANGLE CONSTRAINTS
[Fx,PHIx, THETAx] = PredictionMatrices(A, B, eye(nStates), Np);
PHIxHat =  [zeros(nStates*Np, nInputs), PHIx];
GAux = DiagonalizeMatrix(G,Np);

slipMatrix = [GAux*PHIxHat, -eye(nSlack*Np)];
slipLowLimVector = -1e10*ones(nSlack*Np,1);
slipUppLimVector = -GAux*(Fx*x0 + THETAx*d(:));

% Slack >= 0
slackMatrix = [zeros(nSlack*Np, nInputs*(Np+1)), eye(nSlack*Np)];
slackLowLim = zeros(nSlack*Np,1);
slackUppLim = 1e10*ones(nSlack*Np,1);

% Create matrix Ax and limit vectors l and u
Ax = [prevInputMatrix; inputMatrix; inputRateMatrix; slipMatrix; slackMatrix];
l =  [u0; inputLowLimVector(:); inputRateLowLimVector(:); slipLowLimVector; slackLowLim];
u =  [u0; inputUppLimVector(:); inputRateUppLimVector(:); slipUppLimVector; slackUppLim];

end

