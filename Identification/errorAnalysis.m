clear; clc;
load dataset.mat;

Ntraj = 15000;
NsimMax = 50;
Norder = 8;

% Create random initial states
xLowLim = [10; -30; -10];
xUppLim = [50; 30; 10];
x0 = xLowLim + rand(nx,Ntraj).*(xUppLim - xLowLim);

% Create random input signals
uLow = -10000;
uUpp = 10000;
uTest = uLow + rand(nu,NsimMax+1,Ntraj).*(uUpp - uLow);

tspan = linspace(0,NsimMax*Ts,NsimMax+1);
errVector = zeros(Ntraj,Norder);
sysKoop = cell(Norder,1);
cardinality = zeros(Norder,1);
%%
for k=1:Norder

    % approximate Koopman operator
    order = k;
    disp(strcat(['Order:',num2str(k)]));
    b = mpt.basis.poly(nx, order);
    b{end+1} = '1';
    b{end+1} = 'atan2(x(2) + 1.435*x(3), x(1) - 0.846*x(3))'; % front left slip angle component
    b{end+1} = 'atan2(x(2) + 1.435*x(3), x(1) + 0.846*x(3))'; % front right slip angle component
    b{end+1} = 'atan2(x(2) - 1.31*x(3), x(1) - 0.846*x(3))'; % real left slip angle
    b{end+1} = 'atan2(x(2) - 1.31*x(3), x(1) + 0.846*x(3))'; % real right slip angle
    b = unique(b,'stable');
    sysKoop{k} = KoopmanSystem.fitx(x, u, xNext, b, Ts,'linsolve',2);
    cardinality(k) = size(sysKoop{k}.A,1);
end

%% Simulate trajectories
Nsim = zeros(Ntraj,1);
for i=1:Ntraj

    state0 = x0(:,i);
    input = [tspan; uTest(:,:,i)]';

    sim('VehicleSimulationErrorTest'); %simulate nonlinear system
    % remove the last sample from the data (out of the region of interest)
    X = X(1:end-1,:);
    U = U(1:end-1,:);
    Nsim(i) = size(U,1); % current trajectory size
    
    for k=1:Norder
        ksim = sysKoop{k}.simulate(sysKoop{k}.Psi(state0), U'); % simulate Koopman system

        errVector(i,k) = 100*sqrt(sum((ksim.Y' - X).^2,'all'))/sqrt(sum(X.^2,'all'));
    end
    disp(strcat(['Trajectory ', num2str(i),'/',num2str(Ntraj)]));

end

meanErr = mean(errVector);
save('allErrorData');
