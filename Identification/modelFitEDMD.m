clear; clc;

if(exist('dataset.mat', 'file') ~= 2) 

    % in case the dataset doesn't exist, create one
    N = 25; 
    Ts = 0.01;
    nx = 3;
    nu = 6;
    nD = nx + nu;
    
    vx = linspace(10,50,N);
    vy = linspace(-30,30,N);
    dTheta = linspace(-10,10,N);
    forceOne = linspace(-10,10,N);
    forceTwo = linspace(-50,50,N);
    forceThree = linspace(-500,500,N);
    forceFour = linspace(-1000,1000,N);
    forceFive = linspace(-5000,5000,N);
    forceSix = linspace(-10000,10000,N);

    D1 = zeros(nD,N^4);
    D2 = zeros(nD,N^4);
    D3 = zeros(nD,N^4);
    D4 = zeros(nD,N^4);
    D5 = zeros(nD,N^4);
    D6 = zeros(nD,N^4);
    cntr = 1;
    tic;
    % Create all possible states
    for a=1:N
        for b=1:N
            for c=1:N
                for d=1:N
                        D1(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                      forceOne(d); forceTwo(d); forceThree(d);
                                      forceFour(d); forceFive(d); forceSix(d)];
                        D2(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                      forceSix(d); forceOne(d); forceTwo(d);
                                      forceThree(d); forceFour(d); forceFive(d)];
                        D3(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                      forceFive(d); forceSix(d); forceOne(d);
                                      forceTwo(d); forceThree(d); forceFour(d)];
                        D4(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                      forceFour(d); forceFive(d); forceSix(d);
                                      forceOne(d); forceTwo(d); forceThree(d)];   
                        D5(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                      forceThree(d); forceFour(d); forceFive(d);
                                      forceSix(d); forceOne(d); forceTwo(d)]; 
                        D6(:,cntr) = [vx(a); vy(b); dTheta(c); 
                                      forceTwo(d); forceThree(d); forceFour(d);
                                      forceFive(d); forceSix(d); forceOne(d)];          
                       cntr = cntr + 1;
                       disp(strcat([num2str(a),' ',num2str(b),' ',num2str(c), ' ', num2str(d)]));
                end
            end
        end
    end
    toc;
    dataset = [D1, D2, D3, D4, D5, D6]; % create dataset matrix
    [~, mD] = size(dataset);    % number of samples
 %%   
    x = dataset(1:nx,:);
    u = dataset(nx+1:end,:);

    % create system propagation samples
    xNext = zeros(nx,mD);
    for i=1:mD
        xNext(:,i) = VehicleModelDiscrete(x(:,i),u(:,i), Ts);
        disp(strcat(['Prediction step: ', num2str(i),'/',num2str(mD)]));
    end
    % Clear unnecessary data and save the rest
    clear D1 D2 D3 D4 D5 D6...
        forceOne forceTwo forceThree forceFour forceFive forceSix...
        cntr vx vy dTheta a b c d i;
    save dataset.mat;

else
    % otherwise load the existing dataset
    disp('Dataset loaded!');
    load dataset.mat;
    
end

%% CREATE SYSTEM USING MPT TOOLBOX
% Create Koopman approximation
vehicle = LoadVehicleParameters();

orderMPT = 5;
b = mpt.basis.poly(nx, orderMPT);
b{end+1} = '1';
b{end+1} = 'atan2(x(2) + 1.435*x(3), x(1) - 0.846*x(3))'; % front left slip angle component
b{end+1} = 'atan2(x(2) + 1.435*x(3), x(1) + 0.846*x(3))'; % front right slip angle component
b{end+1} = 'atan2(x(2) - 1.31*x(3), x(1) - 0.846*x(3))'; % real left slip angle
b{end+1} = 'atan2(x(2) - 1.31*x(3), x(1) + 0.846*x(3))'; % real right slip angle
b = unique(b,'stable');
sysKoopMPT = KoopmanSystem.fitx(x, u, xNext, b, Ts,'linsolve',2);
