clear all; clc;

x0 = [20/3.6; 0; 0/180*pi];
Tsim = 10;
Ts = 0.01;
vehicle = LoadVehicleParameters();

Np = 20; %prediction step number WATCH OUT FOR THIS!!!

% Initialize reference parameters

% Sine with dwell
Tref = 4;
fref = 0.5;
Aref = 10;
Tdref = 0.5;
sim('RefGenSWD');

[nSignal,nRef] = size(reference.signals.values);

sim('ControlLoopSim');

%% Plot states
figure; subplot(3,1,1); 
plot(states.time, 3.6*states.signals.values(:,1)); hold;
plot(reference.time, 3.6*reference.signals.values(:,1), '--r');
grid; xlabel('t(s)'); ylabel('v_x(km/h)'); title('States');  axis([0 Tsim -inf inf]);
subplot(3,1,2);
plot(states.time, 3.6*states.signals.values(:,2)); hold;
plot(reference.time, 3.6*reference.signals.values(:,2), '--r');
grid; xlabel('t(s)'); ylabel('v_y(km/h)');  axis([0 Tsim -inf inf]);
subplot(3,1,3);
plot(states.time, 180/pi*states.signals.values(:,3)); hold;
plot(reference.time, 180/pi*reference.signals.values(:,3), '--r');
grid; xlabel('t(s)'); ylabel('\theta(°)');  axis([0 Tsim -inf inf]);

% Plot input and diagnostics
figure;
subplot(5,1,1);
plot(input.time, 180/pi*input.signals.values(:,1),'r');
grid; xlabel('t(s)'); ylabel('delta_{f}(deg)');
subplot(5,1,2);
plot(input.time, 100*input.signals.values(:,2),'r');
grid; xlabel('t(s)'); ylabel('s_{fl}(%)');
subplot(5,1,3);
plot(input.time, 100*input.signals.values(:,3),'r');
grid; xlabel('t(s)'); ylabel('s_{fr}(%)');
subplot(5,1,4);
plot(input.time, 100*input.signals.values(:,4),'r');
grid; xlabel('t(s)'); ylabel('s_{rl}(%)');
subplot(5,1,5);
plot(input.time, 100*input.signals.values(:,5),'r');
grid; xlabel('t(s)'); ylabel('s_{rr}(%)');

% Plot slip angles
figure;
subplot(4,1,1);
plot(input.time, 180/pi*slipAngles.signals.values(:,1));
grid; xlabel('t(s)'); ylabel('\alpha_{fl}(deg)');
subplot(4,1,2);
plot(input.time, 180/pi*slipAngles.signals.values(:,2));
grid; xlabel('t(s)'); ylabel('\alpha_{fr}(deg)');
subplot(4,1,3);
plot(input.time, 180/pi*slipAngles.signals.values(:,3));
grid; xlabel('t(s)'); ylabel('\alpha_{rl}(deg)');
subplot(4,1,4);
plot(input.time, 180/pi*slipAngles.signals.values(:,4));
grid; xlabel('t(s)'); ylabel('\alpha_{rr}(deg)');

% Plot and calculate duration and objective
figure;
subplot(2,1,1);
plot(duration.time, duration.signals.values); grid;
xlabel('t(s)'); ylabel('\Delta t(s)');
subplot(2,1,2);
plot(objective.time, objective.signals.values); grid;
xlabel('t(s)'); ylabel('Objective');

meanDuration = mean(duration.signals.values)
medianDuration = median(duration.signals.values)
maxDuration = max(duration.signals.values)
minDuration = min(duration.signals.values)

%% Closed loop objective
H = [1 0 0; 0 0 1];
Q = [1 0; 0 100];
p = 1e8;

sampleNum = length(states.time);
trackingDiff = H*(states.signals.values - reference.signals.values(1:sampleNum,:))';
trackingCost = sum(diag(trackingDiff'*Q*trackingDiff));

aboveMaxIndex = slipAngles.signals.values > 5/180*pi;
belowMinIndex = slipAngles.signals.values < -5/180*pi;
epsMax = (slipAngles.signals.values - 5/180*pi).*aboveMaxIndex;
epsMin = (-slipAngles.signals.values + 5/180*pi).*belowMinIndex;
slackCost = p*sum(diag(epsMax*epsMax')) + p*sum(diag(epsMin*epsMin'));

totalCost = trackingCost + slackCost