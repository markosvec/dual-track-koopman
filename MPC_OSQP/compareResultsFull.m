%clear; clc;
load('koopSim5.mat');
load('linSim5.mat');

%% Plot states
figure; subplot(3,1,1);
plot(states.time, 3.6*states.signals.values(:,1)); hold;
plot(statesKoop.time, 3.6*statesKoop.signals.values(:,1));
plot(reference.time, 3.6*reference.signals.values(:,1), '--r');
grid; xlabel('t(s)'); ylabel('v_x(km/h)'); title('States');
subplot(3,1,2);
plot(states.time, 3.6*states.signals.values(:,2)); hold;
plot(statesKoop.time, 3.6*statesKoop.signals.values(:,2));
plot(reference.time, 3.6*reference.signals.values(:,2), '--r');
grid; xlabel('t(s)'); ylabel('v_y(km/h)');
subplot(3,1,3);
plot(states.time, 180/pi*states.signals.values(:,3)); hold;
plot(statesKoop.time, 180/pi*statesKoop.signals.values(:,3));
plot(reference.time, 180/pi*reference.signals.values(:,3), '--r');
grid; xlabel('t(s)'); ylabel('\theta(°)');

% Plot input and diagnostics
figure;
subplot(5,1,1);
plot(input.time, 180/pi*input.signals.values(:,1)); hold;
plot(inputKoop.time, 180/pi*inputKoop.signals.values(:,1));
grid; xlabel('t(s)'); ylabel('delta_{f}(deg)');
subplot(5,1,2);
plot(input.time, 100*input.signals.values(:,2)); hold;
plot(inputKoop.time, 100*inputKoop.signals.values(:,2));
grid; xlabel('t(s)'); ylabel('s_{fl}(%)');
subplot(5,1,3);
plot(input.time, 100*input.signals.values(:,3)); hold;
plot(inputKoop.time, 100*inputKoop.signals.values(:,3));
grid; xlabel('t(s)'); ylabel('s_{fr}(%)');
subplot(5,1,4);
plot(input.time, 100*input.signals.values(:,4)); hold;
plot(inputKoop.time, 100*inputKoop.signals.values(:,4)); 
grid; xlabel('t(s)'); ylabel('s_{rl}(%)');
subplot(5,1,5);
plot(input.time, 100*input.signals.values(:,5)); hold;
plot(inputKoop.time, 100*inputKoop.signals.values(:,5));
grid; xlabel('t(s)'); ylabel('s_{rr}(%)');

% Plot slip angles
figure;
subplot(4,1,1);
plot(input.time, 180/pi*slipAngles.signals.values(:,1)); hold;
plot(inputKoop.time, 180/pi*slipAnglesKoop.signals.values(:,1));
grid; xlabel('t(s)'); ylabel('\alpha_{fl}(deg)');
subplot(4,1,2);
plot(input.time, 180/pi*slipAngles.signals.values(:,2)); hold;
plot(inputKoop.time, 180/pi*slipAnglesKoop.signals.values(:,2));
grid; xlabel('t(s)'); ylabel('\alpha_{fr}(deg)');
subplot(4,1,3);
plot(input.time, 180/pi*slipAngles.signals.values(:,3)); hold;
plot(inputKoop.time, 180/pi*slipAnglesKoop.signals.values(:,3));
grid; xlabel('t(s)'); ylabel('\alpha_{rl}(deg)');
subplot(4,1,4);
plot(input.time, 180/pi*slipAngles.signals.values(:,4)); hold;
plot(inputKoop.time, 180/pi*slipAnglesKoop.signals.values(:,4));
grid; xlabel('t(s)'); ylabel('\alpha_{rr}(deg)');

% Plot objective values
figure;
plot(objective.time, objective.signals.values); hold;
plot(objective.time, 1000*1e-15*objectiveKoop.signals.values);
grid; xlabel('t(s)'); ylabel('Objective'); 
