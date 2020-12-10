function [] = HardcodeLinearization(states, input, dStates)
%HARDCODE LINEARIZATION
%   The functions creates MATLAB script with hardcoded linearized system
%   model to avoid symbolic linearization in every step.

% Linearize the model

A = jacobian(dStates, states);
B = jacobian(dStates, input);
[n,m] = size(B);

% Write to file
fileID = fopen('LinearizedMatrices.m','w');

% Write function initialization
fprintf(fileID,'function [Ad, Bd] = LinearizedMatrices(states,input, T)');
fprintf(fileID, '\n%% LINEARIZED MATRICES\n');
fprintf(fileID, '%%   The function returns linearized model of a vehicle.\n\n');

% Write variable initialization
fprintf(fileID,'%% Initialize state variables\n');
fprintf(fileID,'vx = states(1);\nvy = states(2);\nomega = states(3);\n');

fprintf(fileID,'\n%% Initialize input variables\n');
fprintf(fileID,'Fxfl = input(1);\nFyfl = input(2);\nFxfr = input(3);\nFyfr = input(4);\nFxrl = input(5);\nFxrr = input(6);\n');

fprintf(fileID,'\n%% Limit vehicle longitudinal and lateral velocity values in order to \n%% prevent division by 0\n');
fprintf(fileID,'if (vx >= 0)\n\tvx = max(vx, 1e-6);\nelse\n\tvx = min(vx, -1e-6);\nend\n');
fprintf(fileID,'\nif (vy >= 0)\n\tvy = max(vy, 1e-6);\nelse\n\tvy = min(vy, -1e-6);\nend\n');

fprintf(fileID,'\n%% Initialize matrices\n');
fprintf(fileID,'nx = length(states);\nnu = length(input);\nA = zeros(nx,nx);\nB = zeros(nx,nu);\n');

% Write hardcoded matrices
% Matrix A
fprintf(fileID,'\n%% Define A matrix elements\n%% ROW 1\n');
for i=1:n
   for j=1:n
       write_str = strcat(['A(',num2str(i),',',num2str(j),') = %s;\n']);
       fprintf(fileID, write_str, char(A(i,j)));
   end
end

fprintf(fileID,'\n%% Define B matrix elements\n%% ROW 1\n');
for i=1:n
   for j=1:m
       write_str = strcat(['B(',num2str(i),',',num2str(j),') = %s;\n']);
       fprintf(fileID, write_str, char(B(i,j)));
   end
end

fprintf(fileID,'\n%% Create discrete system\n');
fprintf(fileID,'sysCont = ss(A, B, eye(nx), zeros(nx,nu));\n');
fprintf(fileID,'sysDisc = c2d(sysCont, T, ''tustin'');\n');
fprintf(fileID,'Ad = sysDisc.A;\nBd = sysDisc.B;\n');

% Write function ending
fprintf(fileID, '\nend');
fclose(fileID);

end

