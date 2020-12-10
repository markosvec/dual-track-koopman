function [rotMatrix, dRotMatrix] = ForceRotationMatrices(steer, N, T)
%FORCE CONSTRAINTS
% Inputs in form u = [Fxfl Fyfl Fxfr Fyfr Fxrl Fxrr]'

    dSteer = (steer(2:end) - steer(1:end-1))/T; % steering angle derivative vector
    steer = steer(1:end-1); % take only N-1 steps into account 
    
    rotMatrixCell = cell(N,1);
    dRotMatrixCell = cell(N,1);
    for i=1:N
        rotMatrixCell{i} = [cos(steer(i)) sin(steer(i))   0             0              0 0;
                            -sin(steer(i))  cos(steer(i)) 0             0              0 0;
                             0              0             cos(steer(i)) sin(steer(i))  0 0;
                             0              0             -sin(steer(i)) cos(steer(i)) 0 0;
                             0              0             0             0              1 0;
                             0              0             0             0              0 1];
        dRotMatrixCell{i} = dSteer(i)*[-sin(steer(i))  cos(steer(i))    0               0                0 0;
                                       -cos(steer(i))  -sin(steer(i))   0               0                0 0;
                                       0               0                -sin(steer(i))  cos(steer(i))    0 0;
                                       0               0                -cos(steer(i))  -sin(steer(i))   0 0;
                                       0               0                 0              0                0 0;
                                       0               0                 0              0                0 0];
    end
    
    rotMatrix = cell2mat(rotMatrixCell');
    dRotMatrix = cell2mat(dRotMatrixCell');
end

