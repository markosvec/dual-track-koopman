function G = SlipAngleConstraints(steer, vehicle, slipAngMax, slipAngMin)
%SLIP ANGLE CONSTRAINTS
%   Function determines matrix which constraints slip angles. The initial
%   assumption is small angle approximation, i.e. atan(alpha) == alpha.

N = length(steer);
lf = vehicle.lf;
lr = vehicle.lr;
w = vehicle.w;

Gcell = cell(N,1);
for i=1:N
    % Cornering velocities
    vcflVect = [-sin(steer(i)) cos(steer(i)) (lf*cos(steer(i)) + w*sin(steer(i)))];
    vcfrVect = [-sin(steer(i)) cos(steer(i)) (lf*cos(steer(i)) - w*sin(steer(i)))];
    vcrlVect = [0 1 -lr];
    vcrrVect = [0 1 -lr];
    % Longitudinal velocities
    vlflVect = [cos(steer(i)) sin(steer(i)) (lf*sin(steer(i)) - w*cos(steer(i)))];
    vlfrVect = [cos(steer(i)) sin(steer(i)) (lf*sin(steer(i)) + w*cos(steer(i)))];
    vlrlVect = [1 0 -w];
    vlrrVect = [1 0 w];
    
    Gcell{i} = [vcflVect - slipAngMax*vlflVect;
               -vcflVect + slipAngMin*vlflVect;
                vcfrVect - slipAngMax*vlfrVect;
               -vcfrVect + slipAngMin*vlfrVect;
                vcrlVect - slipAngMax*vlrlVect;
               -vcrlVect + slipAngMin*vlrlVect;
                vcrrVect - slipAngMax*vlrrVect;
               -vcrrVect + slipAngMin*vlrrVect];
end

G = reshape(cell2mat(Gcell'),[8,3,N]);

end

