function d = DeltaSignal(x, u, At, Bt, N, T)
% DELTA SIGNAL 
%   The functions creates and returns delta signal needed
%   to calculate deviations from a steady state.

currX = x{1};
d = cell(N,1);
for i=1:N
    newX = VehicleModelDiscrete(currX, u{1}, T);
    d{i} = newX - At{1}*currX - Bt{1}*u{1};
    currX = newX;
end
end

