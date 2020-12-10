function d = DeltaSignal(x0, u0, At, Bt, N, T)
% DELTA SIGNAL 
%   The functions creates and returns delta signal needed
%   to calculate deviations from a steady state.

d = cell(N,1);
for i=1:N
    newX = VehicleModelDiscrete(x0, u0, T);
    d{i} = newX - At*x0 - Bt*u0;
    x0 = newX;
end
end

