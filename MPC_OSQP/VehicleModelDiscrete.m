function statesUpdate = VehicleModelDiscrete(states, input, Ts)
% VEHICLE MODEL DISCRETE
%   The function accepts current system states and inputs, and calculates 
%   updated discrete system states using ode23s method.

    tspan = [0 Ts];
    [~, Y] = ode45(@(t,x)VehicleModel(t,x,input), tspan, states);
    statesUpdate = Y(end,:)';

end