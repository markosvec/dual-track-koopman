function vehicle = LoadVehicleParameters()
% LOAD VEHICLE PARAMETERS
%   The function creates and returns vehicle object based on the specified
%   parameters.

lf = 1.435; % distance between the centre of gravity and the front axle
lr = 1.31;  % distance between the centre of gravity and the rear axle
w = 0.846;  % half of the vehicles width
m = 1750;   % mass
Jz = 2286;  % moment of inertia around z axis

cw = 0.31;  % air drag coefficient
rho = 1.2;  % air density
Aw = 2.2;   % projected area in a transversal view

Cx = 87712; % tire longitudinal stiffness
Cy = 51488; % tire cornering stiffness

vehicle = Vehicle(m, Jz, lf, lr, w, Cx, Cy, cw, rho, Aw); % create vehicle object
end

