function dStates = VehicleModel(t, states, input)
% VEHICLE MODEL 
%   The function accepts current system states and inputs, and calculates 
%   state derivatives.

% State definition
vx = states(1);
vy = states(2);
omega = states(3);

% Input definition
Fxfl = input(1);
Fyfl = input(2);
Fxfr = input(3);
Fyfr = input(4);
Fxrl = input(5);
Fxrr = input(6);

% Initialize vehicle parameters
vehicle = LoadVehicleParameters();

% Vehicle parameters
lf = vehicle.lf;
lr = vehicle.lr;
w = vehicle.w;
m = vehicle.m;
Jz = vehicle.Jz;

% Air drag parameters
cw = vehicle.cw;
rho = vehicle.rho;
Aw = vehicle.Aw;

% VEHICLE VELOCITIES

% Lateral velocities
vyrl = vy - lr*omega;
vyrr = vy - lr*omega;
% Longitudinal velocities
vxrl = vx - w*omega;
vxrr = vx + w*omega;

% TIRE VELOCITIES

% Cornering velocities
vcrl = vyrl;
vcrr = vyrr;

% Longitudinal velocities
vlrl = vxrl;
vlrr = vxrr;

% TIRE SLIP ANGLES

alpha_rl = atan2(vcrl,vlrl);
alpha_rr = atan2(vcrr,vlrr);

% TIRE FORCES
Cy = vehicle.Cy;

% Conrnering forces
Fcrl = -Cy*alpha_rl;
Fcrr = -Cy*alpha_rr;

% AXLE FORCES

% Lateral forces
Fyrl = Fcrl;
Fyrr = Fcrr;

% VEHICLE DYNAMICS MODEL

dvx = vy*omega + 1/m *(-0.5*cw*rho*Aw*sqrt(vx^2+vy^2)*vx + Fxfl + Fxfr + Fxrl + Fxrr);
dvy = -vx*omega + 1/m *(-0.5*cw*rho*Aw*sqrt(vx^2+vy^2)*vy + Fyfl + Fyfr + Fyrl + Fyrr);
dOmega = lf/Jz*(Fyfl + Fyfr) - lr/Jz*(Fyrl + Fyrr) + w/Jz*(-Fxfl + Fxfr -Fxrl + Fxrr);

dStates = [dvx; dvy; dOmega];
end