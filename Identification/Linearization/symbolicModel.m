clear; clc;

% Extract states
vx = sym('vx','real');
vy = sym('vy','real');
omega = sym('omega','real');

% Extract inputs
Fxfl = sym('Fxfl','real');
Fyfl = sym('Fyfl','real');
Fxfr = sym('Fxfr','real');
Fyfr = sym('Fyfr','real');
Fxrl = sym('Fxrl','real');
Fxrr = sym('Fxrr','real');

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
states = [vx; vy; omega];
input = [Fxfl; Fyfl; Fxfr; Fyfr; Fxrl; Fxrr];

HardcodeLinearization(states, input, dStates);


