clear; clc;

% Extract states
vx = sym('vx','real');
vy = sym('vy','real');
omega = sym('omega','real');

% Extract inputs
deltaF = sym('deltaF','real');
sfl = sym('sfl','real');
sfr = sym('sfr','real');
srl = sym('srl','real');
srr = sym('srr','real');

% Initialize vehicle parameters
vehicle = LoadVehicleParameters();

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

% Longitudinal velocities
vxfl = vx - w*omega;
vxfr = vx + w*omega;
vxrl = vx - w*omega;
vxrr = vx + w*omega;
% Lateral velocities
vyfl = vy + lf*omega;
vyfr = vy + lf*omega;
vyrl = vy - lr*omega;
vyrr = vy - lr*omega;

% TYRE VELOCITIES

% Longitudinal velocities
vlfl = vxfl*cos(deltaF) + vyfl*sin(deltaF);
vlfr = vxfr*cos(deltaF) + vyfr*sin(deltaF);
vlrl = vxrl;
vlrr = vxrr;

% Cornering velocities
vcfl = -vxfl*sin(deltaF) + vyfl*cos(deltaF);
vcfr = -vxfr*sin(deltaF) + vyfr*cos(deltaF);
vcrl = vyrl;
vcrr = vyrr;

% TYRE SLIP ANGLES
alpha_fl = atan2(vcfl,vlfl);
alpha_fr = atan2(vcfr,vlfr);
alpha_rl = atan2(vcrl,vlrl);
alpha_rr = atan2(vcrr,vlrr);

% TYRE FORCES
Cx = vehicle.Cx;
Cy = vehicle.Cy;

% Longitudinal forces
Flfl = Cx*sfl;
Flfr = Cx*sfr;
Flrl = Cx*srl;
Flrr = Cx*srr;

% Conrnering forces
Fcfl = -Cy*alpha_fl;
Fcfr = -Cy*alpha_fr;
Fcrl = -Cy*alpha_rl;
Fcrr = -Cy*alpha_rr;

% AXLE FORCES

% Longitudinal forces
Fxfl = Flfl*cos(deltaF) - Fcfl*sin(deltaF);
Fxfr = Flfr*cos(deltaF) - Fcfr*sin(deltaF);
Fxrl = Flrl;
Fxrr = Flrr;

% Lateral forces
Fyfl = Flfl*sin(deltaF) + Fcfl*cos(deltaF);
Fyfr = Flfr*sin(deltaF) + Fcfr*cos(deltaF);
Fyrl = Fcrl;
Fyrr = Fcrr;

% VEHICLE DYNAMICS MODEL

dvx = vy*omega + 1/m*(-0.5*cw*rho*Aw*sqrt(vx^2+vy^2)*vx + Fxfl + Fxfr + Fxrl + Fxrr);
dvy = -vx*omega + 1/m*(-0.5*cw*rho*Aw*sqrt(vx^2+vy^2)*vy + Fyfl + Fyfr + Fyrl + Fyrr);
dOmega = lf/Jz*(Fyfl + Fyfr) - lr/Jz*(Fyrl + Fyrr) + w/Jz*(-Fxfl + Fxfr -Fxrl + Fxrr);

dStates = [dvx; dvy; dOmega];
states = [vx; vy; omega];
input = [deltaF; sfl; sfr; srl; srr];

HardcodeLinearization(states, input, dStates);
