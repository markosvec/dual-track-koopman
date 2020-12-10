function output = TransformInputs(states, inputs, steering0, slip0)
%TRANSFORM INPUTS

% Extract force
forces = inputs(1:6);
steerAng = inputs(7);
Fxrl = forces(5);
Fxrr = forces(6);

% Load parameters
vehicle = LoadVehicleParameters();
Cx = vehicle.Cx;

fun = @(x) solveEquation(x, forces, states, steerAng, vehicle);
options = optimoptions('fsolve','Display','off');
solution = fsolve(fun, slip0, options);

sfl = solution(1); % front wheel slip
sfr = solution(2);
srl = Fxrl/Cx; % rear left wheel slip
srr = Fxrr/Cx; % rear right wheel slip

output = [steerAng; sfl; sfr; srl; srr];

end

function F = solveEquation(x,forces,states, steer, vehicle)

    sfl = x(1);
    sfr = x(2);

    % Load parameters
    lf = vehicle.lf; % Distance between the centre of gravity and the front axle
    w = vehicle.w; % Half of the vehicles width
    Cx = vehicle.Cx;
    Cy = vehicle.Cy;

    % Extract forces
    Fxfl = forces(1);
    Fyfl = forces(2);
    Fxfr = forces(3);
    Fyfr = forces(4);

    % Extract states
    vx = states(1);
    vy = states(2);
    omega = states(3);

    % Longitudinal velocities
    vxfl = vx - w*omega;
    vxfr = vx + w*omega;
    % Lateral velocities
    vyfl = vy + lf*omega;
    vyfr = vy + lf*omega;
    
    % Longitudinal velocities
    vlfl = vxfl*cos(steer) + vyfl*sin(steer);
    vlfr = vxfr*cos(steer) + vyfr*sin(steer);
    % Cornering velocities
    vcfl = -vxfl*sin(steer) + vyfl*cos(steer);
    vcfr = -vxfr*sin(steer) + vyfr*cos(steer);

    % TYRE SLIP ANGLES
    alpha_fl = atan2(vcfl,vlfl);
    alpha_fr = atan2(vcfr,vlfr);

    F(1) = Cx*sfl*cos(steer) - (-Cy*alpha_fl)*sin(steer) - Fxfl;
    F(2) = Cx*sfr*cos(steer) - (-Cy*alpha_fr)*sin(steer) - Fxfr;
    F(3) = Cx*sfl*sin(steer) + (-Cy*alpha_fl)*cos(steer) - Fyfl;
    F(4) = Cx*sfr*sin(steer) + (-Cy*alpha_fr)*cos(steer) - Fyfr;
end
