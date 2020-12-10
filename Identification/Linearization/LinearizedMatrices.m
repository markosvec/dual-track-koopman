function [Ad, Bd] = LinearizedMatrices(states,input, T)
% LINEARIZED MATRICES
%   The function returns linearized model of a vehicle.

% Initialize state variables
vx = states(1);
vy = states(2);
omega = states(3);

% Initialize input variables
Fxfl = input(1);
Fyfl = input(2);
Fxfr = input(3);
Fyfr = input(4);
Fxrl = input(5);
Fxrr = input(6);

% Limit vehicle longitudinal and lateral velocity values in order to 
% prevent division by 0
if (vx >= 0)
	vx = max(vx, 1e-6);
else
	vx = min(vx, -1e-6);
end

if (vy >= 0)
	vy = max(vy, 1e-6);
else
	vy = min(vy, -1e-6);
end

% Initialize matrices
nx = length(states);
nu = length(input);
A = zeros(nx,nx);
B = zeros(nx,nu);

% Define A matrix elements
% ROW 1
A(1,1) = - (1023*(vx^2 + vy^2)^(1/2))/4375000 - (1023*vx^2)/(4375000*(vx^2 + vy^2)^(1/2));
A(1,2) = omega - (1023*vx*vy)/(4375000*(vx^2 + vy^2)^(1/2));
A(1,3) = vy;
A(2,1) = - omega - (25744*((131*omega)/100 - vy))/(875*(((423*omega)/500 + vx)^2 + ((131*omega)/100 - vy)^2)) - (25744*((131*omega)/100 - vy))/(875*(((423*omega)/500 - vx)^2 + ((131*omega)/100 - vy)^2)) - (1023*vx*vy)/(4375000*(vx^2 + vy^2)^(1/2));
A(2,2) = (25744*((423*omega)/500 - vx))/(875*(((423*omega)/500 - vx)^2 + ((131*omega)/100 - vy)^2)) - (1023*(vx^2 + vy^2)^(1/2))/4375000 - (1023*vy^2)/(4375000*(vx^2 + vy^2)^(1/2)) - (25744*((423*omega)/500 + vx))/(875*(((423*omega)/500 + vx)^2 + ((131*omega)/100 - vy)^2));
A(2,3) = (25744*(131/(100*((423*omega)/500 + vx)) - (423*((131*omega)/100 - vy))/(500*((423*omega)/500 + vx)^2))*((423*omega)/500 + vx)^2)/(875*(((423*omega)/500 + vx)^2 + ((131*omega)/100 - vy)^2)) - (25744*((423*omega)/500 - vx)^2*(131/(100*((423*omega)/500 - vx)) - (423*((131*omega)/100 - vy))/(500*((423*omega)/500 - vx)^2)))/(875*(((423*omega)/500 - vx)^2 + ((131*omega)/100 - vy)^2)) - vx;
A(3,1) = (1063043052578856605*((131*omega)/100 - vy))/(36028797018963968*(((423*omega)/500 + vx)^2 + ((131*omega)/100 - vy)^2)) + (1063043052578856605*((131*omega)/100 - vy))/(36028797018963968*(((423*omega)/500 - vx)^2 + ((131*omega)/100 - vy)^2));
A(3,2) = (1063043052578856605*((423*omega)/500 + vx))/(36028797018963968*(((423*omega)/500 + vx)^2 + ((131*omega)/100 - vy)^2)) - (1063043052578856605*((423*omega)/500 - vx))/(36028797018963968*(((423*omega)/500 - vx)^2 + ((131*omega)/100 - vy)^2));
A(3,3) = (1063043052578856605*((423*omega)/500 - vx)^2*(131/(100*((423*omega)/500 - vx)) - (423*((131*omega)/100 - vy))/(500*((423*omega)/500 - vx)^2)))/(36028797018963968*(((423*omega)/500 - vx)^2 + ((131*omega)/100 - vy)^2)) - (1063043052578856605*(131/(100*((423*omega)/500 + vx)) - (423*((131*omega)/100 - vy))/(500*((423*omega)/500 + vx)^2))*((423*omega)/500 + vx)^2)/(36028797018963968*(((423*omega)/500 + vx)^2 + ((131*omega)/100 - vy)^2));

% Define B matrix elements
% ROW 1
B(1,1) = 1/1750;
B(1,2) = 0;
B(1,3) = 1/1750;
B(1,4) = 0;
B(1,5) = 1/1750;
B(1,6) = 1/1750;
B(2,1) = 0;
B(2,2) = 1/1750;
B(2,3) = 0;
B(2,4) = 1/1750;
B(2,5) = 0;
B(2,6) = 0;
B(3,1) = -6826747806805897/18446744073709551616;
B(3,2) = 2894912264410893/4611686018427387904;
B(3,3) = 6826747806805897/18446744073709551616;
B(3,4) = 2894912264410893/4611686018427387904;
B(3,5) = -6826747806805897/18446744073709551616;
B(3,6) = 6826747806805897/18446744073709551616;

% Create discrete system
sysCont = ss(A, B, eye(nx), zeros(nx,nu));
sysDisc = c2d(sysCont, T, 'tustin');
Ad = sysDisc.A;
Bd = sysDisc.B;

end