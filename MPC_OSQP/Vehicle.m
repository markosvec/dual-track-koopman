classdef Vehicle
    % VEHICLE
    %   Class which contains all relevant vehicle model parameters.
    
    properties
        m    % mass
        Jz   % moment of inertia around z axis
        lf   % distance between the centre of gravity and the front axle
        lr   % distance between the centre of gravity and the rear axle
        w    % half of the vehicles width
        Cx   % tire longitudinal stiffness
        Cy   % tire cornering stiffness
        cw   % air drag coefficient
        rho  % air density
        Aw   % projected area in a transversal view
    end
    
    methods
        function obj = Vehicle(m, Jz, lf, lr, w, Cx, Cy, cw, rho, Aw)
           % initialize vehicle object
           obj.m = m;
           obj.Jz = Jz;
           obj.lf = lf;
           obj.lr = lr;
           obj.w = w;
           obj.Cx = Cx;
           obj.Cy = Cy;
           obj.cw = cw;
           obj.rho = rho;
           obj.Aw = Aw; 
        end
    end
end

