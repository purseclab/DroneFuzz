function [dXdt, y] = quadcopterEOM(t, X, U, m, Ix, Iy, Iz, l, b, d, varargin)
% quadcopterEOM - Continuous-time quadcopter dynamics for idnlgrey.
%
% States (12):
%   X(1)  = phi    (roll angle)
%   X(2)  = theta  (pitch angle)
%   X(3)  = psi    (yaw angle)
%   X(4)  = p      (roll rate)
%   X(5)  = q      (pitch rate)
%   X(6)  = r      (yaw rate)
%   X(7)  = x      (position in X)
%   X(8)  = y      (position in Y)
%   X(9)  = z      (position in Z)
%   X(10) = vx     (velocity along X)
%   X(11) = vy     (velocity along Y)
%   X(12) = vz     (velocity along Z)
%
% Inputs (4): Rotor speeds [Omega1, Omega2, Omega3, Omega4]
% Parameters (7): [m; Ix; Iy; Iz; l; b; d]
%
% Outputs (9):  y = [phi, theta, psi, p, q, r, x, y, z]
dbstop if error

    %-----------------------------
    % 0) Extract states
    %-----------------------------
    phi   = X(1);
    theta = X(2);
    psi   = X(3);
    p     = X(4);
    q     = X(5);
    r     = X(6);
    vx  = X(7);
    vy  = X(8);
    vz  = X(9);
    
    %-----------------------------
    % 1) Extract parameters
    %-----------------------------
    % m  = params(1);   % mass
    % Ix = params(2);
    % Iy = params(3);
    % Iz = params(4);
    % l  = params(5);
    % b  = params(6);   % thrust constant
    % d  = params(7);   % drag (yaw) constant

    %-----------------------------
    % 2) Extract inputs
    %-----------------------------
    Om1 = U(1);
    Om2 = U(2);
    Om3 = U(3);
    Om4 = U(4);

    %-----------------------------
    % 3) Compute net forces/torques
    %-----------------------------
    % Thrust
    U_t = b * (Om1^2 + Om2^2 + Om3^2 + Om4^2);

    % Roll torque
    U_phi = b * l * (Om2^2 - Om4^2);

    % Pitch torque
    U_theta = b * l * (Om3^2 - Om1^2);

    % Yaw torque
    U_psi = d * (Om1^2 + Om3^2 - Om2^2 - Om4^2);

    %-----------------------------
    % 4) Compute derivatives
    %-----------------------------
    % (a) Angular kinematics
    dphi   = p;
    dtheta = q;
    dpsi   = r;

    % (b) Angular dynamics
    dp = (U_phi / Ix) + (q*r)*(Iy - Iz)/Ix;
    dq = (U_theta / Iy) + (p*r)*(Iz - Ix)/Iy;
    dr = (U_psi / Iz) + (p*q)*(Ix - Iy)/Iz;

    % (c) Position kinematics
    dx = vx;
    dy = vy;
    dz = vz;

    % (d) Translational dynamics
    dvx = (U_t/m)*( cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi) );
    dvy = (U_t/m)*( cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi) );
    dvz = (U_t/m)*( cos(phi)*cos(theta) ) - 9.81;  % - g

    %-----------------------------
    % 5) Collect derivatives
    %-----------------------------
    dXdt = [
      dphi
      dtheta
      dpsi
      dp
      dq
      dr
      dx
      dy
      dz
      dvx
      dvy
      dvz
    ];

    %-----------------------------
    % 6) Define outputs (9)
    %-----------------------------
    % The measured outputs in your iddata are:
    %   X_output = [phi, theta, psi, p, q, r, x, y, z]
    % so we match that here:
    y = [
      phi
      theta
      psi
      p
      q
      r
      vx
      vy
      vz
    ];
end