function dXdt = quadcopterEOM(t, X, U, m, Ix, Iy, Iz, l, b, d, varargin)
    % quadcopterEOM - Computes the time derivatives of the quadcopter states.
    %
    % Inputs:
    %   t          - current time (not used explicitly here)
    %   X          - state vector [12x1]
    %   U          - input vector [4x1] (rotor speeds: [Omega1, Omega2, Omega3, Omega4])
    %   parameters - parameter vector [7x1]
    %                [m; Ix; Iy; Iz; l; b; d]
    %
    % Output:
    %   dXdt       - time derivative of state vector [12x1]

    % Extract states for readability
    phi     = X(1);
    theta   = X(2);
    psi     = X(3);
    vx      = X(7);
    vy      = X(8);
    vz      = X(9);
    w_phi   = X(4);
    w_theta = X(5);
    w_psi   = X(6);

    % % Extract parameters
    % m  = parameters(1);
    % Ix = parameters(2);
    % Iy = parameters(3);
    % Iz = parameters(4);
    % l  = parameters(5);
    % b  = parameters(6);
    % d  = parameters(7);

    % Extract inputs (rotor speeds)
    Om1 = U(1);
    Om2 = U(2);
    Om3 = U(3);
    Om4 = U(4);

    %--------------------------------------------------------------------------
    % 1) Compute control inputs from rotor speeds
    %--------------------------------------------------------------------------
    U_t     = b * (Om1^2 + Om2^2 + Om3^2 + Om4^2);                 % Thrust
    U_phi   = b * l * (Om2^2 - Om4^2);                             % Roll torque
    U_theta = b * l * (Om3^2 - Om1^2);                             % Pitch torque
    U_psi   = d * (Om1^2 + Om3^2 - Om2^2 - Om4^2);                 % Yaw torque

    %--------------------------------------------------------------------------
    % 2) Derivatives of position/angles
    %--------------------------------------------------------------------------
    dx     = vx;
    dy     = vy;
    dz     = vz;
    dphi   = w_phi;
    dtheta = w_theta;
    dpsi   = w_psi;

    %--------------------------------------------------------------------------
    % 3) Translational accelerations
    %--------------------------------------------------------------------------
    dvx = (U_t / m) * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
    dvy = (U_t / m) * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
    dvz = (U_t / m) * cos(phi) * cos(theta) - 9.81;

    %--------------------------------------------------------------------------
    % 4) Rotational accelerations
    %--------------------------------------------------------------------------
    dw_phi   = (U_phi / Ix) + (w_theta * w_psi) * ((Iy - Iz) / Ix);
    dw_theta = (U_theta / Iy) + (w_phi * w_psi) * ((Iz - Ix) / Iy);
    dw_psi   = (U_psi / Iz) + (w_phi * w_theta) * ((Ix - Iy) / Iz);

    %--------------------------------------------------------------------------
    % Collect derivatives into dXdt
    %--------------------------------------------------------------------------
    dXdt       = zeros(12, 1);
    % Position and orientation derivatives
    dXdt(1:6)  = [dx; dy; dz; dphi; dtheta; dpsi];
    % Velocity and angular velocity derivatives
    dXdt(7:12) = [dvx; dvy; dvz; dw_phi; dw_theta; dw_psi];
end
