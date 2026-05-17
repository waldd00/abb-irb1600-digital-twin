function J = jacobian(q)
% JACOBIAN  Computes the 6x6 geometric Jacobian for the ABB IRB 1600.
%
% The Jacobian relates joint velocities to end-effector velocities:
%   [v; omega] = J(q) * dq/dt
%
% Linear velocity columns  (J_v): numerical partial derivative of position
% Angular velocity columns (J_w): z-axis of frame i-1 (geometric method)
%
% Input:
%   q  — [6x1] joint angles in radians
%
% Output:
%   J  — [6x6] geometric Jacobian matrix
%        Rows 1-3: linear velocity  (mm/s per rad/s)
%        Rows 4-6: angular velocity (rad/s per rad/s)

q        = q(:);
n_joints = 6;
J        = zeros(6, n_joints);
epsilon  = 1e-7;          % perturbation for numerical differentiation

dh = dh_params();

% Current end-effector position
T0 = forward_kinematics(q);
p0 = T0(1:3, 4);

for i = 1:n_joints

    % ── Linear velocity column: ∂p/∂qi ─────────────────────────────────────
    q_pert    = q;
    q_pert(i) = q_pert(i) + epsilon;
    T_pert    = forward_kinematics(q_pert);
    J(1:3, i) = (T_pert(1:3, 4) - p0) / epsilon;

    % ── Angular velocity column: z-axis of frame (i-1) ─────────────────────
    T_partial = eye(4);
    for k = 1:(i-1)
        T_partial = T_partial * dh_matrix(q(k), dh(k,3), dh(k,1), dh(k,2));
    end
    J(4:6, i) = T_partial(1:3, 3);   % z-axis of frame i-1

end

end
