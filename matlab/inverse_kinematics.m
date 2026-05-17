function q_sol = inverse_kinematics(T_desired, q_init)
% INVERSE_KINEMATICS  Numerically solves IK for the ABB IRB 1600.
%
% Uses lsqnonlin with a 12-element residual (3 position + 9 rotation).
% The full rotation matrix difference avoids the 180-degree singularity
% that occurs with the skew-symmetric formulation (where sin(pi)=0
% causes the error to vanish at 180-degree rotations).
%
% Inputs:
%   T_desired — [4x4] desired end-effector homogeneous transform
%   q_init    — [6x1] initial joint angle guess in radians (optional)
%               Default: zeros(6,1)
%
% Output:
%   q_sol     — [6x1] solution joint angles in radians

if nargin < 2
    q_init = zeros(6, 1);
end

q_init = q_init(:);

p_des = T_desired(1:3, 4);
R_des = T_desired(1:3, 1:3);

% Joint limits in radians
lb = deg2rad([-180, -63,  -236, -200, -115, -400]);
ub = deg2rad([ 180,  110,   60,  200,  115,  400]);

opts = optimoptions('lsqnonlin', ...
    'Display',                'off',  ...
    'TolFun',                 1e-12,  ...
    'TolX',                   1e-12,  ...
    'MaxIterations',          5000,   ...
    'MaxFunctionEvaluations', 50000);

cost_fn = @(q) ik_residual(q, p_des, R_des);

% ── Attempt 1: user-supplied initial guess ────────────────────────────────
best_q   = lsqnonlin(cost_fn, q_init, lb, ub, opts);
best_err = norm(cost_fn(best_q));

% ── Multi-start: random restarts if first attempt not good enough ─────────
N_RESTARTS  = 10;
GOOD_ENOUGH = 1e-6;

rng(42);
for k = 1:N_RESTARTS
    if best_err < GOOD_ENOUGH
        break
    end
    q_rand = lb' + rand(6,1) .* (ub' - lb');
    q_k    = lsqnonlin(cost_fn, q_rand, lb, ub, opts);
    err_k  = norm(cost_fn(q_k));
    if err_k < best_err
        best_err = err_k;
        best_q   = q_k;
    end
end

q_sol = best_q;

end


% ── Residual (12 elements) ────────────────────────────────────────────────────
function residual = ik_residual(q, p_des, R_des)
% IK_RESIDUAL  12-element residual: 3 position (mm) + 9 rotation matrix diff.
%
% ROT_SCALE balances units between mm position error and dimensionless
% rotation matrix element difference (range 0..2).
% ROT_SCALE = 500 makes a 1mm position error ~ 1/500 rotation error.

ROT_SCALE = 500;

T = forward_kinematics(q);
p = T(1:3, 4);
R = T(1:3, 1:3);

pos_error = p - p_des;                      % [3x1] mm
rot_error = (R_des(:) - R(:)) * ROT_SCALE;  % [9x1] scaled

residual = [pos_error; rot_error];           % [12x1]

end