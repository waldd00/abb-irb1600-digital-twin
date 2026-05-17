function T = forward_kinematics(q)
% FORWARD_KINEMATICS  Computes the end-effector transform for the ABB IRB 1600.
%
% Multiplies the 6 individual DH transformation matrices:
%   T = A1(q1) * A2(q2) * A3(q3) * A4(q4) * A5(q5) * A6(q6)
%
% Input:
%   q  — [6x1] or [1x6] joint angles in radians
%
% Output:
%   T  — [4x4] homogeneous transformation matrix (base frame → end-effector)
%        T(1:3, 4) = [x; y; z] position in mm
%        T(1:3, 1:3) = rotation matrix

q   = q(:);          % ensure column vector
dh  = dh_params();   % load DH table [a, alpha, d]
T   = eye(4);

for i = 1:6
    a     = dh(i, 1);
    alpha = dh(i, 2);
    d     = dh(i, 3);

    Ai = dh_matrix(q(i), d, a, alpha);
    T  = T * Ai;
end

end
