function T = partial_fk(q, n)
% PARTIAL_FK  Computes the transform from base to frame n (partial chain).
%
% Used for live animation: each link mesh is positioned using its own
% partial FK transform, not the full end-effector transform.
%
% Inputs:
%   q  — [6x1] joint angles in radians
%   n  — integer 1..6, how many joints to include in the chain
%
% Output:
%   T  — [4x4] homogeneous transformation matrix (base → frame n)

q   = q(:);
dh  = dh_params();
T   = eye(4);

for i = 1:n
    a     = dh(i, 1);
    alpha = dh(i, 2);
    d     = dh(i, 3);

    Ai = dh_matrix(q(i), d, a, alpha);
    T  = T * Ai;
end

end
