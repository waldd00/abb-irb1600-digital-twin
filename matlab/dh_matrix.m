function A = dh_matrix(theta, d, a, alpha)
% DH_MATRIX  Computes the 4x4 homogeneous DH transformation matrix for one joint.
%
% Standard DH convention:
%   Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
%
% Inputs:
%   theta  — joint angle (rad)   [variable]
%   d      — joint offset (mm)   [from dh_params]
%   a      — link length (mm)    [from dh_params]
%   alpha  — link twist (rad)    [from dh_params]
%
% Output:
%   A      — [4x4] homogeneous transformation matrix

ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

A = [
    ct,  -st*ca,   st*sa,  a*ct;
    st,   ct*ca,  -ct*sa,  a*st;
    0,    sa,      ca,     d;
    0,    0,       0,      1
];

end
