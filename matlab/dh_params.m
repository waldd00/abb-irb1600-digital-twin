function dh = dh_params()
% DH_PARAMS  Returns the Denavit-Hartenberg parameter table for ABB IRB 1600-6/1.45
%
% Output:
%   dh  — [6x3] matrix, columns: [a (mm), alpha (rad), d (mm)]
%         theta is NOT included here — it comes in as a joint variable at runtime.
%
% Joint limits (degrees):
%   J1: -180 to +180
%   J2:  -63 to +110
%   J3: -236 to  +60
%   J4: -200 to +200
%   J5: -115 to +115
%   J6: -400 to +400
%
% Reference: ABB IRB 1600 Product Specification (3HAC027340-001)
% Verify these values against the official ABB document before use.

dh = [
%   a (mm)   alpha (rad)   d (mm)
    150,      pi/2,         486;    % joint 1
    700,      0,            0;      % joint 2
    115,      pi/2,         0;      % joint 3
    0,       -pi/2,         625;    % joint 4
    0,        pi/2,         0;      % joint 5
    0,        0,            100;    % joint 6
];

end
