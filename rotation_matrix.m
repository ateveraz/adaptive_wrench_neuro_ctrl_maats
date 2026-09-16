function R = rotation_matrix(alpha, axis)
%ROTATION_MATRIX Computes a 3x3 rotation matrix for a given angle and axis.
%
%   R = ROTATION_MATRIX(alpha, axis)
%
%   Inputs:
%       alpha - rotation angle (in radians)
%       axis  - character specifying axis: 'x', 'y', or 'z'
%
%   Output:
%       R - 3x3 rotation matrix

    alpha = alpha*(pi/180);

    switch lower(axis)
        case 'x'
            R = [1,           0,            0;
                 0, cos(alpha), -sin(alpha);
                 0, sin(alpha),  cos(alpha)];

        case 'y'
            R = [ cos(alpha), 0, sin(alpha);
                        0,    1,          0;
                 -sin(alpha), 0, cos(alpha)];

        case 'z'
            R = [cos(alpha), -sin(alpha), 0;
                 sin(alpha),  cos(alpha), 0;
                 0,           0,          1];

        otherwise
            error('Invalid axis. Use ''x'', ''y'', or ''z''.');
    end
end
