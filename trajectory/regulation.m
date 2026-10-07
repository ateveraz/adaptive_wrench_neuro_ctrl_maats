function [xd, xdp, xdpp, qd, omegad] = regulation(~)
    % Cartesian desired position
    xd   = [0 0 0]';
    xdp  = [0 0 0]';
    xdpp = [0 0 0]';
    % Quaternion-based desired attitude
    qd   = [1 0 0 0]';
    omegad = [0 0 0]';
end