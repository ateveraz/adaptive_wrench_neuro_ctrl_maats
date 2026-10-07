function [G, Alpha, cable_lengths] = compute_cable_wrench_map(Rattach, uavs, payload)

    % BUILD_CABLE_WRENCH_MATRIX
    %
    % Builds the cable tension-to-payload wrench map:
    %
    %       wc = G*T
    %
    % Convention:
    %   alpha_i points from UAV i to payload attachment i.
    %
    % Cable force on UAV i:
    %   F_cable,UAV_i = +T_i*alpha_i
    %
    % Cable force on payload:
    %   F_cable,payload_i = -T_i*alpha_i
    %
    % Wrench convention:
    %   wc = [F_c^I; M_c^P]

    xp = payload.position(:);
    qp = payload.quaternion(:);

    Nattach = size(Rattach, 2);
    Nuav = numel(uavs);

    assert(Nattach == Nuav, ...
        'The number of attachment points must match the number of UAVs.');

    N = Nuav;

    G = zeros(6, N);
    Alpha = zeros(3, N);
    cable_lengths = zeros(N, 1);

    Rp = quat_to_rot(qp);

    for i = 1:N

        ri = Rattach(:, i);

        % Attachment position in inertial frame.
        xai = xp + Rp * ri;

        % Direction from UAV i to attachment point i.
        delta_i = xai - uavs(i).position(:);

        ell_i = norm(delta_i);

        assert(ell_i > 1e-8, ...
            'UAV %d is coincident with attachment point %d.', i, i);

        alpha_i = delta_i / ell_i;

        Alpha(:, i) = alpha_i;
        cable_lengths(i) = ell_i;

        % Unit cable force acting ON payload, expressed in I.
        force_unit_I = -alpha_i;

        % Same force expressed in payload frame P.
        force_unit_P = Rp.' * force_unit_I;

        % Unit moment about payload CoM, expressed in P.
        moment_unit_P = cross(ri, force_unit_P);

        % Column i of the cable wrench map.
        G(:, i) = [force_unit_I;
                   moment_unit_P];
    end
end


function R = quat_to_rot(q)

    q = q(:) / max(norm(q), 1e-12);

    q0 = q(1);
    qv = q(2:4);

    S = [   0,  -qv(3),  qv(2);
          qv(3),   0,   -qv(1);
         -qv(2), qv(1),    0   ];

    R = (q0^2 - qv.' * qv) * eye(3) + ...
        2 * (qv * qv.') + ...
        2 * q0 * S;
end