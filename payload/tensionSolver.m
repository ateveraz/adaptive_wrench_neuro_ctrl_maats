function [Alpha, T, cable_error] = tensionSolver( uavs, thrusts, masses, payload, payload_mass, payload_inertia, Rattach, cable_lengths, wn_constraint, zeta_constraint)

    % Coordinate convention:
    % +ez points downward.
    % alpha_i points from UAV i to payload attachment i.
    % Cable force on UAV i:     +T_i alpha_i.
    % Cable force on payload:   -T_i alpha_i.

    g  = 9.81;
    ez = [0; 0; 1];

    N = length(uavs);

    Alpha = zeros(3, N);
    A     = zeros(N, N);
    b     = zeros(N, 1);

    cable_error = zeros(N, 1);

    xp = payload.position(:);
    vp = payload.linear_velocity(:);
    wp = payload.angular_velocity(:);
    qp = payload.quaternion(:);

    Rp = quat_to_rot(qp);

    % Free payload acceleration: no cable forces.
    ap0 = g * ez;

    % Free payload angular acceleration: no cable torques.
    wp_dot0 = payload_inertia \ ...
        (-cross(wp, payload_inertia * wp));

    % Cable-dependent contribution to payload angular acceleration:
    % wp_dot = wp_dot0 + sum_j B(:,j) T_j.
    B = zeros(3, N);

    % Variables associated with each attachment.
    xa    = zeros(3, N);
    va    = zeros(3, N);
    aA0   = zeros(3, N);
    au0   = zeros(3, N);
    vrel  = zeros(3, N);
    dgeom = zeros(N, 1);

    for i = 1:N

        ri = Rattach(:, i);

        % Position of attachment i in inertial frame.
        xa(:, i) = xp + Rp * ri;

        % Cable direction: UAV i --> attachment i.
        delta_i = xa(:, i) - uavs(i).position(:);
        dgeom(i) = norm(delta_i);

        if dgeom(i) < 1e-8
            Alpha(:, i) = ez;
            T = zeros(N, 1);
            cable_error(:) = 1e6;
            return
        end

        Alpha(:, i) = delta_i / dgeom(i);

        % Attachment velocity in inertial frame.
        va(:, i) = vp + Rp * cross(wp, ri);

        % Relative attachment-UAV velocity.
        vrel(:, i) = va(:, i) - uavs(i).linear_velocity(:);

        % Constraint-position residual: ell_i^2 - d_i^2.
        cable_error(i) = 0.5 * (dgeom(i)^2 - cable_lengths(i)^2);

        % u_i := f_i R(q_i, ez).
        ui = thrusts(i) * rotate_by_quat(uavs(i).quaternion(:), ez);

        % Free UAV acceleration, i.e. T_i = 0:
        % m_i a_i = -u_i + m_i g ez.
        au0(:, i) = -ui / masses(i) + g * ez;

        % The force applied on payload is -T_i alpha_i.
        % It generates the following coefficient in payload angular acceleration.
        alpha_i_P = Rp.' * Alpha(:, i);

        B(:, i) = payload_inertia \ ...
            (-cross(ri, alpha_i_P));

        % Free acceleration of attachment point i.
        aA0(:, i) = ap0 + Rp * (cross(wp_dot0, ri) + cross(wp, cross(wp, ri)));
    end

    % Assemble A*T = b.
    for i = 1:N

        ri = Rattach(:, i);
        alpha_i = Alpha(:, i);

        % Baumgarte stabilization:
        %
        % phi_i = 1/2*(||xa_i - xi||^2 - d_i^2)
        %
        % Enforces:
        % phi_ddot + 2*zeta*wn*phi_dot + wn^2*phi = 0.
        %
        % This is NOT a spring model. It compensates numerical drift
        % when integrating a rigid holonomic constraint.

        ell_dot_i = alpha_i.' * vrel(:, i);

        rhs_stab = ...
            norm(vrel(:, i))^2 / dgeom(i) + ...
            2 * zeta_constraint * wn_constraint * ell_dot_i + ...
            (wn_constraint^2 / dgeom(i)) * cable_error(i);

        % Free relative acceleration projected on cable direction.
        free_relative_acc = alpha_i.' * (aA0(:, i) - au0(:, i));

        b(i) = free_relative_acc + rhs_stab;

        for j = 1:N

            alpha_j = Alpha(:, j);

            % Contribution of T_j to the translational acceleration
            % of payload: -(T_j/mp)*alpha_j.
            Ctrans_ij = -alpha_j / payload_mass;

            % Contribution of T_j to payload angular acceleration
            % and then to acceleration of attachment i.
            Crot_ij = Rp * cross(B(:, j), ri);

            % Contribution of T_j to relative acceleration:
            % a_attachment_i - a_UAV_i.
            Cij = Ctrans_ij + Crot_ij;

            % T_i also accelerates UAV i in +alpha_i.
            if i == j
                Cij = Cij - alpha_i / masses(i);
            end

            % We solve A*T = b, with A = -alpha_i' Cij.
            A(i, j) = -alpha_i.' * Cij;
        end
    end

    % Rigid-cable Lagrange multipliers.
    T = A \ b;

end


%% ===== Quaternion helpers =====

function R = quat_to_rot(q)

    q = q(:);
    q = q / norm(q);

    q0 = q(1);
    qv = q(2:4);

    R = (q0^2 - qv.' * qv) * eye(3) + ...
        2 * (qv * qv.') + ...
        2 * q0 * skew(qv);

end

function S = skew(v)

    S = [  0    -v(3)  v(2);
          v(3)  0    -v(1);
         -v(2)  v(1)  0   ];

end

function vI = rotate_by_quat(q, v)

    vq = [0; v(:)];
    tmp = quat_mult(q(:), vq);
    tmp = quat_mult(tmp, quat_conj(q(:)));
    vI = tmp(2:4);

end

function q = quat_conj(qin)

    q = [qin(1); -qin(2:4)];

end

function q = quat_mult(p, r)

    q = [p(1) * r(1) - p(2:4).' * r(2:4);
         p(1) * r(2:4) + r(1) * p(2:4) + ...
         cross(p(2:4), r(2:4))];

end