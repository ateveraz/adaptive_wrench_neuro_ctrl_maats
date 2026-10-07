function [G, Alpha] = build_cable_wrench_matrix(Rattach, payload, uavs)
%BUILD_CABLE_WRENCH_MATRIX Cable wrench map for any number of UAVs.
% alpha_i points from UAV i to payload attachment i.
% Wrench convention: [force in I; moment in P].
    N = size(Rattach,2);

    qp = payload.quaternion(:);
    xp = payload.position(:);

    %assert(isequal(size(xuav),[3,N]), 'xuav must be 3xN.');
    Rp = quat_to_rot(qp);
    G = zeros(6,N);
    Alpha = zeros(3,N);
    for i = 1:N
        ri = Rattach(:,i);
        xai = xp + Rp*ri;
        delta = xai - uavs(i).position(:);
        ell = norm(delta);
        assert(ell > 1e-8, 'Degenerate cable geometry.');
        alpha = delta/ell;
        Alpha(:,i) = alpha;
        fiP = Rp.'*(-alpha);      % Unit cable force on payload in P.
        G(:,i) = [-alpha; cross(ri,fiP)];
    end
end

function R = quat_to_rot(q)
    q = q(:)/max(norm(q),1e-12);
    q0=q(1); qv=q(2:4);
    S=[0 -qv(3) qv(2); qv(3) 0 -qv(1); -qv(2) qv(1) 0];
    R=(q0^2-qv.'*qv)*eye(3)+2*(qv*qv.')+2*q0*S;
end
