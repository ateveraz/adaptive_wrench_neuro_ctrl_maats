function ref = cable_geometry_references(xpd, qpd, Rattach, cable_lengths, alpha_d, Td, masses, a_uav_d)
    %CABLE_GEOMETRY_REFERENCES Generate UAV position and cable-force references.
    % alpha_d(:,i) is the desired direction UAV i -> payload attachment i.
    % xd_i = xpd + Rpd*r_i - d_i*alpha_d_i.
    N = size(Rattach,2);
    Rpd = quat_to_rot(qpd);
    if nargin < 8 || isempty(a_uav_d), a_uav_d=zeros(3,N); end
    ez=[0;0;1]; g=9.81;
    ref.xd=zeros(3,N); ref.alpha_d=zeros(3,N);
    ref.Ftension_d=zeros(3,N); ref.u_d=zeros(3,N);
    for i=1:N
        ai=alpha_d(:,i)/max(norm(alpha_d(:,i)),1e-12);
        ref.alpha_d(:,i)=ai;
        xai=xpd+Rpd*Rattach(:,i);
        ref.xd(:,i)=xai-cable_lengths(i)*ai;
        ref.Ftension_d(:,i)=Td(i)*ai; % Force of cable on UAV i.
        ref.u_d(:,i)=masses(i)*(a_uav_d(:,i)+g*ez)+ref.Ftension_d(:,i);
    end
end

function R=quat_to_rot(q)
    q=q(:)/max(norm(q),1e-12); q0=q(1); qv=q(2:4);
    S=[0 -qv(3) qv(2);qv(3) 0 -qv(1);-qv(2) qv(1) 0];
    R=(q0^2-qv.'*qv)*eye(3)+2*qv*qv.'+2*q0*S;
end
