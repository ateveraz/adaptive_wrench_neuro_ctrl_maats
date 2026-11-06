classdef neuro_payload_controller < matlab.System
    % NEURO_PAYLOAD_CONTROLLER Wrench-feasible adaptive LIP-perceptron
    % controller (stateless block; weight integration is done OUTSIDE with a
    % Simulink 1/s block).
    %
    % LIP = linear-in-the-parameters. Single hidden layer with FIXED inner
    % weights V and biases b, tanh activation, and adaptive outer weights:
    %
    %   Phi(zeta)   = tanh( V*zn + b ),   zn = zeta ./ ZetaScale     (m0x1)
    %   (optional constant feature appended -> m = m0 + 1)
    %   Delta_hat   = What' * Phi                                    (rx1)
    %
    % For the r controllable coordinates selected by S:
    %
    %   sbar_p   = [ ev + Lambda_x*ex ; ew + Lambda_R*<q_e>_R ]      (6x1)
    %   s_p      = S' * sbar_p                                       (rx1)
    %   wc_raw   = -S*Ks*s_p + S*S'*wff - S*Delta_hat                (6x1)
    %   What_dot = Gamma*Phi*s_p' - Mu*What                          (mxr)
    %
    % wc_raw is the desired CABLE wrench (gravity excluded):
    % [force in I ; moment in P]. +e3 points downward, so hover gives
    % wc_raw = [0;0;-mp*g;0;0;0].
    %
    % Wiring: What_dot -> Integrator (IC = zeros(m,r)) -> What -> input 7.
    % The loop closes through the integrator, so it is not algebraic.
    % UseNeuro = false gives the nominal baseline (Delta_hat = 0, What_dot = 0).
    %
    % Inputs : xd, xdp, xdpp, qd, omegad, payload (kinematic bus), What
    % Outputs: wc_raw (6x1), sp (rx1), Delta_hat (rx1), zeta (2r x 1),
    %          What_dot (m x r)

    properties (Nontunable)
        mass = 0.225;
        S = [0 0; 0 0; 1 0; 0 0; 0 1; 0 0];   

        Ks = diag([4.0, 0.25]);
        Lambda_x = diag([0, 0, 1.5]);
        Lambda_R = diag([0, 2.0, 0]);

        UseNeuro = true;
        V = 0.5*ones(8, 4);                          % m0 x nzeta, fixed
        b = zeros(8, 1);                             % m0 x 1, fixed
        UseBias = true;                              % constant output feature
        Gamma = 5.0;                                 % Gamma_p = Gamma*I
        Mu = 0.5;                                    % leakage
        ZetaScale = [0.2; 0.1; 0.5; 0.5];            % normalization of zeta
        DeltaMax = [Inf; Inf];                       % optional output clamp
    end

    properties (Access = private)
        quat;
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.quat = Quaternions();
            r = size(obj.S, 2);
            assert(size(obj.V, 2) == 2*r, ...
                'V must have 2*r columns: zeta = [S''*e_pos ; S''*e_vel].');
            assert(numel(obj.b) == size(obj.V, 1), ...
                'b must have one entry per hidden neuron.');
            assert(numel(obj.ZetaScale) == 2*r, ...
                'ZetaScale must have 2*r entries.');
            assert(numel(obj.DeltaMax) == r, ...
                'DeltaMax must have r entries.');
        end

        function [wc_raw, sp, Delta_hat, zeta, What_dot] = stepImpl(obj, xd, xdp, xdpp, qd, omegad, payload, What)

            r = size(obj.S, 2);

            [sp, zeta] = obj.computeManifold(xd, xdp, qd, omegad, payload);
            Phi = obj.features(zeta);
            
            if obj.UseNeuro
                Delta_hat = What.' * Phi;
                % Delta_hat = min(max(Delta_hat, -obj.DeltaMax(:)), obj.DeltaMax(:));
                What_dot = (obj.Gamma*(Phi*sp.') - obj.Mu*What);
            else
                Delta_hat = zeros(r, 1);
                What_dot = zeros(size(What));
            end

            % Known feedforward: wff = [mp*(xdd_d - g*e3); 0].
            wff = [obj.mass*(xdpp(:) - 9.81*[0; 0; 1]); zeros(3, 1)];

            % wc_raw = -obj.S*(obj.Ks*sp) + obj.S*(obj.S.'*wff) - obj.S*Delta_hat;
            % size(wff);

            % size(sp);

            % size(obj.Ks)
            
            wc_raw = -obj.Ks*sp + wff; % - obj.S*Delta_hat;
        end

        function [a, b, c, d, e] = getOutputSizeImpl(obj)
            r = size(obj.S, 2);
            m = size(obj.V, 1);
            a = [6 1];
            b = [6 1];
            c = [r 1];
            d = [size(obj.V, 2) 1];
            e = [m r];
        end

        function [a, b, c, d, e] = getOutputDataTypeImpl(~)
            a = 'double'; b = 'double'; c = 'double';
            d = 'double'; e = 'double';
        end

        function [a, b, c, d, e] = isOutputComplexImpl(~)
            a = false; b = false; c = false; d = false; e = false;
        end

        function [a, b, c, d, e] = isOutputFixedSizeImpl(~)
            a = true; b = true; c = true; d = true; e = true;
        end
    end

    methods (Access = private)

        function Phi = features(obj, zeta)
            zn = zeta(:); % ./ obj.ZetaScale(:);
            Phi = tanh(obj.V*zn);          % m0 x 1, in (-1,1)
            % Phi = tanh(zn);
            % 
            % if obj.UseBias
            %     Phi = [Phi; 1.0];
            % end
        end

        function [sp, zeta] = computeManifold(obj, xd, xdp, qd, omegad, payload)
            x = payload.position(:);
            v = payload.linear_velocity(:);
            q = obj.normalize_quat(payload.quaternion(:));
            w = payload.angular_velocity(:);
            qd = obj.normalize_quat(qd(:));

            % Translational errors in I.
            ex = x - xd(:);
            ev = v - xdp(:);

            % qe = qd^* (x) q, with nonnegative scalar part.
            qe = obj.quat.product(obj.quat.conj(qd), q);
            qe = obj.normalize_quat(qe);
            if qe(1) < 0
                qe = -qe;
            end

            % Attitude errors in P. <qe>_R is invariant under Rot(qe*, .).
            eR = qe(2:4);
            ew = w - obj.rotate_by_quat_conj(qe, omegad(:));

            sbar = [ev + obj.Lambda_x*ex;
                    ew + obj.Lambda_R*eR];
            % sp = obj.S.' * sbar;
            sp = sbar;

            % Regressor on the controllable coordinates.
            e_pos = [ex; eR];
            e_vel = [ev; ew];
            zeta = [obj.S.' * e_pos; obj.S.' * e_vel];
        end

        function q = normalize_quat(~, q)
            q = q(:) / max(norm(q), 1e-12);
        end

        function vP = rotate_by_quat_conj(obj, q, v)
            qc = obj.quat.conj(q);
            tmp = obj.quat.product(qc, [0; v(:)]);
            tmp = obj.quat.product(tmp, q);
            vP = tmp(2:4);
        end
    end
end
