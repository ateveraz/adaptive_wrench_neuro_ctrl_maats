classdef nominal_controller < matlab.System
    % NOMINAL_CONTROLLER Nominal outer-loop cable-wrench controller.
    %
    % Input payload is a bus with fields:
    %   position, linear_velocity, quaternion, angular_velocity.
    %
    % Outputs:
    %   wc_raw = [Fc_d^I; Mc_d^P], desired wrench exerted BY CABLES
    %            on the payload.
    %   Sp     = [sx; sR], tracking manifold.
    %
    % Convention: +e3 points downward. Hence the payload translation is
    % mp*xpp = mp*g*e3 + Fc. The hover cable wrench is Fc=-mp*g*e3.

    properties (Access = public)
        mass = 0.225;
        Ks = diag([0, 0, 4.0, 0, 0.25, 0]);
        Lambda_x = diag([0, 0, 1.5]);
        Lambda_R = diag([0, 2.0, 0]);
    end

    properties (Access = private)
        gravity = 9.81;
        e3 = [0 0 1]';
        quat;
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.quat = Quaternions();
        end

        function [wc_raw, Sp] = stepImpl(obj, xd, xdp, xdpp, qd, omegad, payload)
            x = payload.position(:);
            v = payload.linear_velocity(:);
            q = obj.normalize_quat(payload.quaternion(:));
            omega = payload.angular_velocity(:);

            qd = obj.normalize_quat(qd(:));

            % Translational manifold, expressed in I.
            ex = x - xd(:);
            ev = v - xdp(:);
            sx = ev + obj.Lambda_x*ex;

            % qe rotates vectors from current payload frame P to desired
            % payload frame Pd: qe = qd^* otimes q.
            qe = obj.quat.product(obj.quat.conj(qd), q);
            qe = obj.normalize_quat(qe);

            % q and -q represent the same physical attitude. Select the
            % shortest-error representative to avoid discontinuities.
            if qe(1) < 0
                qe = -qe;
            end

            % omegad is assumed expressed in Pd. Rotate it into current P.
            omega_d_P = obj.rotate_by_quat_conj(qe, omegad(:));

            % The vector part of qe is initially expressed in Pd. Rotate it
            % into P before combining it with omega.
            eR_P = obj.rotate_by_quat_conj(qe, qe(2:4));
            eomega = omega - omega_d_P;
            sR = eomega + obj.Lambda_R*eR_P;

            Sp = [sx; sR];

            % Nominal cable-force feedforward, expressed in I:
            % Fc_ff = mp*(xdd_d - g*e3).
            Fc_ff = obj.mass*(xdpp(:) - obj.gravity*obj.e3);

            % This first implementation is intended for attitude regulation
            % or slowly varying references. Angular feedforward is zero.
            % A full trajectory-tracking version must add the desired angular
            % acceleration and the corresponding geometric transport terms.
            Mc_ff = zeros(3,1);
            wc_ff = [Fc_ff; Mc_ff];

            % Nominal controller without NN-compensation
            wc_raw = -obj.Ks*Sp + wc_ff;
        end

        function [wc_raw, Sp] = getOutputSizeImpl(~)
            wc_raw = [6 1];
            Sp = [6 1];
        end

        function [wc_raw, Sp] = getOutputDataTypeImpl(~)
            wc_raw = 'double';
            Sp = 'double';
        end

        function [wc_raw, Sp] = isOutputComplexImpl(~)
            wc_raw = false;
            Sp = false;
        end

        function [wc_raw, Sp] = isOutputFixedSizeImpl(~)
            wc_raw = true;
            Sp = true;
        end
    end

    methods (Access = private)

        function q = normalize_quat(~, q)
            q = q(:);
            q = q/max(norm(q), 1e-12);
        end

        function vP = rotate_by_quat_conj(obj, q, v)
            % q maps P -> Pd. q^* maps Pd -> P.
            qc = obj.quat.conj(q);
            vq = [0; v(:)];
            tmp = obj.quat.product(qc, vq);
            tmp = obj.quat.product(tmp, q);
            vP = tmp(2:4);
        end
    end
end
