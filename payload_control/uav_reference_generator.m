classdef uav_reference_generator < matlab.System
    % UAV_REFERENCE_GENERATOR Generates UAV references from a desired
    % payload trajectory and a prescribed desired cable geometry.
    %
    % Input payload_d is a REFERENCE bus, not the measured payload-state
    % bus. It must contain:
    %   position, linear_velocity, linear_acceleration,
    %   quaternion, angular_velocity, angular_acceleration.
    %
    % alpha_d(:,i) points from UAV i to attachment i on the payload.
    % Outputs are 3xN matrices; column i belongs to UAV i.

    properties (Access = public)
        N = 2;
        Rattach = [0.5 -0.5; 0 0; 0 0];
        CableLengths = [0.8; 0.8];
    end

    properties (Access = private)
        quat;
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.quat = Quaternions();
        end

        function [Xuav_d, Vuav_d, Auav_d, Ftension_d] = ...
                stepImpl(obj, Td, Alpha_d, Alpha_dp, Alpha_dpp, payload_d)

            xd = payload_d.position(:);
            vd = payload_d.linear_velocity(:);
            ad = payload_d.linear_acceleration(:);

            qd = payload_d.quaternion(:);
            qd = qd / max(norm(qd), 1e-12);

            omegad = payload_d.angular_velocity(:);
            omegadp = payload_d.angular_acceleration(:);

            Xuav_d = zeros(3, obj.N);
            Vuav_d = zeros(3, obj.N);
            Auav_d = zeros(3, obj.N);
            Ftension_d = zeros(3, obj.N);

            for i = 1:obj.N

                ri = obj.Rattach(:,i);

                alpha_i_d = Alpha_d(:,i);
                alpha_i_d = alpha_i_d / ...
                    max(norm(alpha_i_d), 1e-12);

                alpha_i_dp = Alpha_dp(:,i);
                alpha_i_dpp = Alpha_dpp(:,i);

                % Desired attachment-point kinematics in I.
                xai_d = xd + obj.rotate_by_quat(qd, ri);

                vai_d = vd + obj.rotate_by_quat( ...
                    qd, cross(omegad, ri));

                aai_d = ad + obj.rotate_by_quat(qd, ...
                    cross(omegadp, ri) + ...
                    cross(omegad, cross(omegad, ri)));

                % Rigid-cable geometric references.
                Xuav_d(:,i) = xai_d - ...
                    obj.CableLengths(i)*alpha_i_d;

                Vuav_d(:,i) = vai_d - ...
                    obj.CableLengths(i)*alpha_i_dp;

                Auav_d(:,i) = aai_d - ...
                    obj.CableLengths(i)*alpha_i_dpp;

                % Desired cable force ON UAV i, expressed in I.
                Ftension_d(:,i) = Td(i)*alpha_i_d;
            end
        end

        function [a,b,c,d] = getOutputSizeImpl(obj)
            a = [3 obj.N];
            b = [3 obj.N];
            c = [3 obj.N];
            d = [3 obj.N];
        end

        function [a,b,c,d] = getOutputDataTypeImpl(~)
            a = 'double';
            b = 'double';
            c = 'double';
            d = 'double';
        end

        function [a,b,c,d] = isOutputComplexImpl(~)
            a = false;
            b = false;
            c = false;
            d = false;
        end

        function [a,b,c,d] = isOutputFixedSizeImpl(~)
            a = true;
            b = true;
            c = true;
            d = true;
        end
    end

    methods (Access = private)

        function vI = rotate_by_quat(obj, q, v)
            vq = [0; v(:)];
            tmp = obj.quat.product(q, vq);
            tmp = obj.quat.product(tmp, obj.quat.conj(q));
            vI = tmp(2:4);
        end
    end
end
