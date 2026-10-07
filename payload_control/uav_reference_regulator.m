classdef uav_reference_regulator < matlab.System
    % UAV_REFERENCE_REGULATOR Fixed-geometry UAV references for payload
    % regulation. No alpha derivatives are required.
    %
    % Alpha_d is a prescribed, constant desired cable geometry in I.
    % For the initial two-UAV hover experiment set:
    % Alpha_d = [0 0; 0 0; 1 1].
    %
    % Input payload_d is a reference bus with fields:
    % position, linear_velocity, quaternion, angular_velocity.

    properties (Access = public)
        N = 2;
        Rattach = [0.5 -0.5; 0 0; 0 0];
        CableLengths = [0.8; 0.8];
        Alpha_d = [0 0; 0 0; 1 1];
    end

    properties (Access = private)
        quat;
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.quat = Quaternions();
        end

        function [Xuav_d, Vuav_d, Ftension_d] = stepImpl(obj, Td, payload_d)

            xd = payload_d.position(:);
            vd = payload_d.linear_velocity(:);
            qd = payload_d.quaternion(:);
            qd = qd/max(norm(qd),1e-12);
            omegad = payload_d.angular_velocity(:);

            Xuav_d = zeros(3,obj.N);
            Vuav_d = zeros(3,obj.N);
            Ftension_d = zeros(3,obj.N);

            for i = 1:obj.N
                ri = obj.Rattach(:,i);
                alpha_i_d = obj.Alpha_d(:,i);
                alpha_i_d = alpha_i_d/max(norm(alpha_i_d),1e-12);

                % Desired attachment kinematics.
                xai_d = xd + obj.rotate_by_quat(qd,ri);
                vai_d = vd + obj.rotate_by_quat( ...
                    qd,cross(omegad,ri));

                % alpha_i,d is constant in this regulation block:
                % dot(alpha_i,d) = 0.
                Xuav_d(:,i) = xai_d - ...
                    obj.CableLengths(i)*alpha_i_d;
                Vuav_d(:,i) = vai_d;

                % Desired cable force acting on UAV i, in I.
                Ftension_d(:,i) = Td(i)*alpha_i_d;
            end
        end

        function [a,b,c] = getOutputSizeImpl(obj)
            a=[3 obj.N]; b=[3 obj.N]; c=[3 obj.N];
        end

        function [a,b,c] = getOutputDataTypeImpl(~)
            a='double'; b='double'; c='double';
        end

        function [a,b,c] = isOutputComplexImpl(~)
            a=false; b=false; c=false;
        end

        function [a,b,c] = isOutputFixedSizeImpl(~)
            a=true; b=true; c=true;
        end
    end

    methods (Access = private)
        function vI = rotate_by_quat(obj,q,v)
            vq=[0;v(:)];
            tmp=obj.quat.product(q,vq);
            tmp=obj.quat.product(tmp,obj.quat.conj(q));
            vI=tmp(2:4);
        end
    end
end
