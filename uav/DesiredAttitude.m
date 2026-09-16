classdef DesiredAttitude < matlab.System
    % DesiredAttitude 

    properties (Access = private)
        quat;
        Fb = [0 0 0 1]';
    end


    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.quat = Quaternions();
        end

        function [qe, we] = stepImpl(obj, fu, fup, w, q)
            yaw_d = 0;
            fu_norm = fu / norm(fu);
            quatF = obj.quat.product([0 ; fu_norm], obj.quat.conj(obj.Fb));
            qd_xy = obj.quat.exp(obj.quat.log(quatF)/2);
            qd_yaw = obj.quat.exp(yaw_d*obj.Fb/2);
            qd = obj.quat.product(qd_yaw, qd_xy);

            qe = obj.computeQuaternionError(qd, q);

            Fuph = (eye(3) - fu * fu') * fup / norm(fu);
            Fuph = [0; Fuph];
            Fuh = [0; fu];

            wfd = obj.quat.product(obj.quat.product(obj.quat.product(obj.Fb, obj.quat.conj(Fuh)), Fuph), obj.quat.conj(obj.Fb));
            wd = obj.quat.product(obj.quat.product(obj.quat.conj(qd_yaw), wfd), qd_yaw);

            we = w - wd(2:4);
        end

        function resetImpl(~)
            % Initialize / reset internal properties
        end

        function [qd, wd] = getOutputSizeImpl(~)
            qd = [4 1];
            wd = [3 1];
        end

        function [qd, wd] = getOutputDataTypeImpl(~)
            qd = 'double';
            wd = 'double';
        end

        function [qd, wd] = isOutputComplexImpl(~)
            qd = false;
            wd = false;
        end

        function [qd, wd] = isOutputFixedSizeImpl(~)
            qd = true;
            wd = true;
        end
    end

    methods (Access = private)
        function qe = computeQuaternionError(obj, qd, q)
            qd_conj = obj.quat.conj(qd);
            qe = obj.quat.product(qd_conj, q);
        end

        function qd = desiredQuaternion(obj, fu)
            qd = [0 0 0 0]';
            qd(1) = dot(obj.e3, fu);
            qd(2:4) = cross(obj.e3, fu);
            qd = qd / norm(qd);
        end
    end
end