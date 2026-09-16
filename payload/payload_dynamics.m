classdef payload_dynamics < matlab.System
    properties (Access = private)
        gravity = 9.81;
        e3 = [0;0;1];
        quat;
    end

    properties (Access = public)
        mass = 0.225;
        inertia_matrix = diag([2.1,1.87,3.97]) * 0.01;
        Rattach = zeros(3,4);
        N = 4;
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.quat = Quaternions();
        end

        function [wc, xpp, wp_dot, quatp] = stepImpl(obj, T, Alpha, w, qp)
            wc     = obj.computeWrench(qp, T, Alpha);
            xpp    = obj.cartesianDynamics(T, Alpha);
            wp_dot = obj.attitudeDynamics(qp, T, Alpha, w);
            quatp  = obj.w2qtp(qp, w);
        end

        function resetImpl(~)
        end

        function [wc, xpp, wp_dot, quatp] = getOutputSizeImpl(~)
            wc     = [6 1];
            xpp    = [3 1];
            wp_dot = [3 1];
            quatp  = [4 1];
        end
        
        function [wc, xpp, wp_dot, quatp] = getOutputDataTypeImpl(~)
            wc     = 'double';
            xpp    = 'double';
            wp_dot = 'double';
            quatp  = 'double';
        end
        
        function [wc, xpp, wp_dot, quatp] = isOutputComplexImpl(~)
            wc     = false;
            xpp    = false;
            wp_dot = false;
            quatp  = false;
        end
        
        function [wc, xpp, wp_dot, quatp] = isOutputFixedSizeImpl(~)
            wc     = true;
            xpp    = true;
            wp_dot = true;
            quatp  = true;
        end
    end

    methods (Access = private)

        function xpp = cartesianDynamics(obj, T, Alpha)
            Nloc = size(Alpha,2);
            sumTa = zeros(3,1);

            for i = 1:Nloc
                sumTa = sumTa + T(i) * Alpha(:,i);
            end

            xpp = obj.gravity * obj.e3 - (1/obj.mass) * sumTa;
        end

        function wp_dot = attitudeDynamics(obj, qp, T, Alpha, w)
            M = zeros(3,1);
            %ToDo: Verify sign !

            for i = 1:size(Alpha,2)
                fiP = obj.rotate_by_quat_conj(qp, -T(i) * Alpha(:,i));
                M = M + cross(obj.Rattach(:,i), fiP);
            end

            wp_dot = obj.inertia_matrix \ (M - cross(w, obj.inertia_matrix * w));
        end

        function qtp = w2qtp(obj, qt, w)
            w_qt = [0; w(:)];
            qtp = 0.5 * obj.quat.product(qt, w_qt);
        end

        function wc = computeWrench(obj, qp, T, Alpha)
            F = zeros(3,1);
            M = zeros(3,1);

            for i = 1:size(Alpha,2)
                F = F - T(i) * Alpha(:,i);

                %ToDo: Verify sign !
                fiP = obj.rotate_by_quat_conj(qp, - T(i) * Alpha(:,i));
                M = M + cross(obj.Rattach(:,i), fiP);
            end

            wc = [F; M];
        end

        function vR = rotate_by_quat_conj(obj, q, v)
            qc = obj.quat.conj(q);
            vq = [0; v(:)];
            tmp = obj.quat.product(qc, vq);
            tmp = obj.quat.product(tmp, q);
            vR = tmp(2:4);
        end
    end
end