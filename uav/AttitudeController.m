classdef AttitudeController < matlab.System
    % tau = - Kd*s - beta*tanh(gamma*s)
    %s = we + rho*qe(2:4)

    properties (Access = public)
        Kd = 2.0;
        beta = 0.5;
        gamma = 1.0;
        rho = 5.0;
    end
    
    properties (Access = private)
        gravity = [0 0 9.81]';
        Fb = [0 0 0 1]';
        e3 = [0 0 1]';
    end

    methods (Access = protected)
        function setupImpl(~)
        end

        function [tau, thrust] = stepImpl(obj, u, qe, we)
            s = we + obj.rho * qe(2:4);
            tau = - obj.Kd*s - obj.beta * tanh( obj.gamma * s);
            thrust = norm(u);
        end

        function resetImpl(~)
        end

        function [tau, thrust] = getOutputSizeImpl(~)
            tau = [3 1];
            thrust = [1 1];
        end

        function [tau, thrust] = getOutputDataTypeImpl(~)
            tau = 'double';
            thrust = 'double';
        end

        function [tau, thrust] = isOutputComplexImpl(~)
            tau = false;
            thrust = false;
        end

        function [tau, thrust] = isOutputFixedSizeImpl(~)
            tau = true;
            thrust = true;
        end
    end
end
