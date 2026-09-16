classdef PositionController < matlab.System
    % PositionController Controlador de posición

    properties (Access = public)
        Kd = 2.0;
        alpha = 1.0;
        mass = 0.4;
    end

    properties (Access = private)
        gravity = [0 0 9.81]';
    end

    methods (Access = protected)
        function setupImpl(~)
        end

        function [u, up] = stepImpl(obj, xLdpp, Td, e, ep) 
            
            s = ep + obj.alpha * e;

            u = - obj.Kd * s + obj.mass * (obj.gravity - xLdpp) + Td; 

            up = zeros(3,1); % Fix it ? 
        end

        function resetImpl(~)
        end

        function [u, up] = getOutputSizeImpl(obj)
            u  = [3 1];
            up = [3 1];
        end

        function [u, up] = getOutputDataTypeImpl(~)
            u = 'double';
            up = 'double';
        end

        function [u, up] = isOutputComplexImpl(~)
            u = false;
            up = false;
        end

        function [u, up] = isOutputFixedSizeImpl(~)
            u = true;
            up = true;
        end
    end
end