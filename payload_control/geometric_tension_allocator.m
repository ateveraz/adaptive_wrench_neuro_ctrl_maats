classdef geometric_tension_allocator < matlab.System
    % GEOMETRIC_TENSION_ALLOCATOR Direct geometric wrench inversion.
    %
    % For a projected wrench wc_proj in Im(G), this block computes the
    % least-squares/direct solution
    %
    %   Td = (G'*G)^(-1) G'*wc_proj,
    %
    % whenever G has full column rank. No cost function, no SQP, and no
    % tension smoothing are used. This is intended as the '-Geo' baseline.
    %
    % Important: Td is deliberately NOT clipped to zero. A negative result
    % means that the requested wrench is incompatible with taut unilateral
    % cables in the chosen geometry. Clipping would destroy G*Td=wc_proj.

    properties (Access = public)
        N = 2;
        ConditioningThreshold = 1e-9;
    end

    methods (Access = protected)

        function [Td, wc_geo, residual, min_tension, well_conditioned] = ...
                stepImpl(obj, G, wc_proj)

            H = G.' * G;
            rhs = G.' * wc_proj(:);

            if rcond(H) > obj.ConditioningThreshold
                Td = H \ rhs;
                well_conditioned = 1.0;
            else
                % Diagnostic fallback near singular configurations.
                % This uses the Moore--Penrose solution but preserves the
                % same geometric interpretation.
                Td = pinv(G) * wc_proj(:);
                well_conditioned = 0.0;
            end

            wc_geo = G * Td;
            residual = wc_geo - wc_proj(:);
            min_tension = min(Td);
        end

        function [a,b,c,d,e] = getOutputSizeImpl(obj)
            a = [obj.N 1];
            b = [6 1];
            c = [6 1];
            d = [1 1];
            e = [1 1];
        end

        function [a,b,c,d,e] = getOutputDataTypeImpl(~)
            a = 'double';
            b = 'double';
            c = 'double';
            d = 'double';
            e = 'double';
        end

        function [a,b,c,d,e] = isOutputComplexImpl(~)
            a = false;
            b = false;
            c = false;
            d = false;
            e = false;
        end

        function [a,b,c,d,e] = isOutputFixedSizeImpl(~)
            a = true;
            b = true;
            c = true;
            d = true;
            e = true;
        end
    end
end
