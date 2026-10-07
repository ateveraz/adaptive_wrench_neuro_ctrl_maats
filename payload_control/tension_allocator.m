classdef tension_allocator < matlab.System
    % TENSION_ALLOCATOR Bounded projected-gradient cable-tension allocator.
    % Solves approximately:
    % min_T ||G*T-wc_proj||^2 + rhoT||T||^2
    %       + rhoDelta||T-Tprevious||^2
    % subject to Tmin <= T <= Tmax.
    % The allocator produces a reference Td; it does not replace the
    % physical Lagrange-multiplier tension calculated by rigid_cable_states.
    properties (Access = public)
        N = 2;
        Tmin = 0.0;
        Tmax = 20.0;
        RhoT = 1e-6;
        RhoDelta = 1e-3;
        Iterations = 200;
    end
    properties (Access = private)
        Tprevious;
    end
    methods (Access = protected)
        function setupImpl(obj)
            obj.Tprevious=zeros(obj.N,1);
        end
        function resetImpl(obj)
            obj.Tprevious=zeros(obj.N,1);
        end
        function [Td,wc_alloc,residual] = stepImpl(obj,G,wc_proj)
            I=eye(obj.N);
            H=G.'*G+(obj.RhoT+obj.RhoDelta)*I;
            f=-G.'*wc_proj(:)-obj.RhoDelta*obj.Tprevious;

            % Frobenius norm is an upper bound on the spectral norm and
            % yields a conservative projected-gradient step size.
            step=1/max(norm(H,'fro'),1e-9);
            Td=obj.Tprevious;
            Td=min(max(Td,obj.Tmin),obj.Tmax);
            for k=1:obj.Iterations
                Tnew=Td-step*(H*Td+f);
                Tnew=min(max(Tnew,obj.Tmin),obj.Tmax);
                if norm(Tnew-Td) < 1e-10*(1+norm(Td))
                    Td=Tnew;
                    break
                end
                Td=Tnew;
            end
            obj.Tprevious=Td;
            wc_alloc=G*Td;
            residual=wc_alloc-wc_proj(:);
        end
        function [a,b,c]=getOutputSizeImpl(obj)
            a=[obj.N 1]; b=[6 1]; c=[6 1];
        end
        function [a,b,c]=getOutputDataTypeImpl(~)
            a='double'; b='double'; c='double';
        end
        function [a,b,c]=isOutputComplexImpl(~)
            a=false; b=false; c=false;
        end
        function [a,b,c]=isOutputFixedSizeImpl(~)
            a=true; b=true; c=true;
        end
    end
end
