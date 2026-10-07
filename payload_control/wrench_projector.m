classdef wrench_projector < matlab.System
    % WRENCH_PROJECTOR Orthogonally projects a desired wrench onto Im(G).
    % Input:  G (6xN), wc_raw (6x1).
    % Outputs: wc_proj, wc_perp, PG, rankG, sigma.
    properties (Access = public)
        N = 2;
        RankTolerance = 1e-8;
    end
    methods (Access = protected)
        function [wc_proj,wc_perp,PG,rankG,sigma] = stepImpl(obj,wc_raw,G)
            [U,S,~] = svd(G,'econ');
            sigma = zeros(obj.N,1);
            sigma(1:min(obj.N,size(S,1))) = diag(S);
            PG = zeros(6,6);
            rankG = 0;
            for i=1:obj.N
                if sigma(i) > obj.RankTolerance
                    ui=U(:,i);
                    PG=PG+ui*ui.';
                    rankG=rankG+1;
                end
            end
            wc_proj=PG*wc_raw(:);
            wc_perp=wc_raw(:)-wc_proj;
        end
        function [a,b,c,d,e]=getOutputSizeImpl(obj)
            a=[6 1]; b=[6 1]; c=[6 6]; d=[1 1]; e=[obj.N 1];
        end
        function [a,b,c,d,e]=getOutputDataTypeImpl(~)
            a='double'; b='double'; c='double'; d='double'; e='double';
        end
        function [a,b,c,d,e]=isOutputComplexImpl(~)
            a=false; b=false; c=false; d=false; e=false;
        end
        function [a,b,c,d,e]=isOutputFixedSizeImpl(~)
            a=true; b=true; c=true; d=true; e=true;
        end
    end
end
