classdef uav_reference_generator < matlab.System
    % UAV_REFERENCE_GENERATOR Maps desired payload geometry to UAV references.
    % alpha_d(:,i) points from UAV i to payload attachment i.
    % Inputs omegad and omegadp are expressed in the desired payload frame.
    properties (Access = public)
        N = 2;
        Rattach = [0.5 -0.5; 0 0; 0 0];
        CableLengths = [0.8;0.8];
    end
    properties (Access = private)
        quat;
    end
    methods (Access = protected)
        function setupImpl(obj)
            obj.quat=Quaternions();
        end
        function [Xuav_d,Vuav_d,Auav_d,Ftension_d] = stepImpl(obj,xd,xdp,xdpp,qd,omegad,omegadp,Alpha_d,Alpha_dp,Alpha_dpp,Td)
            qd=qd(:)/max(norm(qd),1e-12);
            N=obj.N;
            Xuav_d=zeros(3,N); Vuav_d=zeros(3,N);
            Auav_d=zeros(3,N); Ftension_d=zeros(3,N);
            for i=1:N
                ri=obj.Rattach(:,i);
                ai=Alpha_d(:,i)/max(norm(Alpha_d(:,i)),1e-12);
                aid=Alpha_dp(:,i);
                aidd=Alpha_dpp(:,i);
                xai=xd(:)+obj.rotate_by_quat(qd,ri);
                vai=xdp(:)+obj.rotate_by_quat(qd,cross(omegad(:),ri));
                aai=xdpp(:)+obj.rotate_by_quat(qd, ...
                    cross(omegadp(:),ri)+cross(omegad(:),cross(omegad(:),ri)));
                Xuav_d(:,i)=xai-obj.CableLengths(i)*ai;
                Vuav_d(:,i)=vai-obj.CableLengths(i)*aid;
                Auav_d(:,i)=aai-obj.CableLengths(i)*aidd;
                Ftension_d(:,i)=Td(i)*ai;
            end
        end
        function [a,b,c,d]=getOutputSizeImpl(obj)
            a=[3 obj.N]; b=[3 obj.N]; c=[3 obj.N]; d=[3 obj.N];
        end
        function [a,b,c,d]=getOutputDataTypeImpl(~)
            a='double'; b='double'; c='double'; d='double';
        end
        function [a,b,c,d]=isOutputComplexImpl(~)
            a=false; b=false; c=false; d=false;
        end
        function [a,b,c,d]=isOutputFixedSizeImpl(~)
            a=true; b=true; c=true; d=true;
        end
    end
    methods (Access = private)
        function vI=rotate_by_quat(obj,q,v)
            vq=[0;v(:)];
            tmp=obj.quat.product(q,vq);
            tmp=obj.quat.product(tmp,obj.quat.conj(q));
            vI=tmp(2:4);
        end
    end
end
