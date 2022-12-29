function [fval, info] = intersectionFS(F,an,bn,ang,x0,y0)

center=F;
N=length(an)-1;
e = 1e-10;
% tangent

theta_final=[];    
nCut = 12;
stepA = 2*pi/nCut;
xL=-pi; xU=-pi+stepA;
if abs(ang-pi/2) <= 1e-5 || abs(ang+pi/2)<= 1e-5
    for ts = 1:nCut
        fL=func_s(an,bn,xL+(ts-1)*stepA,center,x0);
        fU=func_s(an,bn,xU+(ts-1)*stepA,center,x0);
        if fL*fU>0
            Nocross=0;
        else
            theta1=dichotomyRoot_s(xL+(ts-1)*stepA,xU+(ts-1)*stepA,e,an,bn,center,x0);
            theta_final=[theta_final;theta1];
        end
    end
  
    if isempty(theta_final)
        fval=[];
        info=0;
    else
        D = zeros(length(theta_final),1);
        for i = 1:N+1
            D = D + an(i).*cos((i-1).*theta_final) + bn(i).*sin((i-1).*theta_final);
        end
        XX=D.*cos(theta_final)+center(1);
        YY=D.*sin(theta_final)+center(2);

        dists = (YY-y0).^2 + (XX-x0).^2;
        [~,min_id] = min(dists);
        fval=[XX(min_id);YY(min_id)];

        info=1;
    end

else % general case
    k = tan(ang);
    for ts = 1:nCut
        fL=func(an,bn,xL+(ts-1)*stepA,center,k,x0,y0);
        fU=func(an,bn,xU+(ts-1)*stepA,center,k,x0,y0);
        if fL*fU>0
            Nocross=0;
        else
            theta1=dichotomyRoot(xL+(ts-1)*stepA,xU+(ts-1)*stepA,e,an,bn,center,k,x0,y0);
            theta_final=[theta_final;theta1];
        end
    end
    
    if isempty(theta_final)
        fval=[];
        info=0;
    else
        D = zeros(length(theta_final),1);
        bn = [0;bn];
        for i = 1:N+1
            D = D + an(i).*cos((i-1).*theta_final) + bn(i).*sin((i-1).*theta_final);
        end
        XX=D.*cos(theta_final)+center(1);
        YY=D.*sin(theta_final)+center(2);

        temp_angels = abs(atan2(YY-y0,XX-x0) - ang);
        [min_ang,~] = min(temp_angels);
        if min_ang < 1e-5
            info = 1;
        else
            info = 0;
            fval = [];
            return
        end
        
        dists = (YY-y0).^2 + (XX-x0).^2;
        [~,min_id] = min(dists);
        fval=[XX(min_id);YY(min_id)];

    end
end
end


