function func_s = func_s(an,bn,theta0,center,x0)

r=0;
N=length(an)-1;
% syms theta1;
bn = [0;bn];
for n1=0:N
    r=r+an(n1+1)*cos(n1*theta0)+bn(n1+1)*sin(n1*theta0);
end

func_s=center(1)+r*cos(theta0)-x0;
end
