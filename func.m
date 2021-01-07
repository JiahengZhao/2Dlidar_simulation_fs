function func = func(an,bn,theta0,center,k,x0,y0)

r=0;
N=length(an)-1;
% syms theta1;
bn = [0;bn];
for n1=0:N
    r=r+an(n1+1)*cos(n1*theta0)+bn(n1+1)*sin(n1*theta0);
end

 func=center(2)+r*sin(theta0)-y0-k*(center(1)+r*cos(theta0)-x0);
% func=center(1)+r*cos(theta0)-(center(2)+r*sin(theta0))+3;


end
