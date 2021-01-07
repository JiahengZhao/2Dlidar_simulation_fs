function y = dichotomyRoot_s(x1,x2,e,an,bn,center,x0)
x=(x1+x2)/2;
f3=func_s(an,bn,x,center,x0);
f1=func_s(an,bn,x1,center,x0);
if f1*f3<0
    m=x-x1;
    if m>e
        x2=x;
        y=dichotomyRoot_s(x1,x2,e,an,bn,center,x0);
    else
        y=x;
    end
else
    m=x2-x;
    if m>e
        x1=x;
        y=dichotomyRoot_s(x1,x2,e,an,bn,center,x0);
    else
        y=x;
    end
end
end