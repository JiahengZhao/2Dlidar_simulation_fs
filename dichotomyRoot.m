function y = dichotomyRoot(x1,x2,e,an,bn,center,k,x0,y0)
x=(x1+x2)/2;
f3=func(an,bn,x,center,k,x0,y0);
f1=func(an,bn,x1,center,k,x0,y0);
if f1*f3<0
    m=x-x1;
    if m>e
        x2=x;
        y=dichotomyRoot(x1,x2,e,an,bn,center,k,x0,y0);
    else
        y=x;
    end
else
    m=x2-x;
    if m>e
        x1=x;
        y=dichotomyRoot(x1,x2,e,an,bn,center,k,x0,y0);
    else
        y=x;
    end
end
end