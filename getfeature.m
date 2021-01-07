function [an,bn,center]=getfeature(N,numPT)

if nargin < 2
    numPT = 15;
end
button=1;
X=[]; Y=[];
while button == 1
    [x,y] = ginput(1);
    Y=[Y,y];
    X=[X,x];
    plot (x, y,'r*', 'LineWidth', 1);
    while length(X)==numPT
        button=0;
        break;
    end
end

M = length(X); % number of points
noise = 0* randn(2,M) * 1e-1;
pt = [X;Y] + noise;
[V, center,~] = fitWithFS( N, pt(1,:), pt(2,:),[], 0);
pt_noCenter = pt-center;
an = V(1:N+1,1);
bn = V(N+2:end,1);
bn = [0;bn];

dtheta = zeros(1,361);
theta = 0:pi/180:2*pi;
for i = 1:N+1
    dtheta = dtheta + an(i).*cos((i-1).*theta) + bn(i).*sin((i-1).*theta);
end

% to point
newpt = [dtheta.*cos(theta); dtheta.*sin(theta)];
plot(newpt(1,:)+center(1),newpt(2,:)+center(2),'b-');
hold on;
axis equal;

bn(1) = [];
end
