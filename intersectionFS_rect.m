function [fval, info] = intersectionFS_rect(F,ang,x0,y0,hasBorder)
% only exist rect Fea

if nargin <5
    hasBorder = false;
end

% 4 lines of one rect
x1 = F(1);
x2 = F(1) + F(3);
y1 = F(2);
y2 = F(2) + F(4);
if ~hasBorder
    if ang == pi/2 || ang== -pi/2
        if x0 >=x1 && x0<=x2
            if abs(y1-y0) < abs(y2-y0)
                fval = [x0; y1];
            else
                fval = [x0; y2];
            end
            info = 1;
        else
            fval = [];
            info = 0;
        end
    else % general case
        k = tan(ang);
        % cal 4 intersections
        y1_tmp = k*(x1 - x0) + y0;
        y2_tmp = k*(x2 - x0) + y0;
        x1_tmp = (y1 - y0)/k + x0;
        x2_tmp = (y2 - y0)/k + x0;
        tm1 = []; tm2 = []; tm3 = []; tm4 = [];
        if y1_tmp <= y2 && y1_tmp >= y1 % left edge
            tm1 = [x1;y1_tmp];
        end
        if y2_tmp <= y2 && y2_tmp >= y1 % right edge
            tm2 = [x2;y2_tmp];
        end
        if x1_tmp <= x2 && x1_tmp >= x1 % lower edge
            tm3 = [x1_tmp;y1];
        end
        if x2_tmp <= x2 && x2_tmp >= x1 % upper edge
            tm4 = [x2_tmp;y2];
        end
        tm = [tm1 tm2 tm3 tm4];
        if ~isempty(tm)
            dist = (tm(1,:)-x0).^2 + (tm(2,:)-y0).^2;
            [~,minid] = min(dist);
            angtmp = atan2(tm(2,minid)-y0,tm(1,minid)-x0);
            if abs(angtmp - ang) < 1e-5
                fval = tm(:,minid);
                info = 1;
            else
                info = 0;
                fval = [];
                return
            end
        else
            fval = [];
            info = 0;
        end % end if ~isempty
    end % end if angle = pi/2
else % has border
    if ang == pi/2 || ang== -pi/2
        if x0 >=x1 && x0<=x2
            if ang == pi/2
                 fval = [x0; y2];
            else
                fval = [x0; y1];
            end
            info = 1;
        else
            fval = [];
            info = 0;
        end
    else % general case
        k = tan(ang);
        % cal 4 intersections
        y1_tmp = k*(x1 - x0) + y0; 
        y2_tmp = k*(x2 - x0) + y0;
        x1_tmp = (y1 - y0)/k + x0;
        x2_tmp = (y2 - y0)/k + x0;
        tm1 = []; tm2 = []; tm3 = []; tm4 = [];
        if y1_tmp <= y2 && y1_tmp >= y1 % left edge
            tm1 = [x1;y1_tmp];
        end
        if y2_tmp <= y2 && y2_tmp >= y1 % right edge
            tm2 = [x2;y2_tmp];
        end
        if x1_tmp <= x2 && x1_tmp >= x1 % lower edge
            tm3 = [x1_tmp;y1];
        end
        if x2_tmp <= x2 && x2_tmp >= x1 % bottom edge
            tm4 = [x2_tmp;y2];
        end
        tm = [tm1 tm2 tm3 tm4];
        if ~isempty(tm) % 2 intersections at most
            angtmp = atan2(tm(2,:)-y0,tm(1,:)-x0);
            checkID = abs(angtmp - ang) < 1e-5;
            fval = tm(:,checkID);
            info = 1;
        else
            fval = [];
            info = 0;
        end % end if ~isempty
    end % end if angle = pi/2
end
end


