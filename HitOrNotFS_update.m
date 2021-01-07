function [fval,info,range] = HitOrNotFS_update(center,an,bn,rectF,borderF,ang,x0,y0,maxrange)
% find hitted point
% AnyFea: m by n, m is the number of features, n is the number of parameters.
% if nargin<7
%     AnyFea = []; 
% end
fval = [];
info = [];
F = center; % new feature including border.
nF = size(F,2); % num Fea anbn
nR = size(rectF,2); % num Rect
nB = size(borderF,2); % num Border
for idf = 1:nF
    [tmpf,tmpinfo] = intersectionFS(F(:,idf),an(:,idf),bn(:,idf),ang,x0,y0); % Hit or not with specific feature
    fval = [fval tmpf];
    info = [info; tmpinfo]; 
end
for idb = 1:nB
    [tmpf,tmpinfo] = intersectionFS_rect(borderF(:,idb),ang,x0,y0,true); % Hit or not with specific feature
    fval = [fval tmpf];
    info = [info; tmpinfo]; 
end
for idr = 1:nR
    [tmpfr,tmpinfor] = intersectionFS_rect(rectF(:,idr),ang,x0,y0); % Hit or not with specific feature
    fval = [fval tmpfr];
    info = [info; tmpinfor]; 
end

[a,id]=max(info); % judge how many intersections are.
if a>0
    if size(fval,2)~=1 % if more than one intersection, choose the nearer one.
        tmp_id = find(info>0); % find intersection of feature and border
        dist=vecnorm( fval-[x0;y0],2,1);
        [~,id_2]=min(dist); % find the nearest point
        if min(dist) > maxrange
            info = 0;
            fval = nan(3,1);
            range = 60;
            return
        end
        flagid = tmp_id(id_2);
        if flagid > nF && flagid <= nF + nB % Border
            flagid = 10 * (flagid - nF); 
        elseif flagid > nF + nB && flagid <= nF + nB + nR % Rectangle
            flagid = 100 * (flagid - nF - nB);
        end
        fval = [flagid; fval(:,id_2)];
        info = 1;
        range = dist(id_2);
    else
        dist=norm( fval-[x0;y0]);
        if dist > maxrange
            info = 0;
            fval = nan(3,1);
            range = 60;
            return
        end
        if id > nF && id <= nF + nB % Border
            id = 10 * (id - nF);
        elseif id > nF + nB && id <= nF + nB + nR
            id = 100 * (id - nF - nB);
        end
        fval = [id ; fval];
        info = 1;
        range = dist;
    end
else
    fval = nan(3,1);
    info = 0;
    range = 60;
end
end