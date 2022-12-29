%--------------------------Gaussian Geometry SLAM---------------------------------%
% Author: Jiaheng Zhao, Tiancheng Li
% GNRT generates laser scan in a given map which the features are represented by Fourier series
% Update: rectangle feature is updated.
%-----------------------------------------------------------------------------------------------%
clc;
close all
clear all
%% pre config
isdisplay = true; % animation switch;
dataname = 'test_review_6_wall'; % file name
dataFolder = fullfile(pwd,filesep,'Data'); % data folder
if ~exist(dataFolder, 'dir')
    mkdir(dataFolder)
    addpath(dataFolder)
end
trajFile = fullfile(dataFolder,filesep,[dataname,'_traj.mat']); % trajectory directory
finaldata =fullfile(dataFolder,filesep,[dataname,'.mat']); % final saved data directory
featuredata = fullfile(dataFolder,filesep,[dataname,'_fea.mat']); % saved feature directory
olevel = 2*1e-2; % odometry noise level
olevelang =1*1e-4;
obslevel = 2*1e-2; % observation noise
stepsize = 0.25;%0.05; % generate trajectory step size.
numBorder = 1; % Border number
numRect = 3; % rectangle features
numGenFea = 3; %  circular features
N=5;
%%
% Store data
data.scan = [];
data.feature =[];
data.odom = [];
data.groundtruth = [];
% Initialize map parameters
num_ang = 662;
bearing =linspace( 1.9198600, -1.9198600 ,662);
max_range = 30;

% draw and plot features
f1 = figure;
plot(0,0,'k^');
hold on;axis equal;axis([-20,20,-20,20]);
an = zeros(N+1,numGenFea);
bn = zeros(N,numGenFea);
center = zeros(2,numGenFea);
fea = zeros(2*N+3,numGenFea);
rectF = zeros(4, numRect);
borderF = zeros(4,numBorder);
if ~isfile(featuredata)
    for s3 = 1:numBorder % Draw rectangle border
        title(['Input Border ', num2str(s3), ' in ', num2str(numBorder)]);
        tmp = getrect;
        borderF(:,s3) = tmp';
         plot ([tmp(1) tmp(1)+tmp(3) tmp(1) tmp(1) tmp(1)+tmp(3) tmp(1)+tmp(3)  tmp(1)+tmp(3)  tmp(1)]', ...
                    [tmp(2) tmp(2) tmp(2) tmp(2)+tmp(4) tmp(2)+tmp(4) tmp(2)  tmp(2)+tmp(4)  tmp(2)+tmp(4)]','b-', 'LineWidth', 1);
        plot ([tmp(1) tmp(1)+tmp(3) tmp(1) tmp(1) tmp(1)+tmp(3) tmp(1)+tmp(3)  tmp(1)+tmp(3)  tmp(1)]', ...
                    [tmp(2) tmp(2) tmp(2) tmp(2)+tmp(4) tmp(2)+tmp(4) tmp(2)  tmp(2)+tmp(4)  tmp(2)+tmp(4)]','r*', 'LineWidth', 1);
        pause
    end
    for s2 = 1:numRect % Draw rectangular features
        title(['Input Rect Fea: ', num2str(s2), ' in ', num2str(numRect)]);
        tmp = getrect;
        rectF(:,s2) = tmp';
         plot ([tmp(1) tmp(1)+tmp(3) tmp(1) tmp(1) tmp(1)+tmp(3) tmp(1)+tmp(3)  tmp(1)+tmp(3)  tmp(1)]', ...
                    [tmp(2) tmp(2) tmp(2) tmp(2)+tmp(4) tmp(2)+tmp(4) tmp(2)  tmp(2)+tmp(4)  tmp(2)+tmp(4)]','b-', 'LineWidth', 1);
        plot ([tmp(1) tmp(1)+tmp(3) tmp(1) tmp(1) tmp(1)+tmp(3) tmp(1)+tmp(3)  tmp(1)+tmp(3)  tmp(1)]', ...
                    [tmp(2) tmp(2) tmp(2) tmp(2)+tmp(4) tmp(2)+tmp(4) tmp(2)  tmp(2)+tmp(4)  tmp(2)+tmp(4)]','r*', 'LineWidth', 1);
        pause
    end
    for s1 = 1:numGenFea % Draw irregular features
        title(['Input General Fea: ', num2str(s1), ' in ', num2str(numGenFea)]);
        [an1,bn1,center1]=getfeature(N);
        an(:,s1) = an1;
        bn(:,s1) = bn1;
        center(:,s1) = center1;
        pause
    end
    fea = [center;an;bn];
else % if feauters have been created, loading directly.
    load(featuredata);
    center = fea(1:2,:);
    an = fea(3:N+3,:);
    bn = fea(N+4:end,:);
    p = plotFS(center,fea(3:end,:),'r');
    hold on;
    for s3 = 1:numBorder
        tmp = borderF(:,s3);
        plot ([tmp(1) tmp(1)+tmp(3) tmp(1) tmp(1) tmp(1)+tmp(3) tmp(1)+tmp(3)  tmp(1)+tmp(3)  tmp(1)]', ...
            [tmp(2) tmp(2) tmp(2) tmp(2)+tmp(4) tmp(2)+tmp(4) tmp(2)  tmp(2)+tmp(4)  tmp(2)+tmp(4)]','r-', 'LineWidth', 2);
    end
     for s2 = 1:size(rectF,2)
        tmp = rectF(:,s2);
         plot ([tmp(1) tmp(1)+tmp(3) tmp(1) tmp(1) tmp(1)+tmp(3) tmp(1)+tmp(3)  tmp(1)+tmp(3)  tmp(1)]', ...
                    [tmp(2) tmp(2) tmp(2) tmp(2)+tmp(4) tmp(2)+tmp(4) tmp(2)  tmp(2)+tmp(4)  tmp(2)+tmp(4)]','r-', 'LineWidth', 2);
      end
end
data.feature = fea;
data.rectF = rectF;
data.borderF = borderF;
data.fsN = N;

data.borderNum = size(borderF,2);
%%
color='b';
% Generate trajectory from mouse selecting
if ~isfile(trajFile)
    button = 1;
    clicID = 1;
    Truetraj = zeros(5000,2);
    odom = zeros(5000,3);
    title('Input Trajectory...');
    pause
    while button == 1
        [Truetraj(clicID,1),Truetraj(clicID,2),button] = ginput(1);
        %----------------limit noise into +-3sigma----------------------%
        noise_dxdy = randn(1,2)*olevel;
        noise_dphi = randn(1,1)*olevelang;
        noise_dxdy(noise_dxdy > 3*olevel) =olevel;
        noise_dxdy(noise_dxdy < -3*olevel) = -olevel;
        noise_dphi(noise_dphi > 3*olevelang) =olevelang;
        noise_dphi(noise_dphi < -3*olevelang) = -olevelang;
        %-------------------------End limit--------------------------%
        noise = [ noise_dxdy noise_dphi];
        if clicID == 1
            % set initial pose's angle as 0;
            Truetraj(clicID,3) = atan2(Truetraj(clicID,2),Truetraj(clicID,1));
            odom(clicID,:) = Truetraj(clicID,:)+noise;
        else
            Truetraj(clicID,3) =  atan2(Truetraj(clicID,2)-Truetraj(clicID-1,2),Truetraj(clicID,1)-Truetraj(clicID-1,1)); % pose angle
            R = theta2R(Truetraj(clicID-1,3));
            odom(clicID,3) = Truetraj(clicID,3) - odom(clicID-1,3) + noise(3);
            R_noise = theta2R(odom(clicID-1,3));
            odom(clicID,1:2) = (Truetraj(clicID,1:2)+noise(1:2)-odom(clicID-1,1:2))*R_noise;
        end
        plot(Truetraj(clicID,1),Truetraj(clicID,2),'bo'); % Plot real trajectory
        clicID=clicID+1;
    end
    Truetraj(clicID:end,:) = [];
    odom(clicID:end,:) = [];
    plot([0; Truetraj(:,1)],[0; Truetraj(:,2)],'k--');
    pause
    save(trajFile,'Truetraj'); % n by 3
    save(featuredata,'fea', 'rectF','borderF'); % n by 3
    % Existed trajectory
else % If trajectory has been stored
    load(trajFile)
    odom = zeros(length(Truetraj),3);
    for i = 1: size(Truetraj,1)
        noise_dxdy = randn(1,2)*olevel; % limit in 3*sigma
        noise_dphi = randn(1,1)*olevelang;
        noise_dxdy(noise_dxdy > 3*olevel) =olevel;
        noise_dxdy(noise_dxdy < -3*olevel) = -olevel;
        noise_dphi(noise_dphi > 3*olevelang) =olevelang;
        noise_dphi(noise_dphi < -3*olevelang) = -olevelang;
        %-------------------limit noise into +-3sigma
        noise = [ noise_dxdy noise_dphi];
        if i ==1
            odom(i,:) = Truetraj(i,:)+noise;
        else
            odom(i,3) = Truetraj(i,3) - odom(i-1,3) + noise(3);
            R_noise = theta2R(odom(i-1,3));
            odom(i,1:2) = (Truetraj(i,1:2)+noise(1:2)-odom(i-1,1:2))*R_noise;
        end
        if i ~= 1
            odom(i,:) = odom(i,:)+noise;
        end
    end
end
Truetraj=Truetraj'; % 3 by n
% for path planing
path = [0 0; Truetraj(1:2,:)'];

%% Scan
% This part is using spline to generate a smoothy trajectory
spx = path(:,1);
spy = path(:,2);
spt =0:length(spx)-1;
slope0=0;
slopeF=0;
t = 0:stepsize:length(spx)-1; % 1 for multi step
xq = spline(spt,[slope0; spx; slopeF],t);
yq = spline(spt,[slope0; spy; slopeF],t);

conspose = [xq(2:end);yq(2:end)]'; % negelet initial position [0,0] format [x,y]
conspose = [conspose zeros(size(conspose,1),1)];
consodom = zeros(size(conspose,1),3);
nstep = size(conspose,1);
for i = 1: nstep % Generate trajectory and odometry step by step
    %--------------------Limit noise in 3*sigma------------------------%
    noise_dxdy = randn(1,2)*olevel;
    noise_dphi = randn(1,1)*olevelang;
    noise_dxdy(noise_dxdy > 3*olevel) =olevel;
    noise_dxdy(noise_dxdy < -3*olevel) = -olevel;
    noise_dphi(noise_dphi > 3*olevelang) =olevelang;
    noise_dphi(noise_dphi < -3*olevelang) = -olevelang;
    %-------------------------------End Limit--------------------------%
    noise = [ noise_dxdy noise_dphi];
    if i ==1
        conspose(i,3) = atan2(conspose(i,2),conspose(i,1));
        consodom(i,:) = conspose(i,:);
    else
        conspose(i,3) =  atan2(conspose(i,2)-conspose(i-1,2),conspose(i,1)-conspose(i-1,1)); % pose angle
        R = theta2R(conspose(i-1,3));
        consodom(i,1:2) = (conspose(i,1:2)-conspose(i-1,1:2))*R;
        consodom(i,3) = conspose(i,3) - conspose(i-1,3);
    end
    consodom(i,:) = consodom(i,:)+noise;
end

conspose=conspose';
data.odom = consodom;
data.groundtruth = conspose;
range = max_range*ones(nstep,num_ang); % allocate range
scan=cell(1,nstep);

for step = 1 : nstep  % Generate laser data step by step
    
    x0 = conspose(1,step);
    y0 = conspose(2,step);
    th0= conspose(3,step);   % robot position
    R = theta2R(th0);
    
    arrowLength=3;
    sb_dx=arrowLength*cos(th0);
    sb_dy=arrowLength*sin(th0);
    % Laser beam line functions l: y-y0=k(x-x0)
    fval = zeros(3,num_ang);
    info = zeros(1,num_ang);
    bx = zeros(1,num_ang); by = bx;
    for i=1:num_ang % Generate beam and hit point
        ang =  wrapToPi(bearing(i)+th0); % angle of beam
        [bx(i),by(i)]=beamFun(x0,y0,ang,max_range); % beam in Global coordinate
        [fval(:,i), info(i), range(step,i)] = HitOrNotFS_update(center,an,bn,rectF,borderF,ang,x0,y0,max_range);
    end
    
    %--------------------Limit Noise Level-----------------------------%
    noise_scan = obslevel*randn(2,num_ang);
    big=noise_scan > 3*obslevel;
    small = noise_scan < -3*obslevel;
    noise_scan(1,big(1,:)) = obslevel;
    noise_scan(2,big(2,:)) = obslevel;
    noise_scan(1,small(1,:)) = -obslevel;
    noise_scan(2,small(2,:)) = -obslevel;
    %---------------------End limit noise-----------------------------%
    
    fval(2:3,:) = fval(2:3,:) + noise_scan;% add noise to scan.
    range(step,:) = range(step,:) + noise_scan(1,:);% add noise to range
    
    %-----------------------------Plot--------------------------------%
    if isdisplay
        hold on;
        disp(['Step: ',num2str(step),'.']);
        if step ~= 1
            delete(h2_1);
            delete(h2_2);
        end
        tmp1x=[repmat(x0, 1,length(bx(~info)));bx(~info)];
        tmp1y=[repmat(y0, 1,length(by(~info)));by(~info)];
        tmp2x=[repmat(x0, 1,length(fval(2, logical(info))));fval(2, logical(info))];
        tmp2y=[repmat(y0, 1,length(fval(3, logical(info)))); fval(3, logical(info))];
        if ~isempty(tmp1x)
            h3_1 = line(tmp1x, tmp1y,'color','g'); % plot free beam
        else
            h3_1 = [];
        end
        h3_2 = plot(fval(2, logical(info)),fval(3, logical(info)),'b.'); % plot scan point
        h3_3 = line(tmp2x,tmp2y,'color','g'); % plot hitted beam
        h2_1 = plot(x0,y0,'o','color',color,'LineWidth',2); % plot robt
        h2_2 = plot([x0;x0+sb_dx],[y0;y0+sb_dy],color,'LineWidth',2);
        %     h2_2 = arrow([x0,y0],[x0,y0]+[sb_dx, sb_dy],'EdgeColor',color,'FaceColor',color,'Length',10,'TipAngle',10); % Draw robot pose
        axis equal;
        %     pause(0.01);
        drawnow
        if step ~= nstep
            delete(h3_1);
            delete(h3_2);
            delete(h3_3);
        end
        hold off;
    end
    %-------------------------End  Plot--------------------------------%
    % Transform scan to local frame and store.
    fval(2:3, logical(info)) = R'*(fval(2:3, logical(info))-[x0;y0]); % change to local frame;  +noise_scan(:,logical(info)))
    points(2*step-1:2*step,:) = fval(2:3,:);
    scan{step}=fval;
end
data.scan = scan;
data.range = range;
save(finaldata,'-struct','data');
