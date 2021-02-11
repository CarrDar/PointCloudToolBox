function Plot3DPoints(P1, varargin)
%PLOT3DPOINTS Display the clouds of 3D points in a nice way. 
%
% Note : you better use Plot3DPointClouds if you can.
% Input : 
%   P1 : 3xn or nx3 matrix of n 3D points.
%   P2 : 3xn or nx3 matrix of n 3D points (optional).
%    
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.2
%STATUS  : OK
%DATE    : 22 juin 2011

%% Validate input arguments.
ip = inputParser;
ip.addRequired( 'P1', @(x) size(x,1) == 3 || size(x,2) == 3 );
ip.addOptional( 'P2', 0, @(x) size(x,1) == 3 || size(x,2) == 3 );
ip.parse(P1,varargin{:});
arg = ip.Results;
P2 = arg.P2;

%% Eventually transpose the point cloud.
if( size(P1,2) == 3), P1 = P1'; end
if( size(P2,2) == 3), P2 = P2'; end

%% Plot !
% Create figure with white background
figure1 = figure('Color',[1 1 1]);

% Create axes
axes1 = axes('Parent',figure1);
view(3);
hold(axes1,'all');

% Create title
title('3D Point Cloud');

% Create plot3
%Nice opt :
% 'b' blue point
% 'ro' small round
% 'b*' blue star
% 'm*' pink star
hold on;
pc = '3D Point Cloud';
plot3(P1(1,:),P1(2,:),P1(3,:),'b.','DisplayName',['First ' pc]);
if P2
  plot3(P2(1,:),P2(2,:),P2(3,:),'r*','DisplayName',['Second ' pc]);  
end

% create legend
legend1 = legend(axes1,'show');
set(legend1,'Position',[0.12 0.91 0.2 0.07]);
hold off;

