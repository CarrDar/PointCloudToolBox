function Plot3DPointClouds(P1, varargin)
%PLOT3DPOINTCLOUDS Display one or two clouds of 3D points in a nice way. 
%
% Input : 
%   P1 : a PointCloud object.
%   P2 : a PointCloud object (optional).
%
% Additional parameters that can be set via their field name :
%   TruePos1(2) : 
%       {false} | true
%       options for plotting P1, P2 respectively :
%       false  : P1(2).P will be plotted.
%       true   : P1(2).TrueP will be plotted.
% Usage
%   Plot3DPointClouds(P1, 'TruePos1', true );
%   Plot3DPointClouds(P1, P2, 'TruePos1', true, 'TruePos2', true );
%   Plot3DPointClouds(P1, P1, 'TruePos1', true, 'TruePos2', false );
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 3.3
%STATUS  : OK
%DATE    : 26 mai 2011, modified 27 may.

%% Validate input arguments.
ip = inputParser;
ip.addRequired( 'P1', @(x)isa(x,'PointCloud') );
ip.addOptional( 'P2', [], @(x)isa(x,'PointCloud') );
ip.addParamValue( 'TruePos1', false, @(x)islogical(x) );
ip.addParamValue( 'TruePos2', false, @(x)islogical(x) );
ip.parse(P1,varargin{:});
arg = ip.Results;

%% Plot !
%Create figure with white background
figure1 = figure('Color',[1 1 1]);

% Create axes
axes1 = axes('Parent',figure1);
%view(axes1,[322.5 30]);
view(3);
hold(axes1,'all');

% Create title
title('3D Point Clouds');

% Create plot3
%Nice opt :
% 'b' blue point
% 'ro' small round
% 'b*' blue star
% 'm*' pink star
hold on;
name = arg.P1.Name;
if arg.TruePos1, name = [name ' (true positions)']; end
arg.P1.plot3(arg.TruePos1,'b*','DisplayName',name);

if ~isempty(arg.P2)
    name =  arg.P2.Name;
    if arg.TruePos2, name = [name ' (true positions)']; end
    arg.P2.plot3(arg.TruePos2,'ro','DisplayName',name);
end

% create legend
legend1 = legend(axes1,'show');
set(legend1,'Position',[0.12 0.91 0.2 0.07]);
hold off;

end


