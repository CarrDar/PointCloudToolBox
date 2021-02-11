function PlotMultiPointClouds(PC, varargin)
%PLOTMULTIPOINTCLOUDS Display one or more clouds of points in a nice way. 
%
% Input : 
%   PC : a cell of PointCloud/MeshPointCloud objects.
%
% Additional parameters that can be set via their field name :
%
%   TruePos : 
%       vector of bools, false by default.
%       false  : PC(x).P will be plotted.
%       true   : PC(x).TrueP will be plotted.
%   PlotOpt    : plot options for each point cloud.
%
% Usage :
%   myopt = {
%       {'MarkerSize',2,'Marker','*','LineStyle','none','Color',[0.75 0 0.75]},...
%       {'MarkerSize',1,'Marker','.','LineStyle','none','Color',[0.75 0 0.75]},...
%       {'MarkerSize',4,'Marker','+','LineStyle','none','Color',[0 0.5 0]},...
%       {'MarkerSize',4,'Marker','x','LineStyle','none','Color',[0 0 1]}};
%   PlotMultiPointClouds({pc1 pc2 pc3 pc4},'PlotOpt',myopt, 'TruePos', [false false true true]);
%
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 5.3
%STATUS  : OK
%DATE    : 7 july 2011.

%% Validate input arguments.
ip = inputParser;
ip.addRequired( 'PC', @(x)iscell(x) );
ip.addParamValue( 'TruePos', -1, @(x)islogical(x) );
ip.addParamValue( 'PlotOpt', -1, @(x)iscell(x) );
ip.addParamValue( 'Title', '3D Point Clouds', @(x)ischar(x) );

ip.parse(PC,varargin{:});
arg = ip.Results;

%% Initialisation and sanity checks
% number of PointClouds
nbPC = length(PC);
if arg.TruePos == -1
    arg.TruePos = false(1,nbPC);
elseif size(arg.TruePos,2)~=nbPC
    error('Plot:WArg','PC and TruePos arguments have not the same length.');
end
if ~iscell(arg.PlotOpt)
    L = zeros(3,nbPC); U = ones(3,nbPC);
    r = random('unif',L,U);
    arg.PlotOpt = cell(1,nbPC);
    for ip = 1:nbPC
        if isa(PC{ip},'PointCloud')
            arg.PlotOpt{1,ip} = {'MarkerSize',1,'Marker','.','LineStyle','none','Color',r(:,ip)};
        elseif isa(PC{ip},'MeshPointCloud')
            arg.PlotOpt{1,ip} = {'FaceLighting','phong'};
        end
    end
elseif length(arg.PlotOpt)~=nbPC
    error('Plot:WArg','PC and PlotOpt arguments have not the same length.');    
end

%% Plot !
%Create figure with white background
figure1 = figure('Color',[1 1 1]);

% Choose a color map for the mesh
colormap(hot(256));
%colormap(jet(256));
% colormap(hsv(256));

% Create axes
axes1 = axes('Parent',figure1);
%view(axes1,[322.5 30]);
view(3);
hold(axes1,'all');

% Put some light for the surf.
light('Parent',axes1,'Style','local','Position',[104 -603 600]);

% Create title
title(arg.Title);

%Nice opt :
% 'b' blue point
% 'ro' small round
% 'b*' blue star
% 'm*' pink star
% For a nice surf, you may add
%colormap(hot(256));
%colormap(jet(256));
% colormap(hsv(256));
%camlight right;
%lighting phong;
%shading interp;

hold on;

for ip = 1:nbPC
    name = PC{ip}.Name;
    if arg.TruePos(ip), name = [name ' (true positions)']; end             %#ok<AGROW>
    opt = arg.PlotOpt{ip};
    if isa(PC{ip},'PointCloud')
        PC{ip}.plot3(arg.TruePos(ip),opt{:},'DisplayName',name);
    elseif isa(PC{ip},'MeshPointCloud')
        PC{ip}.surf(opt{:},'DisplayName',name);
    else
        warning('Plot:WArg','Unknown object %s, skipping.', class(PC{ip}) );
    end
end


% create legend
legend1 = legend(axes1,'show');
set(legend1,'Position',[0.12 0.91 0.2 0.07]);
hold off;

end


