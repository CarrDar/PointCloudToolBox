%%%%%%%%%%%%%%%%%%%%%%%% VOLUME ESTIMATION WITH ALPHA-SHAPE HULL %%%%%%%%%%%%%%%%%%%%%%
% Function: [BlocksVolume] = RockfallVolume(Rockfall_events)
%
% -------------------------------------------------------------------------
% Aim:
% Alphavol method based
% -------------------------------------------------------------------------
% Input:
% Rockfalls:
% R: radius of reserch alpha-shape
% fig: if you want to plot the histograms or not%
%
%
% -------------------------------------------------------------------------
% Output:
% BlocksVolume: Liste of volumes of différent blocks
% SS : List of simplices structure for each rockfall events
%
%
%
%
% -------------------------------------------------------------------------
% Example of use:
%
%
%
%
% -------------------------------------------------------------------------
% References:
%   [1] W.Shen, Building boundary extraction based on lidar point cloud
%       data,Int. Arch. of Photogram., Rem. Sens. and Spac. Inf. Sci., Vol.
%       37, Beijing, 2008.
%   [2] L. Lin, Conformal Alpha Shape-based Multi-scale Curvature Estimation
%       from Point Clouds, Journal of Computers, Vol. 7, N# 6, June 2012.
% -------------------------------------------------------------------------
%
%
% updated date: February 2015
function [BlocksVolume, SS] = RockfallVolume(Rockfall_events,R,fig)

% Sanity check of the radius
if ~isempty(R) && R <= 0
    error('RockfallVolume:searchRadius','Search radius R must be positive!');
end

if nargin < 2 || isempty(R)
    R = inf;
end

if nargin < 3
    fig = 0;
end

[~,n]=size(Rockfall_events);

BlocksVolume=zeros(n,1);
if isempty(R)
    R = inf;
end

hold on
for i=1:length(Rockfall_events)
    [V,S] = alphavol((Rockfall_events(i).P)',R,fig);
    BlocksVolume(i,1) = V;
    SS(i,1)=S;
end
hold off

%savefile= 'Volume of blocks';
%save(savefile,'Temp/BlocksVolume','-ascii');
end
