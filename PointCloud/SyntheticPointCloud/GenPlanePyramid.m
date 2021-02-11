function CP = GenPlanePyramid( CP )
%GENPLANEPYRAMID Generate a point cloud from the shape of a plane with a pyramid in the middle.
%
% Input :
%   A handle on a PointCloud class (optional).
% Output :
%   The true set of points from the generated shape, out in CP.TrueP.
%
% Usage :
%   PC = GenPlanePyramid;
%   PC = PointCloud('Pyramid'); GenPlanePyramid(PC);
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.1
%STATUS  : OK
%DATE    : 24 mai 2011

% if a PointCloud is not given, provide one !
if nargin < 1
    CP = PointCloud;
end

% Summits of the pyramid
py1 = [ 20 -20 0 ]';
py2 = [ -20 -20 0 ]';
py3 = [ -20 20 0 ]';
py4 = [ 20 20 0 ]';
pys = [ 0 0 30 ]'; 
pyn = 10; %density of the pyramidal mesh
TrueDataPoints = [ TriangularMesh([py1 py2 pys],pyn), ...
    TriangularMesh([py2 py3 pys],pyn), TriangularMesh([py3 py4 pys],pyn),...
    TriangularMesh([py4 py1 pys],pyn) ];

% Remove duplicate points in the pyramid
TrueDataPoints = RemoveDuplicate3DPoints( TrueDataPoints );

% Definition of the plane (centered at 0,0)
plx = 100; % 0.5*width of the plane
ply = 100; % 0.5*length of the plane
pldx = 5;  % point density in x
pldy = 5;  % point density in y
CP.TrueP = [ TrueDataPoints, PlaneMesh([plx,ply,pldx,pldy])];
%CP.TrueP = [ TrueDataPoints ];

end



