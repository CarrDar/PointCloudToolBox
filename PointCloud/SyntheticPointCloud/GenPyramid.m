function CP = GenPyramid( CP )
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
%AUTHOR  : Dario Carrea (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 2 aout 2013

% if a PointCloud is not given, provide one !
if nargin < 1
    CP = PointCloud;
end

% Summits of the pyramid
py1 = [50 -50 0 ]';
py2 = [ -50 -50 0 ]';
py3 = [ -50 50 0 ]';
py4 = [ 50 50 0 ]';
pys = [ 0 0 75 ]'; 
pyn = 1; %density of the pyramidal mesh
TrueDataPoints = [ TriangularMesh([py1 py2 pys],pyn), ...
    TriangularMesh([py2 py3 pys],pyn), TriangularMesh([py3 py4 pys],pyn),...
    TriangularMesh([py4 py1 pys],pyn) ];

% Remove duplicate points in the pyramid
TrueDataPoints = RemoveDuplicate3DPoints( TrueDataPoints );

CP.TrueP = [ TrueDataPoints ];

end