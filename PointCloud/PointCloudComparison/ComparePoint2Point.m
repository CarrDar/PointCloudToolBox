function ComparePoint2Point(tpc,spc)
% COMPARISON Perform the differences of a source point cloud to a target
% point cloud with the Point-to-Point class of algorithms.
%
% To add :
%
%
%
% Ouput :
%   D : An array containing the Euclidian distance between Reference and Data point
%   cloud.
%
% Input :
%   tpc : the target, a (Mesh)PointCloud object.
%   spc : the source, a (Mesh)PointCloud object.
%   Note :
%       the given source will be aligned, ie the result of the alignment
%       procedure will be applied to the source.
%       nans in the positions are not taken into account and wont cause any problem.
%       Given MeshPointClouds will be turned into PointCloud objects.
%
% Additional parameters that can be set via their field name :
%
% Method
%       Gets its arguments as cells. In the first entry, you may pass
%       {P2P} | P2S
%       Defines which minimization should be performed.
%        - P2P is based on Point-to-Point method.
%       Surface normals for all points in the target are computed by
%       using a KDTree, which requires some preprocessing.
%       Think that this process is dependant on the number of nearest
%       neighbours used. Default is 4, which is fine for geometrical
%       (simulated) figures. Consider increasing this number for landscape
%       measurement with a high point density, to smooth a bit the terrain.
%       Search for t.ComputeNormals in this file, read the comments how to proceed.
% Advise :
% - Perform an initial comparison in Point-to-Point before starting
%   Point-to-Surface
% - Make a subsample of your clouds if they are big (>100k points).
%
%
%
% Disclaimer :
%    This function comes with no warranty whatsoever. The responsability is
%    upon the user to test thoroughly that it yields results consistent with expectations.
%    Please signal any bug encountered.
%
%AUTHOR  : Dario Carrea (at unil dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 19 feb 2013

%% Validate input arguments.
ip = inputParser;

ip.addRequired( 'tpc', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud') );
ip.addRequired( 'spc', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud')  );


%% Sanity check

% If MeshPointCloud are given, change them into PointCloud
if isa(tpc,'MeshPointCloud'), tpc = tpc.PointCloud; end
if isa(spc,'MeshPointCloud'), spc = spc.PointCloud; end
if isempty(tpc.TLSPos) && isempty(spc.TLSPos)
    error('Comparison:WTLS','Please provide a TLS position.');
end

%% Start comparison


global t s

% Detect the nearest neighbour for each point of initial PC in SHAPE

% PI the point line position of the nearest neighbor of one point in SHAPE
% to one point in initial PC
% DD the euclidian distance between these two points

[PI, DD] = nearestNeighbor(t.DT,(s.P)');
fin = length(DD);

s.UsefulVar=DD'; %temp

for g = 1:fin
    if EuclDist((s.TLSPos(1:3,1)),(s.P(1:3,g))) > EuclDist((t.TLSPos(1:3,1)),(t.P(1:3,PI(g))))
        s.UsefulVar(:,g) = s.UsefulVar(:,g)*(-1); % temp
    end
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%