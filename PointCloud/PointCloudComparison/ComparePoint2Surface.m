function ComparePoint2Surface(tpc,spc)
% COMPARISON Perform the differences of a source point cloud to a target
% point cloud with the Point-to-Point or Point-to-surface class of algorithms.
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
% Point to plane minimisation. In brief, the idea is to bring the data points 
% close to the planes in which the target points reside. 
% Mathematically this can be done by minimising the dot products of the vectors 
% p_iq_i and normals n_i.
% The rotation matrix is linearised in order to have an analytical solution
% for the problem.
%
% Advise :

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
ip.addRequired( 'spc', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud') );


%% Sanity check 

% If MeshPointCloud are given, change them into PointCloud
if isa(tpc,'MeshPointCloud'), tpc = tpc.PointCloud; end
if isa(spc,'MeshPointCloud'), spc = spc.PointCloud; end
if isempty(tpc.TLSPos) && isempty(spc.TLSPos)
    error('Comparison:WTLS','Please provide a TLS position.');
end

%% Start comparison

global t s

t=tpc;
s=spc;

if isempty(t.DT)
    t.ComputeDelaunayTriangulation;
end

%If point to plane minimisation, normals are needed
if isempty(t.Normals)
    t.ComputeOptimalNormals;
    if ~isempty(t.TLSPos)
        t.NormalsOutTopo;
    end
end

d = distancePointPlane();

s.UsefulVar(2,:) =d;
end

function d = distancePointPlane()
%DISTANCEPOINTPLANE Signed distance betwen 3D point and plane
%
%
global t s
% normalized plane normal
[PI, ~] = nearestNeighbor(t.DT,(s.P)');

n = t.Normals(1:3,PI);


% Use of Hessian form, ie : N.p = d
% I this case, d can be found as : -N.p0, when N is normalized
D = -(sum(bsxfun(@times, n, bsxfun(@minus, t.P(:,PI), s.P))));

d=(D)./norm(n);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

