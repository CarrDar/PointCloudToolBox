function PC = GenSphere( radius, nbpoints )
%GENSPHERE Generate a point cloud from the shape of a sphere centered at the origin.
%
% Input :
%   radius   : radius of the sphere. Default 1.
%   nbpoints : the number of desired points, 100 by default.
%
% Output :
%   A handle on a PointCloud class.
%
% See also GENHALFSPHERE
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 17 july 2011

%initialisation
if nargin < 2
    nbpoints = 100;
    if nargin < 1
        radius = 1;
    end
end
% Sanity checks
if radius <= 0 || ~isscalar(radius)
    warning('GenSphere:WR','Provide positive scalar for the radius %f, reset to 1',radius);
    radius = 1;
end
if nbpoints <= 0 || ~isscalar(nbpoints)
    warning('GenSphere:WN','Provide positive integer for the nb of points %i, reset to 100',nbpoints);
    nbpoints = 1;
end

% create sphere of radius r centered at origin.
Rn = rand(uint32(nbpoints),2);
theta = Rn(:,1)*2*pi;
u = Rn(:,2)*2-1;
% I want to obtain points such that any small area on the sphere is
% expected to contain the same number of points.
u2 = sqrt(1-u.^2);
x = u2.*cos(theta)*radius; 
y = u2.*sin(theta)*radius;
z = u*radius;
PC = PointCloud();
PC.TrueP=[x y z]';

end

