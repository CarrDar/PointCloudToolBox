function [ OutputPoints ] = TriangularMesh( Points, n, par )
%TRIANGULARMESH Decompose a given triangle into smaller triangles.
%   Possibility to create a parallelogram grid.
%
%Input : 
%   Points : 3 summits of a triangle, where the ith summit is Points(:,i)
%   n      : the segments of the triangle will be reduced in n segments.
%   par    : set to true if you wish a parallelogram grid. False by default.
%    
%Output :
%   OutputPoints : the new set of 3D points defining the summits.
%   
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.0
%STATUS  : OK
%DATE    : 24 mai 2011

% sanity check
if( nargin <3 )
    par = false;
end
if( n<2 )
    OutputPoints = Points;
    return
end


% first point will serve as reference.
% define two directions x and y with the two other points
% and compute increment along x and y
dirnx = (Points(:,2)-Points(:,1))./n;
dirny = (Points(:,3)-Points(:,1))./n;

% from the first point, loop in directions x and y and create new points.
OutputPoints = zeros(3, sum(1:(n+1)));
ip=0; %global increment for position in OutputPoints
ix=0;
iy=0;
ixmax=n;

while( iy<=n )
   while( ix<=ixmax )
      ip = ip+1; 
      OutputPoints(:,ip) = Points(:,1) + dirnx.*ix + dirny.*iy;
      ix = ix+1;
   end
   iy = iy+1;
   ix = 0;
   %comment this to have a parallelogram mesh.
   if not(par), ixmax = ixmax-1; end
end


end

