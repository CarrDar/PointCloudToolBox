function PC = GenCube( varargin )
%GENREGULARCUBE Generate a point cloud from the shape of a parallelepiped with regular spacing.
%
% Parameters that can be set via their field name :
%
%   CP : a handle on a PointCloud class (optional).
%
%   Arg : vector with at position
%       1) 0.5*width of the cube
%       2) 0.5*length of the cube
%       3) 0.5*length of the cube
%       4) point density in x
%       5) point density in y
%       6) point density in z
%       Default : [100 100 100 5 5 5].
%
% Output :
%   The true set of points from the generated shape, out in CP.TrueP.
%
% Usage :
%   PC = GenCube([5 5 5],[ 2  2  2]);
%
%AUTHOR  : Dario Carrea (at unil dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 30 juillet 2013
%%
% Default input arguments
inArgs = { ...
  [100 100 100] , ... % Default edge sizes (x,y and z)
  [0 0  0] , ... % Default coordinates of the origin point of the cube
  };
% Replace default input arguments by input values
inArgs(1:nargin) = varargin;

% Create all variables
[edges,origin] = deal(inArgs{:});

length= edges(1,1);
width= edges(1,2);
high= edges(1,3);

X1 = origin(1,1)-(length/2);Y1 = origin(1,2)-(width/2);Z1 = origin(1,3)-(high/2);
X2 = origin(1,1)+(length/2);Y2 = origin(1,2)+(width/2);Z2 = origin(1,3)+(high/2);

P1=[X1 Y1 Z1];  P2=[X1 Y2 Z1];  P3=[X2 Y1 Z1]; P4=[X2 Y2 Z1];
P5=[X1 Y1 Z2];  P6=[X1 Y2 Z2];  P7=[X2 Y1 Z2]; P8=[X2 Y2 Z2];

XYZ= vertcat(P1,P2,P3,P4,P5,P6,P7,P8);

PC = PointCloud();
PC.TrueP=XYZ;
end