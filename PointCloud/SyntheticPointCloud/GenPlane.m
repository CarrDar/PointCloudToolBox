function CP = GenPlane( varargin )
%GENPLANE Generate a point cloud from the shape of a plane.
%
% Parameters that can be set via their field name :
%
%   CP : a handle on a PointCloud class (optional).
%
%   Arg : vector with at position
%       1) 0.5*width of the plane
%       2) 0.5*length of the plane
%       3) point density in x
%       4) point density in y
%       Default : [100 100 5 5].
%
% Output :
%   The true set of points from the generated shape, out in CP.TrueP.
%
% Usage :
%   PC = GenPlane('Arg',[100 100 1 1]);
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.1
%STATUS  : OK
%DATE    : 1 juillet 2011

%% Validate input arguments.
ip = inputParser;
ip.addOptional('CP', PointCloud, @(x)isa(x,'PointCloud'));
% Default plane (centered at 0,0)
plx = 100; % 0.5*width of the plane
ply = 100; % 0.5*length of the plane
pldx = 5;  % point density in x
pldy = 5;  % point density in y
ip.addParamValue('Arg', [plx,ply,pldx,pldy], @(x)isreal(x))
ip.parse(varargin{:});
arg = ip.Results;
CP = arg.CP;

CP.TrueP = PlaneMesh(arg.Arg);

end

