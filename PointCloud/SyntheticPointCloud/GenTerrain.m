function PC = GenTerrain( varargin )
%GENTERRAIN Generate a point cloud similar to a terrain.
%
% Parameters that can be set via their field name :
%
%   PC : a handle on a PointCloud or MeshPointCloud class (optional).
%   Size : a positive real number. Somehow sets the size and the complexity
%   of the shape. Ideal between 15 and 50.
%
% Output :
%   If given PC is a PointCloud : the true set of points from the generated shape, out in PC.TrueP.
%   If given PC is a MeshPointCloud : PC filled with the  set of points
%   from the generated shape:
%   If nothing given, a point cloud is returned.
%
% Advise :
%   1) If you just wish to generate a MeshPointCloud, call
%   mpc = GenTerrain(MeshPointCloud,'Size',25);
%
%   2) You can replace membrane with peaks in the code to generate another ground
%   shape.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 26 august 2011

%% Validate input arguments.
ip = inputParser;
ip.addOptional('PC', PointCloud, @(x)isa(x,'PointCloud')||isa(x,'MeshPointCloud'));
ip.addOptional('Size',25,@(x)isnumeric(x) && x > 0 );
ip.parse(varargin{:});
arg = ip.Results;
PC = arg.PC;

% Initialise
m = arg.Size; 
[X,Y]=meshgrid(0:2*m);

% First term is the Matlab logo. Try using peaks.
% Second introduces some slope.
% Third some longitudinal sinusoidal waves.
% Fouthe some diagonal sinusoidal waves.
% Fifth : hard to describe, but had some chaos.

Z = 2 - 3*membrane(1,m) - 0.09*(X+Y) - 0.3*sin(pi*X/15).*sin(pi*Y/8) ...
    - 0.25*sin(pi*(X+Y+5)/5) - 0.25*sin(pi*(X.*Y)/80) + 4;

if isa(PC,'PointCloud')
    s = size(Z); s = s(1) * s(2);
    PC.TrueP = [ X(1:s)' Y(1:s)' Z(1:s)' ];
else
    PC.xgrid = X; PC.ygrid = Y; PC.zgrid = Z;
end


end

