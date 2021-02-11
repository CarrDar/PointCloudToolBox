function PC = GenWaves( varargin )
%GENWAVES Generate a point cloud with a wavy shape.
%
% Parameters that can be set via their field name :
%
%   PC : a handle on a PointCloud or MeshPointCloud class (optional).
%   W : width of the mesh in x,y. D = [20 20].
%   D : density of points. D = [5 5].
%   A : amplitude of the waves. D = 1.
%   L : wavelengths. D = [6 6].
%   For W,D,L arguments you can provide separate values for x and y. If
%   only one value is given, it will be applied to both x and y.
%
%   Peaks : add a sublying structure with peaks. The set value of Peaks is used to
%   set the relative amplitude of the peaks with respect to A.
%
%   Sphere : add a sublying structure with with the form of a half sphere. 
%   The set value of Sphere is used to set the relative amplitude of the sphere with respect to A.
%
% Output :
%   If given PC is a PointCloud : the true set of points from the generated shape, out in PC.TrueP.
%   If given PC is a MeshPointCloud : PC filled with the  set of points
%   from the generated shape:
%   If nothing given, a point cloud is returned.
%
% Usage :
%   PC = GenWaves('W',[100 20],'Peaks',10);
%   PC = PointCloud('Waves with peaks'); GenWaves(PC,'W',[100 20],'Peaks',10);
%   To get a MeshPointCloud, use the following trick :
%   MPC = GenWaves(MeshPointCloud,'W',[100 20],'Peaks',10);
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.4
%STATUS  : OK
%DATE    : 29 august 2011

%% Validate input arguments.
ip = inputParser;
ip.addOptional('PC', PointCloud, @(x)isa(x,'PointCloud')||isa(x,'MeshPointCloud'));
ip.addParamValue('W',[20 20],@(x)isnumeric(x) && length(x) <= 2 );
ip.addParamValue('D',[5 5],@(x)isnumeric(x) && length(x) <= 2 && x > 0 );
ip.addParamValue('A',1,@(x)isnumeric(x) && length(x) <= 1 && x ~= 0 );
ip.addParamValue('L',[6 6],@(x)isnumeric(x) && length(x) <= 2 && x ~= 0 );
ip.addParamValue('Peaks',0,@(x)isnumeric(x) && length(x) <= 1 && x >= 0 );
ip.addParamValue('Sphere',0,@(x)isnumeric(x) && length(x) <= 1 && x >= 0 );
ip.parse(varargin{:});
arg = ip.Results;
PC = arg.PC;

if length(arg.W) < 2, arg.W = [arg.W arg.W]; end
if length(arg.D) < 2, arg.D = [arg.D arg.D]; end
if length(arg.L) < 2, arg.L = [arg.L arg.L]; end

%% Initialise
d = [1. 1.]/arg.D;
[X,Y]=meshgrid(0:d(1):arg.W(1), 0:d(1):arg.W(2));

%% Do the job
Z = arg.A * sin(2*pi*X/arg.L(1)) .* sin(2*pi*Y/arg.L(2));

%% Add some peaks ?
if arg.Peaks
    
    % Increase it to have the peaks take a smaller proportion of the x,y
    % space.
    sp = 6;
    Dp = arg.D .* arg.W ./ (2*sp); dp = [1. 1.]./Dp;
    [Xp,Yp] = meshgrid(-sp:dp(1):sp, -sp:dp(2):sp);
    
    Zp =  3*(1-Xp).^2.*exp(-(Xp.^2) - (Yp+1).^2) ...
        - 10*(Xp/5 - Xp.^3 - Yp.^5).*exp(-Xp.^2-Yp.^2) ...
        - 1/3*exp(-(Xp+1).^2 - Yp.^2);
    
    % The max is always
    mx = max(max(Zp));
    %Normalise Z to the given amplitude.
    Z = Z + Zp .* arg.Peaks .* arg.A ./ mx;
    
end

%% Add a half sphere ?
if arg.Sphere
    
    % relative width of sphere
    sp = 4;
    Dp = arg.D .* arg.W ./ (2*sp); dp = [1. 1.]./Dp;
    [Xp,Yp] = meshgrid(-sp:dp(1):sp, -sp:dp(2):sp);
    
    Zp = zeros(size(Xp));
    % Index of points to modify.
    Id = Xp.^2 + Yp.^2 <= 1 ;
    % Add the sphere to those points.
    Zp(Id) = 1 - Xp(Id).^2 -  Yp(Id).^2;
    
    % The max is always
    mx = max(max(Zp));
    %Normalise Z to the given amplitude.
    Z = Z + Zp .* arg.Sphere .* arg.A ./ mx;     
    
end

if isa(PC,'PointCloud')
    s = size(Z); s = s(1) * s(2);
    PC.TrueP = [ X(1:s)' Y(1:s)' Z(1:s)' ];
else
    PC.xgrid = X; PC.ygrid = Y; PC.zgrid = Z;
end


end

