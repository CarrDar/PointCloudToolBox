function CP = GenCornetdeFrites( CP, nbpoints )
%GENCORNETDEFRITES Generate a point cloud from the shape of a "cornet de frites".
%
% Input :
%   CP : a handle on a PointCloud class (optional).
%   nbpoints : the number of desired points (optional), default 1000.
%
% Output :
%   Computes the true set of points from the generated shape 
%   (3xnbpoints matrix). Set CP.TrueP.
%  
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.3
%STATUS  : OK
%DATE    : 25 mai 2011, modified 27 may.

% Menu
% Width of the Cornet
W = 100;

%initialisation
if nargin == 0
    CP = PointCloud;
    nbpoints = 1000;
elseif nargin == 1
    if isa(CP,'PointCloud')
        nbpoints = 1000;
    elseif isreal(CP)
        nbpoints = CP;
        CP = PointCloud;
    else
        msg='Please provide a PointCloud or a real integer.';
        error('GenCornetdeFrites:WVar',msg);
    end
end
r = rand(nbpoints,2);
x = r(:,1)*W - W/2; 
y = r(:,2)*W - W/2; 
z = sqrt(x.^2+y.^2);
CP.TrueP=[x y z]';

end

