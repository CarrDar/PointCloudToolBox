function CP = GenValley( CP, nbpoints )
%GENVALLEY Generate a point cloud from two planes side by side in a V-shape.
%
% Input :
%   CP : a handle on a PointCloud class (optional).
%   nbpoints : the number of desired points (optional), default 1000.
%
% Output :
%   Computes the true set of points from the generated shape 
%   (3xnbpoints matrix). Set CP.TrueP.
%  
%AUTHOR  : Dario Carrea (at unil dot ch)
%VERSION : 3.0
%STATUS  : OK
%DATE    : 17 september 2011


%%
% Menu
% Width of the valley
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
        error('GenValley:WVar',msg);
    end
end
r = rand(nbpoints,2);
x = r(:,1)*W - W/2; 
y = r(:,2)*W - W/2; 
z = abs(x-(r(:,2)));
CP.TrueP =[x y z]';

end