function PC = GenParallelepiped(width,length,high,pdensity)
%GENPARALLELEPIPED Generate a point cloud from the shape of a parallelepiped.
%
% Input :
%   PC : a handle on a PointCloud class (optional).
%   width of the parallelepiped
%   {100} | scalar in [1,inf]
%   length of the parallelepiped
%   {100} | scalar in [1,inf]
%   high of the parallelepiped
%   {100} | scalar in [1,inf] 
%   pdty of the parallelepiped
%   {100} | scalar in [1,Inf]
% Output :
%   The true set of points from the generated shape, out in CP.TrueP.
%
% Usage :
%   PC = GenParallelepiped;
%   PC = PointCloud('Parallelepiped'); GenParallelepiped(PC);
%                                                                      
%AUTHOR  : Dario Carrea (at unil dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 12 mars 2013

%% Validate input arguments.

% Default cube (centered at 1,1,1)
if nargin < 4
    pdty = 1000;
else
    pdty = pdensity;
end
if nargin < 3
    high = 100;
    if nargin < 2
        length = 100;
        if nargin < 1
            width=100;
        end
    end
end
X1 = randi(width,1,pdty); Y1=randi(length,1,pdty); Z1=randi(high,1,pdty);

Face1(1,:) = ones(1,pdty); Face1(2,:) = Y1; Face1(3,:) = Z1;
Face2(1,:) = X1; Face2(2,:) = ones(1,pdty); Face2(3,:) = Z1;
Face3(1,:) = X1; Face3(2,:) = Y1; Face3(3,:) = ones(1,pdty);

Face6(1,:) = X1; Face6(2,:) = Y1; Face6(3,:) = ones(1,pdty);
if Face6(3,:)==1
    Face6(3,:)=high;
end

Face5(1,:) = X1; Face5(2,:) = ones(1,pdty); Face5(3,:) = Z1;
if Face5(2,:)==1
    Face5(2,:)=length;
end

Face4(1,:) = ones(1,pdty); Face4(2,:) = Y1; Face4(3,:) = Z1;
if Face4(1,:)==1
Face4(1,:)= width;
end

Cube = horzcat(Face1,Face2,Face3,Face4,Face5,Face6);

PC = PointCloud();
PC.TrueP=Cube;

end