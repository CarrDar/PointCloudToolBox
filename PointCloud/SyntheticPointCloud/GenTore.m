function PC = GenTore( radius, Radius, nbpoints )
%GENSPHERE Generate a point cloud from the shape of a donut centered at the tore.
%
% Input :
%   radius   : radius of the tube.
%   nbpoints : the number of desired points, 100 by default.
%
% Output :
%   A handle on a PointCloud class.
%
%
%AUTHOR  : Dario Carrea (at unil dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 12 january 2018

%initialisation
if nargin < 3
    nbpoints = 100;
    if nargin < 2
        Radius = 3;
        if nargin < 1
            radius = 1;
        end
    end
end
% Sanity checks
if radius <= 0 || ~isscalar(radius)
    warning('GenTore:WR','Provide positive scalar for the radius %f, reset to 1',radius);
    radius = 1;
end

if radius <= 0 || ~isscalar(Radius)
    warning('GenTore:WR','Provide positive scalar for the Radius %f, reset to 3',Radius);
    radius = 3;
end

if nbpoints <= 0 || ~isscalar(nbpoints)
    warning('GenTore:WN','Provide positive integer for the nb of points %i, reset to 100',nbpoints);
    nbpoints = 100;
end

% I want to obtain points such that any small area on the sphere is
% expected to contain the same number of points.

R=Radius;
r=radius;
u=linspace(0,2*pi,nbpoints);
v=linspace(0,2*pi,nbpoints);
[U,V]=meshgrid(u,v);
x=(R+r.*cos(V)).*cos(U);
y=(R+r.*cos(V)).*sin(U);
z=r.*sin(V);
figure(1)
X=[];
Y=[];
Z=[];
for i=1:length(x)
    X=cat(1,X,x(:,i));
    Y=cat(1,Y,y(:,i));
    Z=cat(1,Z,z(:,i));
end
torus(1,:)=X;
torus(2,:)=Y;
torus(3,:)=Z;


PC = PointCloud();
PC.TrueP=torus;

PC.Name='Torus';

end

