function [V,S] = alphavol(X,R,fig)
%ALPHAVOL Alpha shape of 2D or 3D point set.
%   V = ALPHAVOL(X,R) gives the area or volume V of the basic alpha shape
%   for a 2D or 3D point set. X is a coordinate matrix of size Nx2 or Nx3.
%
%   R is the probe radius with default value R = Inf. In the default case
%   the basic alpha shape (or alpha hull) is the convex hull.
%
%   [V,S] = ALPHAVOL(X,R) outputs a structure S with fields:
%    S.tri - Triangulation of the alpha shape (Mx3 or Mx4)
%    S.vol - Area or volume of simplices in triangulation (Mx1)
%    S.rcc - Circumradius of simplices in triangulation (Mx1)
%    S.bnd - Boundary facets (Px2 or Px3)
%
%   ALPHAVOL(X,R,1) plots the alpha shape.
%
% Written by Jonas Lundgren - 2010
%   splinefit@gmail.com
%   ALPHAVOL method based on previous code
%   Update Author: Dario Carrea <Dario dot Carrea at unil dot ch>
%
%   2010-09-27  First version of ALPHAVOL.
%   2010-10-05  DelaunayTri replaced by DELAUNAYN. 3D plots added.
%   2012-03-08  More output added. DELAUNAYN replaced by DELAUNAY.
%   2013-03-13  Adapted to IGAR-PointCloudToolBox by Dario Carrea.


% Sanity Check
dim = size(X,2);
if dim < 2 || dim > 3
    error('alphavol:dimension','X must have 2 or 3 columns.')
end

if ~isscalar(R) || ~isreal(R) || isnan(R)
    error('alphavol:radius','R must be a real number.')
end

% Unique points
[X,imap] = unique(X,'rows');

% Delaunay triangulation
T = delaunayTriangulation(X);

% Remove zero volume tetrahedra since
% these can be of arbitrary large circumradius
if dim == 3
    n = size(T,1);
    vol = volumes_tetra(T,X);
    epsvol = 1e-12*sum(vol)/n;
    T = T(vol > epsvol,:);
    holes = size(T,1) < n;
end

% Limit circumradius of simplices
[~,rcc] = circumcenter(triangulation(T,X));
T = T(rcc < R,:);
rcc = rcc(rcc < R);

% Volume/Area of alpha shape
vol = volumes_tetra(T,X);
V = sum(vol);

% Return?
if nargout < 2 && ~fig
    return
end



% Alpha shape boundary
if ~isempty(T)
    % Facets referenced by only one simplex
    B = freeBoundary(triangulation(T,X));
    if dim == 3 && holes
        % The removal of zero volume tetrahedra causes false boundary
        % faces in the interior of the volume. Take care of these.
        B = trueboundary(B,X);
    end
else
    B = zeros(0,dim);
end
vol = volumes_tetra(T,X);
% Plot alpha shape
if fig ==1
    if ~isempty(B)
        % Plot boundary faces
        trisurf(triangulation(B,X),'FaceColor','red','FaceAlpha',1/3);
        str = 'Volume';
    else
        cla
        str = 'Volume';
    end
    axis equal
    str = sprintf('Radius = %g,   %s = %.3f',R,str,V);
    title(str,'fontsize',12)
end

% Turn on triangulation warning
warning('off','MATLAB:triangulation:PtsNotInTriWarnId')

% Return structure
if nargout == 2
    S = struct('tri',imap(T),'vol',vol,'rcc',rcc,'bnd',imap(B));
end
end
