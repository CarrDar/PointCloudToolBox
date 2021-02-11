function mpc = GridFit(pc,xnodes,ynodes,varargin)
% GRIDFIT Estimate a surface on a 2d grid, based on scattered data (3D points). 
%
% GridFit uses a modified ridge estimator to generate the surface, 
% where the bias is toward smoothness.
% Remember that Gridfit is not an interpolant but rather an "estimant". 
% Its goal is a smooth surface that approximates your data, 
% but allows you to control the amount of smoothing.
% Replicates are allowed. 
% All methods extrapolate to the grid boundaries. 
%
% Note : Matlab 2011 provides such tools in the CurveFitting toolbox, to be purchased separately. 
%       Try doc tpaps. Some GUI for splines are also praised.
%
% To add :
% - Add cubic interpolant
%
% Input : 
%
% pc :  
%       a PointCloud or MeshPointCloud object containing the points to be used for estimation.
%       The x,y grid will lay on pc.P(1,:) and pc.P(2,:).
%       Those cannot ALL fall on a single line in the x-y plane. 
%       Replicate points will be treated in a least squares sense.
%       ANY points containing a NaN are ignored in the estimation
%
% xnodes and ynodes : 
%       vectors defining the nodes in the grid in the independent variables 
%       x and y, respectively. 
%       xnodes and ynodes do not need to be equally spaced. 
%       They must completely span the data. If they do not, then the
%       'extend' property is applied, adjusting the first and last nodes to 
%       be extended as necessary. See below for a complete description of 
%       the 'extend' property.
%
%       If xnodes and/or ynodes are a scalar integer, then they specify 
%       the number of equally spaced nodes between the min and max of the data.
%
% Ouput :
%
% mpc :
%       a MeshPointCloud object containing the x and y grids as returned by
%       meshgrid(xnodes,ynodes) and a array of size (nx,ny) containing the fitted surface (z).
%
% Additional parameters that can be set via their field name :
%
% Smoothness 
%       {1} | scalar or vector of length 2 in ]0, inf[
%       Determines the eventual smoothness of the estimated surface. 
%       A larger value here means the surface will be smoother. 
%       Smoothness must be a non-negative real number.
%
%       If this parameter is a vector of length 2, then it defines
%       the relative smoothing to be associated with the x and y
%       variables. This allows the user to apply a different amount
%       of smoothing in the x dimension compared to the y dimension.
%
%       Note: the problem is normalised in advance so that a
%       smoothness of 1 MAY generate reasonable results. If you
%       find the result is too smooth, then use a smaller value
%       for this parameter. Likewise, bumpy surfaces suggest use
%       of a larger value. (Sometimes, use of an iterative solver
%       with too small a limit on the maximum number of iterations
%       will result in non-convergence.)
%
% Interpolation
%       {'triangle'} | 'bilinear' | 'nearest'
%       Denotes the interpolation scheme used to interpolate the data.
%
%       - 'bilinear' makes use of bilinear interpolation within the grid
%          (also known as tensor product linear interpolation)
%
%       - 'triangle' splits each cell in the grid into a triangle,
%          then interpolates linearly inside each triangle.
%
%       - 'nearest' makes use of nearest neighbor interpolation. 
%          This will rarely be a good choice, this option is included for 
%          completeness.
%
%
% Regularizer
%       {'gradient'} | 'diffusion','laplacian' | 'springs'
%       Denotes the regularization paradignm to be used.
%
%       - 'diffusion' or 'laplacian' uses a finite difference
%         approximation to the Laplacian operator (i.e, del^2).
%
%         We can think of the surface as a plate, wherein the
%         bending rigidity of the plate is specified by the user
%         as a number relative to the importance of fidelity to
%         the data. A stiffer plate will result in a smoother
%         surface overall, but fit the data less well.
%         A simple plate is modeled using the Laplacian, del^2. (A
%         projected enhancement is to do a better job with the
%         plate equations.)
%
%         We can also view the regularizer as a diffusion problem,
%         where the relative thermal conductivity is supplied.
%         Here interpolation is seen as a problem of finding the
%         steady temperature profile in an object, given a set of
%         points held at a fixed temperature. Extrapolation will
%         be linear. Both paradigms are appropriate for a Laplacian
%         regularizer.
%
%       - 'gradient' attempts to ensure the gradient is as smooth
%         as possible everywhere. It is subtly different from the
%         'diffusion' option, in that here the directional
%         derivatives are biased to be smooth across cell
%         boundaries in the grid.
%
%         The gradient option uncouples the terms in the Laplacian.
%         Think of it as two coupled PDEs instead of one PDE. Why
%         are they different at all? The terms in the Laplacian
%         can balance each other.
%
%       - 'springs' uses a spring model connecting nodes to each
%         other, as well as connecting data points to the nodes
%         in the grid. This choice will cause any extrapolation
%         to be as constant as possible.
%
%         Here the smoothing parameter is the relative stiffness
%         of the springs connecting the nodes to each other compared
%         to the stiffness of a spting connecting the lattice to
%         each data point. Since all springs have a rest length
%         (length at which the spring has zero potential energy)
%         of zero, any extrapolation will be minimized.
%
%         Note: The 'springs' regularizer tends to drag the surface
%         towards the mean of all the data, so too large a smoothing
%         parameter may be a problem.
%
%
% Solver 
%       {'\','backslash'} | 'normal' | 'symmlq' | 'lsqr'
%       Denotes the solver used for the resulting linear system. 
%       Different solvers will have different solution times depending upon 
%       the specific problem to be solved. Up to a certain size grid, the
%       direct 'backslash' solver will often be speedy, until memory swaps causes problems.
%
%       What solver should you use ? Problems with a significant amount of 
%       extrapolation should avoid lsqr. 'backslash' may be best numerically for 
%       small smoothnesss parameters and high extents of extrapolation.
%
%       Large numbers of points will slow down the direct 'backslash', 
%       but when applied to the normal equations, 'backslash' can be quite fast. 
%       Since the equations generated by these methods will tend to be well 
%       conditioned, the normal equations are not a bad choice of method to use. 
%       Beware when a small smoothing parameter is used, since this will
%       make the equations less well conditioned.
%
%       - '\' or 'backslash' use matlab's backslash operator to solve the sparse
%         system. 'backslash' is an alternate name.
%
%       - 'symmlq' uses matlab's iterative symmlq solver
%
%       - 'lsqr' uses matlab's iterative lsqr solver
%
%       - 'normal' uses '\' to solve the normal equations.
%
%
% MaxIter
%       { min(10000,length(xnodes)*length(ynodes)) } | integer in [1,inf]
%       Only applies to iterative solvers. Defines the maximum number of 
%       iterations for an iterative solver.
%
% Extend 
%       {'warning'} | 'never' | 'always'
%       Controls whether the first and last nodes in each dimension are 
%       allowed to be adjusted to bound the data, and whether the user will 
%       be warned if this was deemed necessary to happen.
%
%       - 'warning' adjusts the first and/or last node in x or y if the nodes 
%         do not FULLY contain the data. Issue a warning message to this
%         effect, telling the amount of adjustment applied.
%
%       - 'never' issues an error message when the nodes do not absolutely contain the data.
%
%       - 'always' automatically adjusts the first and last nodes in each 
%         dimension if necessary. No warning is given when this option is set.
%
%
% TileSize
%       {inf} | integer in [1,inf[
%       Grids which are simply too large to solve for in one single 
%       estimation step can be built as a set of tiles. 
%       For example, a 1000x1000 grid will require the estimation of 1e6 unknowns. 
%       This is likely to require more memory (and time) than you have available.
%       But if your data is dense enough, then you can model
%       it locally using smaller tiles of the grid.
%
%       A recommendation for a reasonable tilesize is roughly 100 to 200. 
%       Tiles of this size take only a few seconds to solve normally, 
%       so the entire grid can be modeled in a finite amount of time. 
%
%       If your data is so sparse than some tiles contain insufficient data 
%       to model, then those tiles will be left as NaNs.
%
% Overlap
%       {0.2} | scalar in [0,0.5]
%       Tiles in a grid have some overlap, so they can minimize any problems 
%       along the edge of a tile. In this overlapped region, the grid is 
%       built using a bi-linear combination of the overlapping tiles.
%
%       The overlap is specified as a fraction of the tile size, so an overlap 
%       of 0.20 means there will be a 20 percent overlap of successive tiles. 
%       A zero overlap is not allowed, and it must be no more than 1/2.
%
% Autoscale
%       {on} | off
%       Some data may have widely different scales on the respective x and y axes. 
%       If this happens, then the regularization may experience difficulties. 
%          
%       Autoscale = 'on' will cause GridFit to scale the x and y node intervals
%       to a unit length. This should improve the regularization procedure. 
%       The scaling is purely internal. 
%
%       Autoscale = 'off' will disable automatic scaling.
%
% TrueP
%       {false} | true
%       if TrueP is true, GridFit will use the positions found in pc.TrueP.
%
%
% Speed considerations :
% - Remember that GridFit must solve a LARGE system of linear equations. 
%   There will be as many unknowns as the total number of nodes in the final lattice. 
%   While these equations may be sparse, solving a system of 10000 equations 
%   takes less than a second. Very large problems may benefit from the iterative 
%   solvers or from tiling.
%
% Disclaimer :
%   This function comes with no warranty whatsoever. The responsability is 
%   upon the user to test thoroughly that it yields results consistent with expectations. 
%   Please signal any bug encountered.
%   Please feel free to adapt and distribute.
%
%   This function is an adaptation for MatlabPointCloudToolbox of GridFit 
%   version 2.0 developped by John D'Errico (woodchips@rochester.rr.com)
%   Respect the licenses.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 3.2
%STATUS  : OK
%DATE    : 17 august 2011


%% Validate input arguments.
ip = inputParser;

ip.addRequired( 'pc', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud') );
ip.addRequired( 'xnodes', @(x)isnumeric(x) );
ip.addRequired( 'ynodes', @(x)isnumeric(x) );

ip.addParamValue('Smoothness', 1, @(x)isnumeric(x) );
validInt = {'triangle','bilinear','nearest'};
ip.addParamValue('Interpolation', 'triangle', @(x)any(strcmpi(x,validInt)));
validReg = {'gradient','diffusion','laplacian','springs'};
ip.addParamValue('Regularizer', 'gradient', @(x)any(strcmpi(x,validReg)));
validSol = {'normal','\','backslash','symmlq','lsqr'};
ip.addParamValue('Solver','normal',@(x)any(strcmpi(x,validSol)));
ip.addParamValue('MaxIter',[], @(x)isnumeric(x) && x>=1 );
validExt = {'warning','never','always'};
ip.addParamValue('Extend','warning',@(x)any(strcmpi(x,validExt)));
ip.addParamValue('TileSize', inf, @(x)isnumeric(x) && x>0 && isscalar(x));
ip.addParamValue('Overlap', 0.2, @(x)isnumeric(x) && isscalar(x) && x>=0 && x<=0.5);
validAS = {'on','off'};
ip.addParamValue('Autoscale','on',@(x)any(strcmpi(x,validAS)));
ip.addParamValue('TrueP',false,@(x)islogical(x));

ip.parse(pc,xnodes,ynodes,varargin{:});
arg = ip.Results;

% I work only with PointClouds
if isa(pc,'MeshPointCloud'), pc = pc.PointCloud; end

% Sanity checks smoothness
if (numel(arg.Smoothness)>2) || any(arg.Smoothness<=0)
    error 'Smoothness must be scalar (or length 2 vector), real, finite, and positive.'
end

% Add some internal parameters
arg.Mask = [];
arg.maskflag = [];
arg.xscale = 1;
arg.yscale = 1;

% Drop any NaN data
%pc.RemoveNans; % Bad idea here

% Get x,y,z as column vectors,
if arg.TrueP
    if isempty(pc.TrueP), error('GridFit:NoP','No true positions to fit on.'); end
    x = transpose(pc.TrueP(1,:));
    y = transpose(pc.TrueP(2,:));
    z = transpose(pc.TrueP(3,:));
else
    if isempty(pc.P), error('GridFit:NoP','No positions to fit on.'); end
    x = transpose(pc.P(1,:));
    y = transpose(pc.P(2,:));
    z = transpose(pc.P(3,:));
end
% Remove any nan.
k = isnan(x) | isnan(y) | isnan(z);
if any(k)
  x(k)=[];
  y(k)=[];
  z(k)=[];
end

xmin = min(x);
xmax = max(x);
ymin = min(y);
ymax = max(y);

% Initialise x,ynodes if scalar is provided
if length(xnodes)==1
  xnodes = linspace(xmin,xmax,xnodes)';
  xnodes(end) = xmax; % make sure it hits the max
end
if length(ynodes)==1
  ynodes = linspace(ymin,ymax,ynodes)';
  ynodes(end) = ymax; % make sure it hits the max
end

% Be sure it's in column format.
xnodes=xnodes(:);
ynodes=ynodes(:);
nx = length(xnodes);
ny = length(ynodes);

% set the scaling if autoscale was on
if strcmpi(arg.Autoscale,'on')
    dx = diff(xnodes);
    dy = diff(ynodes);
    arg.xscale = mean(dx);
    arg.yscale = mean(dy);
    arg.Autoscale = 'off';
end

if isinf(arg.TileSize) && nx*ny > 400000
    warning('GridFit:LN','The number of nodes is faily large : %i and %i . Consider setting the TileSize option.',nx,ny);
end

% Is tiling requested ?
if (arg.TileSize < max(nx,ny))
  % split it into smaller tiles. 
  mpc = tiled_gridfit(x,y,z,xnodes,ynodes,arg);
else
  % Single tile.
  mpc = base_gridfit(x,y,z,xnodes,ynodes,arg);
end

end % GridFit

function mpc = tiled_gridfit(x,y,z,xnodes,ynodes,arg)
% tiled_gridfit is a tiled version of GridFit, continuous across tile boundaries 
%
% usage: [zgrid,xgrid,ygrid]=tiled_gridfit(x,y,z,xnodes,ynodes,arg)
%
% Tiled_gridfit is used when the total grid is far too large
% to model using a single call to GridFit. While GridFit may take
% only a second or so to build a 100x100 grid, a 2000x2000 grid
% will probably not run at all due to memory problems.
%
% Tiles in the grid with insufficient data (<4 points) will be
% filled with NaNs. Avoid use of too small tiles, especially
% if your data has holes in it that may encompass an entire tile.
%
% A mask may also be applied, in which case tiled_gridfit will
% subdivide the mask into tiles. Note that any boolean mask
% provided is assumed to be the size of the complete grid.
%
% Tiled_gridfit may not be fast on huge grids, but it should run
% as long as you use a reasonable tilesize. 8-)

% Matrix elements in a square tile
tilesize = arg.TileSize;
% Size of overlap in terms of matrix elements. Overlaps
% of purely zero cause problems, so force at least two
% elements to overlap.
overlap = max(2,floor(tilesize*arg.Overlap));

Tparams = arg;

nx = length(xnodes);
ny = length(ynodes);
zgrid = zeros(ny,nx);

% linear ramp for the bilinear interpolation
rampfun = inline('(t-t(1))/(t(end)-t(1))','t');

% loop over each tile in the grid
h = waitbar(0,'Relax and have a cup of JAVA. Its my treat.');
warncount = 0;
xtind = 1:min(nx,tilesize);
while ~isempty(xtind) && (xtind(1)<=nx)
  
  xinterp = ones(1,length(xtind));
  if (xtind(1) ~= 1)
    xinterp(1:overlap) = rampfun(xnodes(xtind(1:overlap)));
  end
  if (xtind(end) ~= nx)
    xinterp((end-overlap+1):end) = 1-rampfun(xnodes(xtind((end-overlap+1):end)));
  end
  
  ytind = 1:min(ny,tilesize);
  while ~isempty(ytind) && (ytind(1)<=ny)
    % update the waitbar
    waitbar((xtind(end)-tilesize)/nx + tilesize*ytind(end)/ny/nx)
    
    yinterp = ones(length(ytind),1);
    if (ytind(1) ~= 1)
      yinterp(1:overlap) = rampfun(ynodes(ytind(1:overlap)));
    end
    if (ytind(end) ~= ny)
      yinterp((end-overlap+1):end) = 1-rampfun(ynodes(ytind((end-overlap+1):end)));
    end
    
    % was a mask supplied?
    if ~isempty(arg.Mask)
      submask = arg.Mask(ytind,xtind);
      Tparams.Mask = submask;
    end
    
    % extract data that lies in this grid tile
    k = (x>=xnodes(xtind(1))) & (x<=xnodes(xtind(end))) & ...
        (y>=ynodes(ytind(1))) & (y<=ynodes(ytind(end)));
    k = find(k);
    
    if length(k)<4
      if warncount == 0
        warning('GRIDFIT:tiling','A tile was too underpopulated to model. Filled with NaNs.')
      end
      warncount = warncount + 1;
      
      % fill this part of the grid with NaNs
      zgrid(ytind,xtind) = NaN;
      
    else
      % build this tile
      zgtile = base_gridfit(x(k),y(k),z(k),xnodes(xtind),ynodes(ytind),Tparams);
      
      % bilinear interpolation (using an outer product)
      interp_coef = yinterp*xinterp;
      
      % accumulate the tile into the complete grid
      zgrid(ytind,xtind) = zgrid(ytind,xtind) + zgtile.*interp_coef;
      
    end
    
    % step to the next tile in y
    if ytind(end)<ny
      ytind = ytind + tilesize - overlap;
      % are we within overlap elements of the edge of the grid?
      if (ytind(end)+max(3,overlap))>=ny
        % extend this tile to the edge
        ytind = ytind(1):ny;
      end
    else
      ytind = ny+1;
    end
    
  end % while loop over y
  
  % step to the next tile in x
  if xtind(end)<nx
    xtind = xtind + tilesize - overlap;
    % are we within overlap elements of the edge of the grid?
    if (xtind(end)+max(3,overlap))>=nx
      % extend this tile to the edge
      xtind = xtind(1):nx;
    end
  else
    xtind = nx+1;
  end

end % while loop over x

% close down the waitbar
close(h)

if warncount>0
  warning('GRIDFIT:tiling',[num2str(warncount),' tiles were underpopulated & filled with NaNs'])
end

% Create MeshPointCloud object
mpc = MeshPointCloud(xnodes,ynodes,zgrid);


end % tiled_gridfit

function mpc = base_gridfit(x,y,z,xnodes,ynodes,arg)
% base_gridfit Core function that do the job.

xmin = min(x);
xmax = max(x);
ymin = min(y);
ymax = max(y);
nx = length(xnodes);
ny = length(ynodes);
ngrid = nx*ny;
dx = diff(xnodes);
dy = diff(ynodes);
 
% Mask must be either an empty array, or a boolean
% aray of the same size as the final grid.
nmask = size(arg.Mask);
if ~isempty(arg.Mask) && ((nmask(2)~=nx) || (nmask(1)~=ny))
    if ((nmask(2)==ny) || (nmask(1)==nx))
        error 'Mask array is probably transposed from proper orientation.'
    else
        error 'Mask array must be the same size as the final grid.'
    end
end
if ~isempty(arg.Mask)
    arg.maskflag = 1;
else
    arg.maskflag = 0;
end

% default for maxiter?
if isempty(arg.MaxIter)
    arg.MaxIter = min(10000,nx*ny);
end

% check lengths of the data
n = length(x);
if (length(y)~=n) || (length(z)~=n)
    error 'Data vectors are incompatible in size.'
end
if n<3
    error 'Insufficient data for surface estimation.'
end

% verify the nodes are distinct
if any(diff(xnodes)<=0) || any(diff(ynodes)<=0)
    error 'xnodes and ynodes must be monotone increasing'
end

% do we need to tweak the first or last node in x or y?
if xmin<xnodes(1)
    switch arg.Extend
        case 'always'
            xnodes(1) = xmin;
        case 'warning'
            warning('GRIDFIT:extend',['xnodes(1) was decreased by: ',num2str(xnodes(1)-xmin),', new node = ',num2str(xmin)])
            xnodes(1) = xmin;
        case 'never'
            error(['Some x (',num2str(xmin),') falls below xnodes(1) by: ',num2str(xnodes(1)-xmin)])
    end
end
if xmax>xnodes(end)
    switch arg.Extend
        case 'always'
            xnodes(end) = xmax;
        case 'warning'
            warning('GRIDFIT:extend',['xnodes(end) was increased by: ',num2str(xmax-xnodes(end)),', new node = ',num2str(xmax)])
            xnodes(end) = xmax;
        case 'never'
            error(['Some x (',num2str(xmax),') falls above xnodes(end) by: ',num2str(xmax-xnodes(end))])
    end
end
if ymin<ynodes(1)
    switch arg.Extend
        case 'always'
            ynodes(1) = ymin;
        case 'warning'
            warning('GRIDFIT:extend',['ynodes(1) was decreased by: ',num2str(ynodes(1)-ymin),', new node = ',num2str(ymin)])
            ynodes(1) = ymin;
        case 'never'
            error(['Some y (',num2str(ymin),') falls below ynodes(1) by: ',num2str(ynodes(1)-ymin)])
    end
end
if ymax>ynodes(end)
    switch arg.Extend
        case 'always'
            ynodes(end) = ymax;
        case 'warning'
            warning('GRIDFIT:extend',['ynodes(end) was increased by: ',num2str(ymax-ynodes(end)),', new node = ',num2str(ymax)])
            ynodes(end) = ymax;
        case 'never'
            error(['Some y (',num2str(ymax),') falls above ynodes(end) by: ',num2str(ymax-ynodes(end))])
    end
end

  % determine which cell in the array each point lies in
  [~,indx] = histc(x,xnodes); 
  [~,indy] = histc(y,ynodes); 
  % any point falling at the last node is taken to be
  % inside the last cell in x or y.
  k=(indx==nx);
  indx(k)=indx(k)-1;
  k=(indy==ny);
  indy(k)=indy(k)-1;
  ind = indy + ny*(indx-1);
  
  % Do we have a mask to apply?
  if arg.maskflag
    % if we do, then we need to ensure that every
    % cell with at least one data point also has at
    % least all of its corners unmasked.
    arg.Mask(ind) = 1;
    arg.Mask(ind+1) = 1;
    arg.Mask(ind+ny) = 1;
    arg.Mask(ind+ny+1) = 1;
  end
  
  % interpolation equations for each point
  tx = min(1,max(0,(x - xnodes(indx))./dx(indx)));
  ty = min(1,max(0,(y - ynodes(indy))./dy(indy)));
  
  % Future enhancement: add cubic interpolant
  switch arg.Interpolation
    case 'triangle'
      % linear interpolation inside each triangle
      k = (tx > ty);
      L = ones(n,1);
      L(k) = ny;
      
      t1 = min(tx,ty);
      t2 = max(tx,ty);
      A = sparse(repmat((1:n)',1,3),[ind,ind+ny+1,ind+L], ...
        [1-t2,t1,t2-t1],n,ngrid);
      
    case 'nearest'
      % nearest neighbor interpolation in a cell
      k = round(1-ty) + round(1-tx)*ny;
      A = sparse((1:n)',ind+k,ones(n,1),n,ngrid);
      
    case 'bilinear'
      % bilinear interpolation in a cell
      A = sparse(repmat((1:n)',1,4),[ind,ind+1,ind+ny,ind+ny+1], ...
        [(1-tx).*(1-ty), (1-tx).*ty, tx.*(1-ty), tx.*ty], ...
        n,ngrid);
      
  end
  rhs = z;

    % do we have relative smoothing parameters?  
  if numel(arg.Smoothness) == 1
    % it was scalar, so treat both dimensions equally
    smoothparam = arg.Smoothness;
    xyRelativeStiffness = [1;1];
  else
    % It was a vector, so anisotropy reigns.
    % I've already checked that the vector was of length 2
    smoothparam = sqrt(prod(arg.Smoothness));
    xyRelativeStiffness = arg.Smoothness(:)./smoothparam;
  end
  
   % Build regularizer. Add del^4 regularizer one day.
  switch arg.Regularizer
    case 'springs'
      % zero "rest length" springs
      [i,j] = meshgrid(1:nx,1:(ny-1));
      ind = j(:) + ny*(i(:)-1);
      m = nx*(ny-1);
      stiffness = 1./(dy/arg.yscale);
      Areg = sparse(repmat((1:m)',1,2),[ind,ind+1], ...
        xyRelativeStiffness(2)*stiffness(j(:))*[-1 1], ...
        m,ngrid);
      
      [i,j] = meshgrid(1:(nx-1),1:ny);
      ind = j(:) + ny*(i(:)-1);
      m = (nx-1)*ny;
      stiffness = 1./(dx/arg.xscale);
      Areg = [Areg;sparse(repmat((1:m)',1,2),[ind,ind+ny], ...
        xyRelativeStiffness(1)*stiffness(i(:))*[-1 1],m,ngrid)];
      
      [i,j] = meshgrid(1:(nx-1),1:(ny-1));
      ind = j(:) + ny*(i(:)-1);
      m = (nx-1)*(ny-1);
      stiffness = 1./sqrt((dx(i(:))/arg.xscale/xyRelativeStiffness(1)).^2 + ...
        (dy(j(:))/arg.yscale/xyRelativeStiffness(2)).^2);
      
      Areg = [Areg;sparse(repmat((1:m)',1,2),[ind,ind+ny+1], ...
        stiffness*[-1 1],m,ngrid)];
      
      Areg = [Areg;sparse(repmat((1:m)',1,2),[ind+1,ind+ny], ...
        stiffness*[-1 1],m,ngrid)];
      
    case {'diffusion' 'laplacian'}
      % thermal diffusion using Laplacian (del^2)
      [i,j] = meshgrid(1:nx,2:(ny-1));
      ind = j(:) + ny*(i(:)-1);
      dy1 = dy(j(:)-1)/arg.yscale;
      dy2 = dy(j(:))/arg.yscale;
      
      Areg = sparse(repmat(ind,1,3),[ind-1,ind,ind+1], ...
        xyRelativeStiffness(2)*[-2./(dy1.*(dy1+dy2)), ...
        2./(dy1.*dy2), -2./(dy2.*(dy1+dy2))],ngrid,ngrid);
      
      [i,j] = meshgrid(2:(nx-1),1:ny);
      ind = j(:) + ny*(i(:)-1);
      dx1 = dx(i(:)-1)/arg.xscale;
      dx2 = dx(i(:))/arg.xscale;
      
      Areg = Areg + sparse(repmat(ind,1,3),[ind-ny,ind,ind+ny], ...
        xyRelativeStiffness(1)*[-2./(dx1.*(dx1+dx2)), ...
        2./(dx1.*dx2), -2./(dx2.*(dx1+dx2))],ngrid,ngrid);
      
    case 'gradient'
      % Subtly different from the Laplacian. A point for future
      % enhancement is to do it better for the triangle interpolation
      % case.
      [i,j] = meshgrid(1:nx,2:(ny-1));
      ind = j(:) + ny*(i(:)-1);
      dy1 = dy(j(:)-1)/arg.yscale;
      dy2 = dy(j(:))/arg.yscale;
      
      Areg = sparse(repmat(ind,1,3),[ind-1,ind,ind+1], ...
        xyRelativeStiffness(2)*[-2./(dy1.*(dy1+dy2)), ...
        2./(dy1.*dy2), -2./(dy2.*(dy1+dy2))],ngrid,ngrid);
      
      [i,j] = meshgrid(2:(nx-1),1:ny);
      ind = j(:) + ny*(i(:)-1);
      dx1 = dx(i(:)-1)/arg.xscale;
      dx2 = dx(i(:))/arg.xscale;
      
      Areg = [Areg;sparse(repmat(ind,1,3),[ind-ny,ind,ind+ny], ...
        xyRelativeStiffness(1)*[-2./(dx1.*(dx1+dx2)), ...
        2./(dx1.*dx2), -2./(dx2.*(dx1+dx2))],ngrid,ngrid)];
      
  end
  nreg = size(Areg,1);
  
  % Append the regularizer to the interpolation equations,
  % scaling the problem first. Use the 1-norm for speed.
  NA = norm(A,1);
  NR = norm(Areg,1);
  A = [A;Areg*(smoothparam*NA/NR)];
  rhs = [rhs;zeros(nreg,1)];
  % do we have a mask to apply?
  if arg.maskflag
    unmasked = find(arg.Mask);
  end
  % solve the full system, with regularizer attached
  switch arg.Solver
    case {'\' 'backslash'}
      if arg.maskflag
        % there is a mask to use
        zgrid=nan(ny,nx);
        zgrid(unmasked) = A(:,unmasked)\rhs;
      else
        % no mask
        zgrid = reshape(A\rhs,ny,nx);
      end
      
    case 'normal'
      % The normal equations, solved with \. Can be faster
      % for huge numbers of data points, but reasonably
      % sized grids. The regularizer makes A well conditioned
      % so the normal equations are not a terribly bad thing
      % here.
      if arg.maskflag
        % there is a mask to use
        Aunmasked = A(:,unmasked);
        zgrid=nan(ny,nx);
        zgrid(unmasked) = (Aunmasked'*Aunmasked)\(Aunmasked'*rhs);
      else
        zgrid = reshape((A'*A)\(A'*rhs),ny,nx);
      end
      
    case 'symmlq'
      % iterative solver - symmlq - requires a symmetric matrix,
      % so use it to solve the normal equations. No preconditioner.
      tol = abs(max(z)-min(z))*1.e-13;
      if arg.maskflag
        % there is a mask to use
        zgrid=nan(ny,nx);
        [zgrid(unmasked),flag] = symmlq(A(:,unmasked)'*A(:,unmasked), ...
          A(:,unmasked)'*rhs,tol,arg.MaxIter);
      else
        [zgrid,flag] = symmlq(A'*A,A'*rhs,tol,arg.MaxIter);
        zgrid = reshape(zgrid,ny,nx);
      end
      % display a warning if convergence problems
      switch flag
        case 0
          % no problems with convergence
        case 1
          % SYMMLQ iterated MAXIT times but did not converge.
          warning('GRIDFIT:Solver',['Symmlq performed ',num2str(arg.MaxIter), ...
            ' iterations but did not converge.'])
        case 3
          % SYMMLQ stagnated, successive iterates were the same
          warning('GRIDFIT:Solver','Symmlq stagnated without apparent convergence.')
        otherwise
          warning('GRIDFIT:Solver',['One of the scalar quantities calculated in',...
            ' symmlq was too small or too large to continue computing.'])
      end
      
    case 'lsqr'
      % iterative solver - lsqr. No preconditioner here.
      tol = abs(max(z)-min(z))*1.e-13;
      if arg.maskflag
        % there is a mask to use
        zgrid=nan(ny,nx);
        [zgrid(unmasked),flag] = lsqr(A(:,unmasked),rhs,tol,arg.MaxIter);
      else
        [zgrid,flag] = lsqr(A,rhs,tol,arg.MaxIter);
        zgrid = reshape(zgrid,ny,nx);
      end
      
      % display a warning if convergence problems
      switch flag
        case 0
          % no problems with convergence
        case 1
          % lsqr iterated MAXIT times but did not converge.
          warning('GRIDFIT:Solver',['Lsqr performed ', ...
            num2str(arg.MaxIter),' iterations but did not converge.'])
        case 3
          % lsqr stagnated, successive iterates were the same
          warning('GRIDFIT:Solver','Lsqr stagnated without apparent convergence.')
        case 4
          warning('GRIDFIT:Solver',['One of the scalar quantities calculated in',...
            ' LSQR was too small or too large to continue computing.'])
      end
      
  end  % switch arg.Solver  
  
  % Create MeshPointCloud object
  mpc = MeshPointCloud(xnodes,ynodes,zgrid);

end

