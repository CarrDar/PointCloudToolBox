function [mu sigma thetahat kappa] = stat_orientation(original,directory)
%
%   STAT_ORIENTATION provide some statistical information about distribution your selected areas.
%   [direction_slip1 dip_slip dispertion1 dispertion2] = stat_orientation(original,varargin)
%   Give the direction/dip for point according to displacement obtain by
%   Deformation or Shape tracking algorithm. Based on Von Mises (circular-normal distribution)
%   for angles
%
%   1/Kappa = sigma^2 for std dev.
%   theta = mu
%
% INPUT: - original: point cloud obtained after computing displacement.
%
% OUTPUT: - Plot and statistical information about the displacement of the
%           different selection have make with another software (i.e. COLTOP3D).
%
% Remarks:
% - ORIENTATION works with libraries of PointCloudToolBox developed at
%   the Institute of Geomatics and Analysis of Risk - University of Lausanne
%   (IGAR-UNIL) by Neal Gauvin. All the PointCloudToolBox is propriety of
%   IGAR-UNIL.
% Update:
%
%
% AUTHOR  : Dario Carrea (at unil dot ch)
% VERSION : Beta
% STATUS  : In progress
% DATE    : 21 January 2013
%

cd(directory);

files = dir('*.txt');
filePC = length(files);

matlabpool open local
parfor i=1:filePC
    PCSelection(i) = ImportPointCloudFromASCII(files(i).name,'',{'P'});
end
matlabpool close



dt = DelaunayTri(original(:,1:3));
for n=1:filePC
    [PI ~] = nearestNeighbor(dt,(PCSelection(1,n).P(1:3,:))');
    direction(n,1) = {PI};
    PC_direction(n,1) = {original(direction{n,1}(:,1),1:12)};
    
    x=-90:1:90;
    fitnormal = fitdist(PC_direction{n,1}(:,11),'normal');
    mu(n) = fitnormal.Params(1,1);
    sigma(n) = fitnormal.Params(1,2);
    gauss_std = normpdf(x,mu(n),sigma(n));
    
    deg = PC_direction{n,1}(:,12);
    rad_dir = ((pi/180).*deg);
    [thetahat(n) kappa(n)] = circ_vmpar(rad_dir);
    [p alpha] = circ_vmpdf([],thetahat(n),kappa(n));
    
    subplot(1,2,1);
    hist(PC_direction{n,1}(:,12),72);
    xlim([0 360]);
    ax1 = gca;
    ylabel('Frequency');
    box off
    set(ax1,'YAxisLocation','left');
    ax2 = axes('Position',get(ax1,'Position'),'YAxisLocation','right',...
        'Color','none');     
    hold on
    plot(p,'r','LineWidth',2,'Parent',ax2);
    xlim([0 360]);axis off
    xlabel('Distribution Direction');
    ylabel('Line');
    hold off
    
    subplot(1,2,2);
    hist(PC_direction{n,1}(:,11),36);
    xlim([-90 90]);
    ax1 = gca;
    ylabel('Frequency');
    box off
    set(ax1,'YAxisLocation','left');
    ax2 = axes('Position',get(ax1,'Position'),'YAxisLocation','right',...
        'Color','none');     
    hold on
    plot(gauss_std,'r','LineWidth',2);
    xlim([0 180]);axis off
    xlabel('Distribution Dip');
    ylabel('Line');
    hold off
    figure;
end
end

function [thetahat kappa] = circ_vmpar(alpha,w,d)

% r = circ_vmpar(alpha, w, d)
%   Estimate the parameters of a von Mises distribution.
%
%   Input:
%     alpha	sample of angles in radians
%     [w		number of incidences in case of binned angle data]
%     [d    spacing of bin centers for binned data, if supplied 
%           correction factor is used to correct for bias in 
%           estimation of r, in radians (!)]
%
%   Output:
%     thetahat		preferred direction
%     kappa       concentration parameter
%
% PHB 3/23/2009
%
% References:
%   Statistical analysis of circular data, N.I. Fisher
%
% Circular Statistics Toolbox for Matlab

% By Philipp Berens, 2009
% berens@tuebingen.mpg.de

alpha = alpha(:);
if nargin < 2
  w = ones(size(alpha));
end
if nargin < 3
  d = 0;
end

r = circ_r(alpha,w,d);
kappa = circ_kappa(r);

thetahat = circ_mean(alpha, w);

end
function r = circ_r(alpha, w, d, dim)
% r = circ_r(alpha, w, d)
%   Computes mean resultant vector length for circular data.
%
%   Input:
%     alpha	sample of angles in radians
%     [w		number of incidences in case of binned angle data]
%     [d    spacing of bin centers for binned data, if supplied 
%           correction factor is used to correct for bias in 
%           estimation of r, in radians (!)]
%     [dim  compute along this dimension, default is 1]
%
%     If dim argument is specified, all other optional arguments can be
%     left empty: circ_r(alpha, [], [], dim)
%
%   Output:
%     r		mean resultant length
%
% PHB 7/6/2008
%
% References:
%   Statistical analysis of circular data, N.I. Fisher
%   Topics in circular statistics, S.R. Jammalamadaka et al. 
%   Biostatistical Analysis, J. H. Zar
%
% Circular Statistics Toolbox for Matlab

% By Philipp Berens, 2009
% berens@tuebingen.mpg.de - www.kyb.mpg.de/~berens/circStat.html

if nargin < 4
  dim = 1;
end

if nargin < 2 || isempty(w) 
  % if no specific weighting has been specified
  % assume no binning has taken place
	w = ones(size(alpha));
else
  if size(w,2) ~= size(alpha,2) || size(w,1) ~= size(alpha,1) 
    error('Input dimensions do not match');
  end 
end

if nargin < 3 || isempty(d)
  % per default do not apply correct for binned data
  d = 0;
end

% compute weighted sum of cos and sin of angles
r = sum(w.*exp(1i*alpha),dim);

% obtain length 
r = abs(r)./sum(w,dim);

% for data with known spacing, apply correction factor to correct for bias
% in the estimation of r (see Zar, p. 601, equ. 26.16)
if d ~= 0
  c = d/2/sin(d/2);
  r = c*r;
end
end
function kappa = circ_kappa(alpha,w)
%
% kappa = circ_kappa(alpha,[w])
%   Computes an approximation to the ML estimate of the concentration 
%   parameter kappa of the von Mises distribution.
%
%   Input:
%     alpha   angles in radians OR alpha is length resultant
%     [w      number of incidences in case of binned angle data]
%
%   Output:
%     kappa   estimated value of kappa
%
%   References:
%     Statistical analysis of circular data, Fisher, equation p. 88
%
% Circular Statistics Toolbox for Matlab

% By Philipp Berens, 2009
% berens@tuebingen.mpg.de - www.kyb.mpg.de/~berens/circStat.html


alpha = alpha(:);

if nargin<2
  % if no specific weighting has been specified
  % assume no binning has taken place
	w = ones(size(alpha));
else
  if size(w,2) > size(w,1)
    w = w';
  end 
end

N = length(alpha);

if N>1
  R = circ_r(alpha,w);
else
  R = alpha;
end

if R < 0.53
  kappa = 2*R + R^3 + 5*R^5/6;
elseif R>=0.53 && R<0.85
  kappa = -.4 + 1.39*R + 0.43/(1-R);
else
  kappa = 1/(R^3 - 4*R^2 + 3*R);
end

if N<15 && N>1
  if kappa < 2
    kappa = max(kappa-2*(N*kappa)^-1,0);    
  else
    kappa = (N-1)^3*kappa/(N^3+N);
  end
end
end
function [mu_ ul ll] = circ_mean(alpha, w, dim)
%
% mu = circ_mean(alpha, w)
%   Computes the mean direction for circular data.
%
%   Input:
%     alpha	sample of angles in radians
%     [w		weightings in case of binned angle data]
%     [dim  compute along this dimension, default is 1]
%
%     If dim argument is specified, all other optional arguments can be
%     left empty: circ_mean(alpha, [], dim)
%
%   Output:
%     mu		mean direction
%     ul    upper 95% confidence limit
%     ll    lower 95% confidence limit 
%
% PHB 7/6/2008
%
% References:
%   Statistical analysis of circular data, N. I. Fisher
%   Topics in circular statistics, S. R. Jammalamadaka et al. 
%   Biostatistical Analysis, J. H. Zar
%
% Circular Statistics Toolbox for Matlab

% By Philipp Berens, 2009
% berens@tuebingen.mpg.de - www.kyb.mpg.de/~berens/circStat.html

if nargin < 3
  dim = 1;
end

if nargin < 2 || isempty(w)
  % if no specific weighting has been specified
  % assume no binning has taken place
	w = ones(size(alpha));
else
  if size(w,2) ~= size(alpha,2) || size(w,1) ~= size(alpha,1) 
    error('Input dimensions do not match');
  end 
end

% compute weighted sum of cos and sin of angles
r = sum(w.*exp(1i*alpha),dim);

% obtain mean by
mu_ = angle(r);

% confidence limits if desired
if nargout > 1
  t = circ_confmean(alpha,0.05,w,[],dim);
  ul = mu_ + t;
  ll = mu_ - t;
end
end
function [p alpha] = circ_vmpdf(alpha, thetahat, kappa)

% [p alpha] = circ_vmpdf(alpha, w, p)
%   Computes the circular von Mises pdf with preferred direction thetahat 
%   and concentration kappa at each of the angles in alpha
%
%   The vmpdf is given by f(phi) =
%   (1/(2pi*I0(kappa))*exp(kappa*cos(phi-thetahat)
%
%   Input:
%     alpha     angles to evaluate pdf at, if empty alphas are chosen to
%               100 uniformly spaced points around the circle
%     [thetahat preferred direction, default is 0]
%     [kappa    concentration parameter, default is 1]
%
%   Output:
%     p         von Mises pdf evaluated at alpha
%     alpha     angles at which pdf was evaluated
%
%
%   References:
%     Statistical analysis of circular data, Fisher
%
% Circular Statistics Toolbox for Matlab

% By Philipp Berens and Marc J. Velasco, 2009
% velasco@ccs.fau.edu

% if no angles are supplied, 100 evenly spaced points around the circle are
% chosen
if nargin < 1 || isempty(alpha)
    alpha = linspace(0, 2*pi, 361)';
    alpha = alpha(1:end-1);
end
if nargin < 3
    kappa = 1;
end
if nargin < 2
    thetahat = 0;
end

alpha = alpha(:);

% evaluate pdf
C = 1/(2*pi*besseli(0,kappa));
p = C * exp(kappa*cos(alpha-thetahat));
end