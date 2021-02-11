function PlotPCAlignMeanDist(ER)
%PLOTPCALIGNMEANDIST Plot the mean distance between the coincident points after a point cloud alignment with respect to the number of iterations.
%
%  ER:  vector of mean distance data
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : Obsolete
%DATE    : 27 mai 2011

hold off;

% Create figure
figure1 = figure('Color',[1 1 1]);

% Create axes
axes1 = axes('Parent',figure1);
box(axes1,'on');
hold(axes1,'all');

% Create plot
plot(ER,'DisplayName','ER');

% Create xlabel
xlabel({'Nb iterations'});

% Create ylabel
ylabel({'RMS + Error'});

% Create title
title({'3D Point Cloud alignment'});

