function PlotPCAlignCPU(T)
%PLOTPCALIGNCPU Plot the CPU time consumption for the 3D point cloud alignement, with respect to the number of iterations.
%
%  T:  vector of time data
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : Obsolete
%DATE    : 27 mai 2011

% Create figure
hold off;
figure1 = figure('Color',[1 1 1]);

% Create axes
axes1 = axes('Parent',figure1);
box(axes1,'on');
hold(axes1,'all');

% Create plot
plot(T,'DisplayName','t');

% Create ylabel
ylabel({'CPU time [s]'});

% Create xlabel
xlabel({'Nb of iterations'});

% Create title
title({'3D Point Cloud Alignement CPU consumption'});

