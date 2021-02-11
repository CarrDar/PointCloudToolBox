clear all
close all

r=1;    % donut ring radius
R=3;    % donut circle radius
%(2 x pi^2) x r^2 x R = Volume d'un tore
% =>          r R
Torus=GenTore(1,3,33);
Torus.copyTrue2MeasPos;
Torus.addNoise('OutlierProb',0.000, 'GaussSmear', [0.1 1],'DropOutProb', 0.0);
%Torus.plot3;
KK=[];
II=[];
Real_volume=(2*pi^2)*r^2*R;

for i=0.05:0.01:10
    [K, ~]=RockfallVolume(Torus,i,0);
    KK=cat(1,KK,K);
    II=cat(1,II,i);
end

figure;
subplot(4,4,[1 4]);
scatter(II,KK,'.');xlabel('Research radius [m]');ylabel('Volume [m3]');
subplot(4,4,5);view(3);
RockfallVolume(Torus,0.5,1);
subplot(4,4,6);view(3);
RockfallVolume(Torus,0.65,1);
subplot(4,4,7);view(3);
RockfallVolume(Torus,.75,1);
subplot(4,4,8);view(3);
RockfallVolume(Torus,.85,1);
subplot(4,4,9);view(3);
RockfallVolume(Torus,.95,1);
subplot(4,4,10);view(3);
RockfallVolume(Torus,1.0,1);
subplot(4,4,11);view(3);
RockfallVolume(Torus,1.25,1);
subplot(4,4,12);view(3);
RockfallVolume(Torus,1.5,1);
subplot(4,4,13);view(3);
RockfallVolume(Torus,1.75,1);
subplot(4,4,14);view(3);
RockfallVolume(Torus,2.0,1);
subplot(4,4,15);view(3);
RockfallVolume(Torus,2.55,1);
subplot(4,4,16);view(3);
RockfallVolume(Torus,inf,1);
