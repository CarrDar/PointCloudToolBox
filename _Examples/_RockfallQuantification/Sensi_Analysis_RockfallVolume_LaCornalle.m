KK=[];
II=[];

[m, n]=size(Rockfall_events);

for y=1:n
    KK=[];
    II=[];
    vari=Rockfalls_events(y);
    for i=0.05:0.01:10
        [K, ~]=RockfallVolume(vari,i,0);
        KK=cat(1,KK,K);
        II=cat(1,II,i);
    end
    [K, ~]=RockfallVolume(vari,inf,0);
    KK=cat(1,KK,K);
    II=cat(1,II,i+10);
    figure;
    subplot(4,4,[1 4]);
    scatter(II,KK,'.');xlabel('Research radius [m]');ylabel('Volume [m3]');
    subplot(4,4,5);view(3);
    RockfallVolume(vari,.05,1);
    subplot(4,4,6);view(3);
    RockfallVolume(vari,.06,1);
    subplot(4,4,7);view(3);
    RockfallVolume(vari,.07,1);
    subplot(4,4,8);view(3);
    RockfallVolume(vari,.08,1);
    subplot(4,4,9);view(3);
    RockfallVolume(vari,.09,1);
    subplot(4,4,10);view(3);
    RockfallVolume(vari,.1,1);
    subplot(4,4,11);view(3);
    RockfallVolume(vari,.11,1);
    subplot(4,4,12);view(3);
    RockfallVolume(vari,.12,1);
    subplot(4,4,13);view(3);
    RockfallVolume(vari,.13,1);
    subplot(4,4,14);view(3);
    RockfallVolume(vari,.14,1);
    subplot(4,4,15);view(3);
    RockfallVolume(vari,.15,1);
    subplot(4,4,16);view(3);
    RockfallVolume(vari,inf,1);
end

%%

KK=[];
II=[];

[m, n]=size(PCR_R);

for y=1:n
    KK=[];
    II=[];
    vari=PCR_R(y);
    vari.ComputeKDTree;
    [M N]=knnsearch(vari.KDTree,vari.P','k',2);
    %for i=max(N(:,2)):0.01:10
        [K, ~]=RockfallVolume(vari,max(N(:,2))*2,0);
        KK=cat(1,KK,K);
        II=cat(1,II,1);
    %end
    [K, ~]=RockfallVolume(vari,inf,0);
    KK=cat(1,KK,K);
    II=cat(1,II,2);
    figure;
    subplot(4,4,[1 4]);
    scatter(II,KK,'.');xlabel('Research radius [m]');ylabel('Volume [m3]');
    subplot(4,4,5);view(3);
    RockfallVolume(vari,max(N(:,2)),1);
    subplot(4,4,6);view(3);
    RockfallVolume(vari,max(N(:,2))*2,1);
    subplot(4,4,16);view(3);
    RockfallVolume(vari,inf,1);
end
