function [PCRockfall,T1]=RockfallExtract(Pc1,Pc2,T1,T2,a,fig)
%% %%%%%%%%%%%%%%%%%%%%%%%% ROCKFALL EXTRACTION %%%%%%%%%%%%%%%%%%%%%%
% The method for point cloud distribution is Point-to-Point method.
% If you want a better segmentation choose Point-to-Surface method but it
% will increase time of computing.
%
% INPUT: - a (Mesh)PointCloud object for post-failure surface Pc1.
%        - a (Mesh)PointCloud object for pre-failure surface Pc2.
%        - T1 first threshold
%        - T2 second threshold
%        - a method to compute point cloud comparison 'P2S' or 'P2P'
%        - fig if you want to plot the data 
%
% OUTPUT:
%        - a PointCloud object with point belonging to rockfalls and noise.
%        - T1 return first threshold if not insert by the user
%

% Validate input arguments.

% Sanity check

% If MeshPointCloud are given, change them into PointCloud
if isa(Pc1,'MeshPointCloud'), Pc1 = Pc1.PointCloud; end
if isa(Pc2,'MeshPointCloud'), Pc1 = Pc2.PointCloud; end

% Sanity check of the thresholds
if ~isempty(T1) && T1 <= 0
    error('ExtractRockfall:threshold','First threshold T1 must be positive!');
end

if ~isempty(T2) && T2 <= 0
    error('ExtractRockfall:threshold2','Second threshold T2 must be positive!');
end

if ~isempty(a) && ~ischar(a)
    error('ExtractRockfall:threshold','Computing method must be is point-to-surface "P2S" or point-to-point "P2P"');
end

% if ~isempty(fig) && fig=1
%     error('ExtractRockfall:threshold','Error in input argument! fig  1 or 0');
% else isempty(fig)
%     fig=0;
% end


%%
if strcmp(a,'P2P')
    Comparison(Pc1,Pc2,'Method',{'P2P'});
    Comparison(Pc2,Pc1,'Method',{'P2P'});
    D2  = Pc1.UsefulVar(1,:);
    DD2 = Pc2.UsefulVar(1,:);

% Plot the distribution of euclidian distance between points
% Allow to estimate the noise for the next IF loop
    if isempty(T1)
        % Create the edges of the histogram
        data=cat(2,D2,DD2);
        mode_1=std(data);
        MAX = max(data);
        MIN = min(data);
        step=(MAX-MIN)/100;
        edges= MIN:step:MAX;
                              %                                       /|\
                              %                                      / ¦ \
         Threshold = mode_1/2;% distance between 0 and mode + mode  /  ¦  \/\____
        if fig == 1           %                                    0   m  2m     
            figure;
            hax=axes;
            hist(cat(2,D2,DD2),length(edges));
            hold on
            line([Threshold Threshold],get(hax,'YLim'),'Color',[1 0 0]);
            line([-Threshold -Threshold],get(hax,'YLim'),'Color',[1 0 0]);
            str= 'Distribution of distance between 2 point clouds';
            str = sprintf('%s   -   Threshold 1 = %.3f',str,Threshold);
            title(str,'fontsize',12);
            hold off
        end

    else
        Threshold = T1;
        data=cat(2,D2,DD2);
        MAX = max(data);
        MIN = min(data);
        step=(MAX-MIN)/100;
        edges= MIN:step:MAX;
        if fig == 1
            figure;
            hax=axes;
            hist(data,length(edges));
            hold on
            line([Threshold Threshold],get(hax,'YLim'),'Color',[1 0 0]);
            line([-Threshold -Threshold],get(hax,'YLim'),'Color',[1 0 0]);
            str= 'Distribution of distance between 2 point clouds';
            str = sprintf('%s   -   Threshold 1 = %.3f',str,Threshold);
            title(str,'fontsize',12);
            hold off
        end
    end
    %%% Create Point cloud with point belonging to the fallen block.
    % Post rockfall surface
    Pc_Block_post=PointCloud('',Pc1.P(:,Pc1.UsefulVar(1,:) > Threshold | Pc1.UsefulVar(1,:) < -(Threshold)));
    
    % Pre rockfall surface
    Pc_Block_pre = PointCloud('',Pc2.P(:,Pc2.UsefulVar(1,:) > Threshold | Pc2.UsefulVar(1,:) < -(Threshold)));

    T1=Threshold;
end

%%
if strcmp(a,'P2S')
    Comparison(Pc2,Pc1,'Method',{'P2S'});
    Comparison(Pc1,Pc2,'Method',{'P2S'});
    D2  = Pc1.UsefulVar(2,:);
    DD2 = Pc2.UsefulVar(2,:);

% Plot the distribution of euclidian distance between points
% Allow to estimate the noise for the next IF loop    
    if isempty(T1)
        % Create the edges of the histogram
        data=cat(2,D2,DD2);
        mode_1=std(data);
        MAX = max(data);
        MIN = min(data);
        step=(MAX-MIN)/100;
        edges= MIN:step:MAX;
                              %                                       /|\
                              %                                      / ¦ \
         Threshold = mode_1/2;% distance between 0 and mode + mode  /  ¦  \/\____
        if fig == 1           %                                    0   m  2m
            figure;
            hax=axes;
            hist(cat(2,D2,DD2),length(edges));
            hold on
            line([Threshold Threshold],get(hax,'YLim'),'Color',[1 0 0]);
            line([-Threshold -Threshold],get(hax,'YLim'),'Color',[1 0 0]);
            str= 'Distribution of distance between 2 point clouds';
            str = sprintf('%s   -   Threshold 1 = %.3f',str,Threshold);
            title(str,'fontsize',12);
            hold off
        end
        
    else
        Threshold = T1;
        MAX = max(D2);
        MIN = min(D2);
        step=(MAX-MIN)/100;
        edges= MIN:step:MAX;
        if fig == 1
            figure;
            hax=axes;
            hist(cat(2,D2,DD2),length(edges));
            hold on
            line([Threshold Threshold],get(hax,'YLim'),'Color',[1 0 0]);
            line([-Threshold -Threshold],get(hax,'YLim'),'Color',[1 0 0]);
            str= 'Distribution of distance between 2 point clouds';
            str = sprintf('%s   -   Threshold 1 = %.3f',str,Threshold);
            title(str,'fontsize',12);
            hold off
        end
    end
    
    %%% Create Point cloud with point belonging to the fallen block.
    % Post rockfall surface
    Pc_Block_post=PointCloud('',Pc1.P(:,Pc1.UsefulVar(2,:) > Threshold | Pc1.UsefulVar(2,:) < -(Threshold)));
    
    % Pre rockfall surface
    Pc_Block_pre = PointCloud('',Pc2.P(:,Pc2.UsefulVar(2,:) > Threshold | Pc2.UsefulVar(2,:) < -(Threshold)));

    T1=Threshold;
end



%%
% Merging both (pre-,post-)to have one point cloud with the shape of the block
PcBlock = Add(Pc_Block_post,Pc_Block_pre);

clear Pc_Block_post Pc_Block_pre

% Save it in ACSII *.txt
% SaveInASCII(PcBlock,'BLOCK\BLOCK.txt',{'P'});



%%%%%%%%%%%%%%%%%%%%%%%%  REMOVE NON BLOCK POINTS %%%%%%%%%%%%%%%%%%%%%%%%
PcBlock.ComputeKDTree;

[~, D3] = knnsearch(PcBlock.KDTree,(PcBlock.P)', 'k', 20);

if isempty(T2)
    MAX = max(D3(:,10));
    MIN = min(D3(:,10));
    step=(MAX-MIN)/100;
    edges= MIN:step:MAX;
    [~,bin] = histc(D3(:,10),edges);
    m = mode(bin);
    mode_= edges([m, m+1]);                %                                        ^
    mode_3= mean(mode_);                   %                                       /¦\
    %                                      %                                      / ¦ \
    Threshold2 =  2*(mode_3);              % distance between 0 and mode + mode  /  ¦  \______
    %                                                                           0   m    3m
    if fig == 1
        figure;
        hax=axes;
        hist(D3(:,10),length(edges));
        hold on
        line([Threshold2 Threshold2],get(hax,'YLim'),'Color',[1 0 0]);
        str= 'Distribution of point spacing';
        str = sprintf('%s   -   Threshold 2 = %.3f',str,Threshold2);
        title(str,'fontsize',12);
        hold off
    end
else
    MAX = max(D3(:,10));
    MIN = min(D3(:,10));
    step=(MAX-MIN)/100;
    edges= MIN:step:MAX;
    Threshold2 = T2;
    
    
    if fig == 1
        figure;
        hax=axes;
        hist(D3(:,10),length(edges));
        hold on
        line([Threshold2 Threshold2],get(hax,'YLim'),'Color',[1 0 0]);
        str= 'Distribution of point spacing';
        str = sprintf('%s   -   Threshold 2 = %.3f',str,Threshold2);
        title(str,'fontsize',12);
        hold off
    end
end

PCRockfall = PointCloud('',PcBlock.P(:,D3(:,10) < Threshold2));

% clear PcBlock

PCRockfall.PlotPCLViewer({'P'});
end
    
