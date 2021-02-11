function p = Plot_Intensity_corr(Incidence_deg, Corr_I_Tot)
mean_dist=[];

n=10;
idx=Incidence_deg==n;
B=Corr_I_Tot(idx);
idx_nan=isnan(B);
B(idx_nan)=0;
B_=mean(B);

for n=0:1:85
    idx=Incidence_deg==n;
    AAAAAA=Corr_I_Tot(idx);
    idx_nan=isnan(AAAAAA);
    AAAAAA(idx_nan)=0;
    mean_dist(1,1+n)=mean(AAAAAA)/B_;

end

p=plot(mean_dist,'r');
top_lim=1.1;
ylim([0 top_lim]);

end

