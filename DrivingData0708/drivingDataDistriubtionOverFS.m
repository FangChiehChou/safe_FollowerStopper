%% This script is for computing number of human driving data points in each region of the FollowerStopper

clear all
clc


load("2020-07-08-15-15-54_2T3MWRFVXLW056972_CAN_Messages.mat")

%% filter available data
valid_accel = accelx;
valid_relative_dist = lead_distance;
valid_relative_spd = relative_vel;
valid_relative_spd(lead_distance>120) = nan;
valid_speed = speed/3.6;
valid_relative_dist(valid_relative_dist>150) = nan;

time_aligned = Time-Time(1);

[~,highway_s_I] = min(abs(time_aligned-450));
[~,highway_e_I] = min(abs(time_aligned-1301));
d_rel = valid_relative_dist(highway_s_I:highway_e_I);
v_rel = valid_relative_spd(highway_s_I:highway_e_I);
v_av = valid_speed(highway_s_I:highway_e_I);
%% compute number of points in each region of the FS

m = length(d_rel) - sum(isnan(d_rel));
region_index = zeros(m,1);
region_index_newFS = zeros(m,1);
valid_data_count = 0;

signed_dist_toC1 = zeros(length(d_rel),1);
signed_dist_toC2 = zeros(length(d_rel),1);
signed_dist_toC3 = zeros(length(d_rel),1);

%for each point compute signed distance to each curve in Y-axis

for i =1:1:length(d_rel)
    
    temp_d_rel = d_rel(i);
    temp_v_rel = v_rel(i);
    temp_v_av = v_av(i);
    
    if(isnan(temp_d_rel))
        continue;
    end
    valid_data_count = valid_data_count+1;
    signed_dist_toC1(i) = sign_dist2curve(temp_d_rel,temp_v_rel,1);
    signed_dist_toC2(i)= sign_dist2curve(temp_d_rel,temp_v_rel,2);
    signed_dist_toC3(i) = sign_dist2curve(temp_d_rel,temp_v_rel,3);
    
    if(sign_dist2curve(temp_d_rel,temp_v_rel,3)>=0)
        
        region_index(valid_data_count) = 4;
    elseif(sign_dist2curve(temp_d_rel,temp_v_rel,2) >= 0)
        
        region_index(valid_data_count) = 3;
    elseif(sign_dist2curve(temp_d_rel,temp_v_rel,1) >=0)
        
        region_index(valid_data_count) = 2;
    else
        region_index(valid_data_count) = 1;
    end
    
%     if(sign_dist2curve(temp_d_rel,temp_v_rel,1) < 0)
%         count_unsafe = count_unsafe+1;
%         region_index(i) = 1;
%     elseif(sign_dist2curve(temp_d_rel,temp_v_rel,2)< 0)
%         count_adap_II = count_adap_II+1;
%         region_index(i) = 2;
%     elseif(sign_dist2curve(temp_d_rel,temp_v_rel,3)< 0)
%         count_adap_I = count_adap_I +1 ;
%         region_index(i) = 3;
%     else       
%         count_safe_region = count_safe_region+1;
%         region_index(i) = 4;
%     end


    if(sign_dist2curve_newFS(temp_d_rel,temp_v_rel,temp_v_av,1)<0)
        region_index_newFS(valid_data_count) = 1;
    elseif(sign_dist2curve_newFS(temp_d_rel,temp_v_rel,temp_v_av,2)<0)
        region_index_newFS(valid_data_count) = 2;
    elseif(sign_dist2curve_newFS(temp_d_rel,temp_v_rel,temp_v_av,3)<0)
        region_index_newFS(valid_data_count) = 3;
    else
        region_index_newFS(valid_data_count) = 4;
    end
    
 
end


figure()
C_FS = categorical(region_index,[1 2 3 4],{'S_1','S_2','S_3','S_4'});
h = histogram(C_FS,'BarWidth',0.5,'Normalization','probability');
ylabel('%','FontSize',30)
set(gca,'FontSize',30)
ytix = get(gca, 'YTick');
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
set(gcf, 'Color', 'w');
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
eval(['export_fig ','FS_data_distribution',' -pdf']);



figure()
C_NewFS = categorical(region_index_newFS,[1 2 3 4],{'S_1','S_2','S_3','S_4'});
h = histogram(C_NewFS,'BarWidth',0.5,'Normalization','probability');
ylabel('%','FontSize',30)
set(gca,'FontSize',30)
ytix = get(gca, 'YTick');
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
set(gcf, 'Color', 'w');
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
ylim([0 1])
eval(['export_fig ','NewFS_data_distribution',' -pdf']);

% figure()
% h = histogram([C_FS,C_NewFS],'BarWidth',0.5,'Normalization','probability');
% ylabel('%','FontSize',30)
% set(gca,'FontSize',30)
% ytix = get(gca, 'YTick');
% set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
% set(gcf, 'Color', 'w');




%%

function signed_dist = sign_dist2curve(d_rel,v_rel,j)
%sign_dist = signed distance from the data point to the j-th curve of the FS

a = [1.5,1.0,0.5];
w = [4.5,5.25,6.0];

v_star = min([0,v_rel]);
signed_dist = d_rel - (w(j)+(v_star^2)/(2*a(j)));

end


function signed_dist = sign_dist2curve_newFS(d_rel,v_rel,v_av,j)
%sign_dist = signed distance from the data point to the j-th curve of the FS

a = [1.5,1.0,0.5];
w = [4.5,5.25,6.0];
h = [0.4,1.2,1.8];
v_star = min([0,v_rel]);
signed_dist = d_rel - (w(j)+v_av*h(j)+(v_star^2)/(2*a(j)));

end

