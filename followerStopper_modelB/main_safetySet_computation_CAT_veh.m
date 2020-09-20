%% write a simulator to validate reachability safety set by points
close all 
clear all
clc

%%
d_rel_0 = 10;
v_rel_0 = 5;
v_f_0 = 30;


%% 
uMin = -5.3;
uMax = 3.5;

%% run simulation for a given initial state

d_rel_g = 0:2:50;
v_lead_g = 0:1:30;
v_follower_g = 0:1:30;

total_N = length(d_rel_g)*length(v_lead_g)*length(v_follower_g);

tspan = [0 30];

ming_gap = zeros(size(v_lead_g));
end_rel_spd = zeros(size(v_lead_g));
end_v_f =  zeros(size(v_lead_g));

safetySlice(length(v_follower_g))=struct();

min_gap_matrix = zeros(length(d_rel_g),length(v_lead_g),length(v_follower_g));
lead_veh_end_spd = zeros(length(d_rel_g),length(v_lead_g),length(v_follower_g));
follower_veh_end_spd = zeros(length(d_rel_g),length(v_lead_g),length(v_follower_g));

% params.external_r = 5;
params.external_r = 30;
% params.delay_size = 0.05;
params.delay_size = 0;

% global hist y_hist t_hist delay_size
% temp_hist(length(v_lead_g))=struct();
% hist = temp_hist;
% delay_size = 0.2;

for k = 1:length(v_follower_g)
    v_f_0 = v_follower_g(k); 
    safetyValue = zeros(length(d_rel_g),length(v_lead_g));
    
    for j = 1:1:length(d_rel_g)
        d_rel_0 = d_rel_g(j);
%         y_hist = [];
%         t_hist = [];
%         
%         for i = 1:length(v_lead_g)
%             hist(i).y_hist = [];
%             hist(i).t_hist = [];
%         end
        for i = 1:length(v_lead_g)
            
            v_lead_0 = v_lead_g(i);
            x0 = [d_rel_0;v_lead_0;v_f_0;0;0;0;0]; 
%             [t,y] = ode45(@(t,x) CFM_TwoCars_model(t,x,uMin,uMax,params),tspan,x0);    
%             opts = odeset('RelTol',1e-10,'AbsTol',1e-10);
%             exp_num = i;
%             [t,y] = ode45(@(t,x) CFM_CATVEH_model(t,x,uMin,uMax,params,exp_num),tspan,x0,opts);    
            
            %simulation warm-up
            params.dMin = 0.0;            
            [~,y] = ode45(@(t,x) CFM_CATVEH_model(t,x,uMin,uMax,params,1),[0 3],x0);  
            
            params.dMin = -3.5; 
            x0 = [d_rel_0;v_lead_0;v_f_0;y(end,4);y(end,5);y(end,6);y(end,7)]; 
            [t,y] = ode45(@(t,x) CFM_CATVEH_model(t,x,uMin,uMax,params,1),tspan,x0); 
            d_rel = y(:,1);
            v_lead = y(:,2);
            v_f = y(:,3);

            min_dist = min(d_rel);
            safetyValue(j,i) = min_dist;
            min_gap_matrix(j,i,k) = min_dist;
            lead_veh_end_spd(j,i,k) = v_lead(end);
            follower_veh_end_spd(j,i,k) = v_f(end);
            
%             if(v_lead(end) < 0)
%                 fprintf("warning : lead vehicle speed is less than zero = %f \n",v_lead(end));
%             end
%             if(v_lead(end)>0 || v_f(end) >0)
%                 fprintf("warning : lead vehicle end speed = %f, follower vehicle end speed = %f \n",v_lead(end),v_f(end));
%             end            
        end
    end    
    
%     [XX,YY] = meshgrid(v_lead_g,d_rel_g);
%     figure()
%     contour(XX,YY,safetyValue,[0 0])
%     xlabel('lead vehicle speed [m/s]')
%     ylabel('Gap [m]') 
%     
%     pause
    
    fprintf('%f percent completed \n',100*(length(v_lead_g)*j*k)/total_N);

    safetySlice(k).safetyValue = safetyValue; %minimum distance at each d_rel / v_lead
    safetySlice(k).d_rel_g = d_rel_g;
    safetySlice(k).v_lead_g = v_lead_g;
    safetySlice(k).v_follower = v_f_0;
end

formatOut = 'mm_dd_yy_HHMM';
t_str = datestr(now,formatOut);
IchNameIst = mfilename;
save([IchNameIst,'_',t_str,'.mat']);


%% plot the 3D safety set
[XX,YY,ZZ] = meshgrid(d_rel_g,v_lead_g,v_follower_g);
V = permute(min_gap_matrix,[2 1 3]);

figure_3dsafetySet_handle = figure()
P = patch(isosurface(XX,YY,ZZ,V,0));
isonormals(XX,YY,ZZ,V,P)
P.FaceColor = 'red';
P.EdgeColor = 'none';
daspect([1 1 1])
view(3); 
axis tight
camlight 
lighting gouraud
xlabel('distance[m]');
ylabel('lead speed[m/s]');
zlabel('follower speed[m/s]')
grid on


%% Make rotation gif movie
figure_3dsafetySet_handle = gcf;
% axis off
%     view(0,10)
center = get(gca, 'CameraTarget');
pos = get(gca, 'CameraPosition');
radius = norm(center(1:2) - pos(1:2));
% angles = 0:0.02*pi:2*pi;

angles = 4*pi/3:-0.01*pi:7*pi/6;
angles = [angles,7*pi/6:0.01*pi:9*pi/6];
angles = [angles,9*pi/6:-0.01*pi:8*pi/6];
view(3); 

fname = 'followerStopper_safe_set_3D_toolbox.gif';

for ii=1:length(angles)
   angle = angles(ii);

   set(gca, 'CameraPosition', [center(1) + radius * cos(angle),...
                               center(2) + radius * sin(angle),...
                               pos(3)]);
   drawnow;
%    frame = getframe(1);
   frame =  getframe(figure_3dsafetySet_handle) ;
   im = frame2im(frame);
   [imind,cm] = rgb2ind(im,256);
   if ii == 1
       imwrite(imind,cm,fname,'gif', 'Loopcount',inf);
   else
       imwrite(imind,cm,fname,'gif','WriteMode','append','DelayTime', 0.25);
   end
end

