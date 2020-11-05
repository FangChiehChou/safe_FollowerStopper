
%%

TwoCarAnimation_handle = figure()

sfh2 = subplot(2,2,3);
plot(test_data_time-test_data_time(1),test_data_d_rel,'LineWidth',2,'Color',[0.9290, 0.6940, 0.1250])
hold on
plot(t,d_rel,'-.','LineWidth',2)
% plot(t_no_delay,d_rel_no_delay,'-.','LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('Relative position[m]','FontSize',30)
set(gca,'FontSize',30)
legend('human driver','FollowerStopper')
xlim([0 t(end)])
y_axis_max = 45;
ylim([0 y_axis_max])
progress_cursor1 = plot([0,0],[0,y_axis_max],'LineWidth',2,'Color','black');
grid on
set(get(get(progress_cursor1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

sfh3 = subplot(2,2,4);
plot(data_time,data_lead_spd,'LineWidth',2)
hold on
plot(data_time,test_data_v_f,'LineWidth',2,'Color',[0.9290, 0.6940, 0.1250])
plot(t,v_f,'-.','LineWidth',2,'Color',[0.8500 0.3250 0.0980])
plot(t,v_cmd,'LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('Speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
legend('leader','human driver','FollowerStopper','followerStopper_{cmd}')
xlim([0 t(end)])
progress_cursor2 = plot([0,0],[0,30],'LineWidth',2,'Color','black');
grid on
set(get(get(progress_cursor2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%%
% sim_time = test_data_time-test_data_time(1);
% 
% test_data_d_rel  Vq = interp1(X,V,Xq,METHOD,EXTRAPVAL)

animation_time = t(1):0.1:t(end);
anim_FS_d_rel = interp1(t,d_rel,animation_time);
animation_FS_v_rel = interp1(t,v_rel,animation_time);
animation_FS_v_f = interp1(t,v_f,animation_time);
animation_FS_v_lead = interp1(data_time,data_lead_spd,animation_time);

sim_v_human_follower =  interp1(data_time,test_data_v_f,animation_time); % test_data_v_f;   %data_time,test_data_v_f
animation_human_d_rel = interp1(data_time,test_data_d_rel,animation_time);

sim_FS_x_follower = zeros(length(animation_time),1);

sim_FS_x_follower(1) = 0;
sim_x_human_follower(1) =0;
for i =2:1:length(animation_time)
    temp_dt = animation_time(i) - animation_time(i-1);
    sim_FS_x_follower(i) = sim_FS_x_follower(i-1) + animation_FS_v_f(i)*temp_dt;
    
end



%%
sfh1 =subplot(2,2,[1 2]);

%% this scipt is for animation of the results of the simulation

leftLane_dash_lines.xx = -20:10:3000; 
leftLane_dash_lines.yy = 2.5*ones(length(leftLane_dash_lines.xx));

rightLane_dash_lines.xx = -20:10:3000; 
rightLane_dash_lines.yy = -2.5*ones(length(leftLane_dash_lines.xx));


hold on
%plot road boundaries
plot(leftLane_dash_lines.xx,leftLane_dash_lines.yy,'--','Color',[0 0 0],'LineWidth',2);
hold on
plot(rightLane_dash_lines.xx,rightLane_dash_lines.yy,'--','Color',[0 0 0],'LineWidth',2);
xlabel('$[m]$','interpreter','latex','FontSize',30)
% ylabel('$[m]$','interpreter','latex','FontSize',30)
set(gca,'FontSize',30)
yticks([-50 0 50]);
grid on
xlim([anim_FS_d_rel(1)-20 anim_FS_d_rel(1)+20]);


%% adjust size of subplots
set(gcf, 'Units', 'centimeters', 'OuterPosition', [0.2117 2.2225 50 25]);
set(sfh1, 'Units', 'centimeters');
set(sfh2, 'Units', 'centimeters');
set(sfh3, 'Units', 'centimeters');
pause(1)

plot_vertial_alternation = sfh1.Position(4)/2;
sfh1.Position = sfh1.Position + [0 plot_vertial_alternation 0 -plot_vertial_alternation];

sfh2.Position = sfh2.Position + [0 1 0 plot_vertial_alternation-1];
sfh3.Position = sfh3.Position + [0 1 0 plot_vertial_alternation-1];

%%  Plot vehicls
axes(sfh1)
hold on
Veh_Handle = [];
vehLength = 4.5;
vehWidth = 3;

leadVeh.x0 = 0+vehLength;
leadVeh.y0 = 0;
followerVeh.x0 = 0;
followerVeh.y0 = 0;

LeadVeh_Vertex = [leadVeh.x0,-(vehWidth/2);...
              leadVeh.x0,+(vehWidth/2);...  
              leadVeh.x0+vehLength,(vehWidth/2);...
              leadVeh.x0+vehLength,-(vehWidth/2)]; 
          
FollowerVeh_Vertex = [followerVeh.x0,-(vehWidth/2);...
                followerVeh.x0,+(vehWidth/2);...  
                followerVeh.x0+vehLength,(vehWidth/2);...
                followerVeh.x0+vehLength,-(vehWidth/2)];  
            
            
          
h = fill(LeadVeh_Vertex(:,1),LeadVeh_Vertex(:,2),[0 0.4470 0.7410]);                   
% h(1) = plot([LeadVeh_Vertex(1,1),LeadVeh_Vertex(2,1)],[LeadVeh_Vertex(1,2),LeadVeh_Vertex(2,2)],'Color',[0 0.4470 0.7410]);
% h(2) = plot([LeadVeh_Vertex(2,1),LeadVeh_Vertex(3,1)],[LeadVeh_Vertex(2,2),LeadVeh_Vertex(3,2)],'Color',[0 0.4470 0.7410]);
% h(3) = plot([LeadVeh_Vertex(3,1),LeadVeh_Vertex(4,1)],[LeadVeh_Vertex(3,2),LeadVeh_Vertex(4,2)],'Color',[0 0.4470 0.7410]);
% h(4) = plot([LeadVeh_Vertex(4,1),LeadVeh_Vertex(1,1)],[LeadVeh_Vertex(4,2),LeadVeh_Vertex(1,2)],'Color',[0 0.4470 0.7410]);
% h(5) = text(RR*cos(theta0(1)),RR*sin(theta0(1))+2,num2str(i-1));
t_temp = hgtransform('Parent',gca);
set(h,'Parent',t_temp)
%     Veh_Handle = t1;
    
%     t_temp = hgtransform('Parent',gca);
h_temp = copyobj(h,t_temp);
Veh_Handle =[Veh_Handle;t_temp]; 


% h = fill(FollowerVeh_Vertex(:,1),FollowerVeh_Vertex(:,2),[0.8500 0.3250 0.0980]);     
h(1) = plot([FollowerVeh_Vertex(1,1),FollowerVeh_Vertex(2,1)],[FollowerVeh_Vertex(1,2),FollowerVeh_Vertex(2,2)],'Color',[0.8500 0.3250 0.0980],'LineWidth',2);
h(2) = plot([FollowerVeh_Vertex(2,1),FollowerVeh_Vertex(3,1)],[FollowerVeh_Vertex(2,2),FollowerVeh_Vertex(3,2)],'Color',[0.8500 0.3250 0.0980],'LineWidth',2);
h(3) = plot([FollowerVeh_Vertex(3,1),FollowerVeh_Vertex(4,1)],[FollowerVeh_Vertex(3,2),FollowerVeh_Vertex(4,2)],'Color',[0.8500 0.3250 0.0980],'LineWidth',2);
h(4) = plot([FollowerVeh_Vertex(4,1),FollowerVeh_Vertex(1,1)],[FollowerVeh_Vertex(4,2),FollowerVeh_Vertex(1,2)],'Color',[0.8500 0.3250 0.0980],'LineWidth',2);
t_temp = hgtransform('Parent',gca);
set(h,'Parent',t_temp)
h_temp = copyobj(h,t_temp);
Veh_Handle =[Veh_Handle;t_temp]; 


h = fill(FollowerVeh_Vertex(:,1),FollowerVeh_Vertex(:,2),[0.9290, 0.6940, 0.1250]);     
% h(1) = plot([FollowerVeh_Vertex(1,1),FollowerVeh_Vertex(2,1)],[FollowerVeh_Vertex(1,2),FollowerVeh_Vertex(2,2)],'Color',[0.8500 0.3250 0.0980]);
% h(2) = plot([FollowerVeh_Vertex(2,1),FollowerVeh_Vertex(3,1)],[FollowerVeh_Vertex(2,2),FollowerVeh_Vertex(3,2)],'Color',[0.8500 0.3250 0.0980]);
% h(3) = plot([FollowerVeh_Vertex(3,1),FollowerVeh_Vertex(4,1)],[FollowerVeh_Vertex(3,2),FollowerVeh_Vertex(4,2)],'Color',[0.8500 0.3250 0.0980]);
% h(4) = plot([FollowerVeh_Vertex(4,1),FollowerVeh_Vertex(1,1)],[FollowerVeh_Vertex(4,2),FollowerVeh_Vertex(1,2)],'Color',[0.8500 0.3250 0.0980]);
t_temp = hgtransform('Parent',gca);
set(h,'Parent',t_temp)
h_temp = copyobj(h,t_temp);
Veh_Handle =[Veh_Handle;t_temp]; 



% tempVehHandle = Veh_Handle(1); %plot handle for the lead vehicle
% translation_temp = makehgtform('translate',[20,0,0]);
% t_temp = tempVehHandle;
% set(t_temp,'Matrix',translation_temp)


drawnow

%%

fname = 'FollowerStopperAnimation_lowSpeed.gif';

camview_center = 21;
for frame_index = 1:1:length(animation_time)
    
    temp_relDist = anim_FS_d_rel(frame_index);
    temp_relSpd = animation_FS_v_rel(frame_index);
    temp_Spd = animation_FS_v_f(frame_index);
    
    temp_x_follower = sim_FS_x_follower(frame_index);
    temp_x_follower_human = temp_x_follower+ temp_relDist-animation_human_d_rel(frame_index);
    
    tempVehHandle = Veh_Handle(1); %plot handle for the lead vehicle
    translation_temp = makehgtform('translate',[temp_x_follower+temp_relDist,0,0]);
    t_temp = tempVehHandle;
    set(t_temp,'Matrix',translation_temp)
    

    tempVehHandle = Veh_Handle(2);  %plot handle for follower vehicle - FollowerStopper
    translation_temp = makehgtform('translate',[temp_x_follower,0,0]);
    t_temp = tempVehHandle;
    set(t_temp,'Matrix',translation_temp)
    
    tempVehHandle = Veh_Handle(3);  %plot handle for follower vehicle - human driver
    translation_temp = makehgtform('translate',[temp_x_follower_human,0,0]);
    t_temp = tempVehHandle;
    set(t_temp,'Matrix',translation_temp)
    
    
    camview_center = temp_x_follower + temp_relDist/2;
%   xLim([camview_center-20 camview_center+20]);
    sfh1.XLim = [camview_center-30 camview_center+30];
%     xlim([0 100]);
    
    progress_cursor1.XData = [animation_time(frame_index) animation_time(frame_index)];
    progress_cursor2.XData = [animation_time(frame_index) animation_time(frame_index)];

    drawnow
%     pause

    frame =  getframe(TwoCarAnimation_handle) ;
    MakeGIF(fname,frame,frame_index,0.01)
    
    
end
