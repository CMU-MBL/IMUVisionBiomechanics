%% VisualizeDirectCollocationSimulation
% This script takes output data from the direct collocation simulations
% and creates a video of the motion from output kinematics and contact
% forces against an afterimage representing the ground truth motion.
 
% Initialize drawing area
if(doExperimentalData == 0)
    lim_x = [-1.5 2.5];
    lim_y = [-0.2 2.2];
elseif(doExperimentalData == 1)
    lim_x = [0.5 3.5];
    lim_y = [-0.2 1.7];
end
lim_y = [-0.2 2.2];
figh = figure('MenuBar','none','ToolBar','None', 'DockControls', 'off');
figh.Color = [.95 .95 .95];
figh.Position(3:4) = [1200 420];
figh.OuterPosition(3:4) = [1210, 450];
hold on

drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} )  ;     
RADtoDEG = 180/pi;

for k = 1:length(cycle_viz)
    % Wipe the slate
    clf(figh)
    
    % Setup Figure
    pos2 = get(figh, 'Position'); % gives the position of current plot
    new_pos2 = pos2 + [-.009 0 0.035 0];
    set(figh, 'Position', new_pos2); % set new position of current plot
    hold on
    
    % Plot Model Ground Truth (lighter colors)
    plot([hipjoint_x_cycle(k), proxtorso_x_cycle(k)], [hipjoint_y_cycle(k), proxtorso_y_cycle(k)], 'Color', [0, .2, 0, .5], 'LineWidth',3)
    plot([hipjoint_x_cycle(k), lkneejoint_x_cycle(k)], [hipjoint_y_cycle(k), lkneejoint_y_cycle(k)], 'Color', [0, 0, .2, .5], 'LineWidth',3, 'HandleVisibility', 'off')
    plot([lkneejoint_x_cycle(k), lanklejoint_x_cycle(k)], [lkneejoint_y_cycle(k), lanklejoint_y_cycle(k)], 'Color', [0, 0, .2, .5], 'LineWidth',3, 'HandleVisibility', 'off')
    plot([lanklejoint_x_cycle(k), ldistfoot_x_cycle(k)], [lanklejoint_y_cycle(k), ldistfoot_y_cycle(k)], 'Color', [0, 0, .2, .5], 'LineWidth',3, 'HandleVisibility', 'off')
    plot([hipjoint_x_cycle(k), rkneejoint_x_cycle(k)], [hipjoint_y_cycle(k), rkneejoint_y_cycle(k)], 'Color', [.2, 0, 0, .5], 'LineWidth',3, 'HandleVisibility', 'off')
    plot([rkneejoint_x_cycle(k), ranklejoint_x_cycle(k)], [rkneejoint_y_cycle(k), ranklejoint_y_cycle(k)], 'Color', [.2, 0, 0, .5], 'LineWidth',3, 'HandleVisibility', 'off')
    plot([ranklejoint_x_cycle(k), rdistfoot_x_cycle(k)], [ranklejoint_y_cycle(k), rdistfoot_y_cycle(k)], 'Color', [.2, 0, 0, .5], 'LineWidth',3, 'HandleVisibility', 'off')
    
    % Plot Model State
    plot([hipjoint_x_viz_state(k), proxtorso_x_viz_state(k)], [hipjoint_y_viz_state(k), proxtorso_y_viz_state(k)], 'g', 'LineWidth',3)
    plot([hipjoint_x_viz_state(k), lkneejoint_x_viz_state(k)], [hipjoint_y_viz_state(k), lkneejoint_y_viz_state(k)], 'b', 'LineWidth',3, 'HandleVisibility', 'off')
    plot([lkneejoint_x_viz_state(k), lanklejoint_x_viz_state(k)], [lkneejoint_y_viz_state(k), lanklejoint_y_viz_state(k)], 'b', 'LineWidth',3, 'HandleVisibility', 'off')
    plot([lanklejoint_x_viz_state(k), ldistfoot_x_viz_state(k)], [lanklejoint_y_viz_state(k), ldistfoot_y_viz_state(k)], 'b', 'LineWidth',3, 'HandleVisibility', 'off')
    plot([hipjoint_x_viz_state(k), rkneejoint_x_viz_state(k)], [hipjoint_y_viz_state(k), rkneejoint_y_viz_state(k)], 'r', 'LineWidth',3, 'HandleVisibility', 'off')
    plot([rkneejoint_x_viz_state(k), ranklejoint_x_viz_state(k)], [rkneejoint_y_viz_state(k), ranklejoint_y_viz_state(k)], 'r', 'LineWidth',3, 'HandleVisibility', 'off')
    plot([ranklejoint_x_viz_state(k), rdistfoot_x_viz_state(k)], [ranklejoint_y_viz_state(k), rdistfoot_y_viz_state(k)], 'r', 'LineWidth',3, 'HandleVisibility', 'off')
    
    % Plot Revolute Joints
    plot(hipjoint_x_viz_state(k), hipjoint_y_viz_state(k), 'k.', 'MarkerSize', 40)
    plot(rkneejoint_x_viz_state(k), rkneejoint_y_viz_state(k), 'k.', 'MarkerSize', 40, 'HandleVisibility', 'off')
    plot(ranklejoint_x_viz_state(k), ranklejoint_y_viz_state(k), 'k.', 'MarkerSize', 40, 'HandleVisibility', 'off')
    plot(lkneejoint_x_viz_state(k), lkneejoint_y_viz_state(k), 'k.', 'MarkerSize', 40, 'HandleVisibility', 'off')
    plot(lanklejoint_x_viz_state(k), lanklejoint_y_viz_state(k), 'k.', 'MarkerSize', 40, 'HandleVisibility', 'off')
    
    % Plot Artificial IMU Squares
    plot(torso_x_viz_state(k), torso_y_viz_state(k), 'ks', 'MarkerSize', 10)
    plot(lthigh_x_viz_state(k), lthigh_y_viz_state(k), 'ks', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(rthigh_x_viz_state(k), rthigh_y_viz_state(k), 'ks', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(lshank_x_viz_state(k), lshank_y_viz_state(k), 'ks', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(rshank_x_viz_state(k), rshank_y_viz_state(k), 'ks', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(lfoot_x_viz_state(k), lfoot_y_viz_state(k), 'ks', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(rfoot_x_viz_state(k), rfoot_y_viz_state(k), 'ks', 'MarkerSize', 10, 'HandleVisibility', 'off')
    
    % Plot Vision Keypoint Locations
    plot(proxtorso_x_viz_state(k), proxtorso_y_viz_state(k), 'm.', 'MarkerSize', 10)
    plot(hipjoint_x_viz_state(k), hipjoint_y_viz_state(k), 'm.', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(lkneejoint_x_viz_state(k), lkneejoint_y_viz_state(k), 'm.', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(rkneejoint_x_viz_state(k), rkneejoint_y_viz_state(k), 'm.', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(lanklejoint_x_viz_state(k), lanklejoint_y_viz_state(k), 'm.', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(ranklejoint_x_viz_state(k), ranklejoint_y_viz_state(k), 'm.', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(ldistfoot_x_viz_state(k), ldistfoot_y_viz_state(k), 'm.', 'MarkerSize', 10, 'HandleVisibility', 'off')
    plot(rdistfoot_x_viz_state(k), rdistfoot_y_viz_state(k), 'm.', 'MarkerSize', 10, 'HandleVisibility', 'off')
    
    % Plot Ground Reaction Force Vectors
    xl_arrow = [lcontact_x_state(k) - grflx_viz(k)/(4*max(abs(grflx_viz) + 1) ), lcontact_x_state(k)];
    yl_arrow = [lcontact_y_state(k) - grfly_viz(k)/(4*max(abs(grfly_viz) + 1) ), lcontact_y_state(k)];
    drawArrow(xl_arrow, yl_arrow, 'linewidth',3,'MaxHeadSize',0.8, 'color', 'k')
    
    xr_arrow = [rcontact_x_state(k) - grfrx_viz(k)/(4*max(abs(grfrx_viz) + 1) ), rcontact_x_state(k)];
    yr_arrow = [rcontact_y_state(k) - grfry_viz(k)/(4*max(abs(grfry_viz) + 1) ), rcontact_y_state(k)];
    drawArrow(xr_arrow, yr_arrow, 'linewidth',3,'MaxHeadSize',0.8, 'color', 'k', 'HandleVisibility', 'off')
    
    % Edit Plot
    xlabel('X - Ground Frame')
    ylabel('Y - Ground Frame')
    grid on
    set(gca,'Box','off')
    ylim(lim_y)
    xlim(lim_x)
    legend('Ground Truth', 'Model State', 'Revolute Joints', 'IMUs', 'Vision Keypoints', 'Ground Reaction Forces')
    hold off
        
    % Get the movie
    movieVector(k) = getframe(figh, [0 0 1200 400]);
end

% Save the Movie
myWriter = VideoWriter('simulation','MPEG-4');
myWriter.FrameRate = 30;

open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);
