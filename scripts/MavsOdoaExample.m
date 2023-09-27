function MavsOdoaExample()
    CheckMavsLoaded();
    % set the MAVS scene and vehicle to load
    mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
    scene_to_load = mavs_data_path+"/scenes/single_tree.json";
    veh_to_load = mavs_data_path+"/vehicles/rp3d_vehicles/mrzr4_tires_low_gear.json";

    % control parameters
    desired_speed = 7.5;
    look_ahead_dist = 1.5*desired_speed;

    % perception parameters
    map_res = 2; % per meters
    scan_range_limits=[8.0,60.0]; % set the range of lidar points to use
    max_ray_range = scan_range_limits(2)-1;  % set the max range of points to put into scan

    % map parameters
    % dimensions of the scene [[llx, urx], [lly, ury]];
    scene_limits = [[-40.0,40.0]; [-20.0, 20.0]];
    goal_pose = [scene_limits(1,2)-5 0.0 0.0]; % goal pose in local enu
    goal_radius = 3.0;
    waypoints = [goal_pose(1), goal_pose(2)];

    % create the vehicle
    vehicle = MavsVehicle(veh_to_load,[scene_limits(1,1)+5.0, 0.0], 0.0);

    % load the scene
    scene = MavsScene(scene_to_load);

    % create a lidar
    lidar = MavsLidar('OS2');
    lidar.SetOffset([0.0, 0.0, 1.25], [1.0, 0.0, 0.0, 0.0]);
   
    % create the camera
    camera = MavsCamera();
    camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
    camera.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);

    % create a map
    map_2d = binaryOccupancyMap(scene_limits(1,2)-scene_limits(1,1),...
        scene_limits(2,2)-scene_limits(2,1),map_res);
    map_2d.GridLocationInWorld = [scene_limits(1,1), scene_limits(2,1)];

    % create a state space for the planner
    % see: https://www.mathworks.com/help/nav/ref/plannerhybridastar.html
    ss = stateSpaceSE2;
    ss.StateBounds = [map_2d.XWorldLimits;map_2d.YWorldLimits;[-pi pi]];
    sv = validatorOccupancyMap(ss);

    % create a planner
    planner = plannerHybridAStar(sv, MinTurningRadius=6.5, MotionPrimitiveLength=10.0);

    % create a controller
    ppc = controllerPurePursuit;

    % set the initial throttle, steering, and braking
    throttle = 0.0;
    steering = 0.0;
    braking = 0.0;

    % create a figure window for plotting results
    h=figure;
    h.Position = [100 100 1024 512];
    lidar_plot_h = subplot(2,2,3:4);
    max_sim_time = 120.0; % seconds
    movie_frames = cell((max_sim_time+1)*20);

    % do the check for the distance to goal
    [p,~]=vehicle.GetPose();
    dist_to_goal = norm([p(1),p(2)]-[goal_pose(1),goal_pose(2)]);

    % start the main simulation loop
    frame_count = 0;
    elapsed_time = 0.0;
    num_imgs = 1;
    sim_dt = 1.0/100.0;
    while(ishandle(h) && p(1)<scene_limits(1,2)-1 && elapsed_time<max_sim_time && dist_to_goal>goal_radius)
        
        % update the vehicle and render the camera at 100 Hz
        vehicle.Update(scene.id, throttle, steering, braking, sim_dt);
        [p,q, veh_vel, ~] = vehicle.GetState();
        dist_to_goal = norm([p(1),p(2)]-[goal_pose(1),goal_pose(2)]);
        vspeed = sqrt(veh_vel(1)*veh_vel(1)+ veh_vel(2)*veh_vel(2));
            
        %update the camera at 20 Hz
        if (mod(frame_count,5)==0)
            camera.SetPose(p,q);
            camera.Update(scene.id);
            img = camera.GetImage();
            subplot(2,2,1);
            image(img);
            drawnow;
            if (ishandle(h))
                movie_frames{num_imgs} = getframe(h);
                num_imgs = num_imgs+1;
            end
        end

        % update the lidar at 10 Hz
        if (mod(frame_count,10)==0)
            % set the pose of the sensor
            lidar.SetPose(p,q);
        
            % do a scan and get the points
            lidar.Update(scene.id);
            xyz_points = lidar.GetPoints(true);
        
            % convert the points to a matlab point cloud object
            pc_cloud = pointCloud(xyz_points');

            % register the point cloud and plot it
            if (ishandle(h))
                rot_mat = quat2rotm(q);
                tform = rigidtform3d(rot_mat,p);
                pc_reg = pctransform(pc_cloud,tform);
                subplot(2,2,3:4);
                scatter3(pc_reg.Location(:,1),pc_reg.Location(:,2),pc_reg.Location(:,3),4,pc_reg.Location(:,3),'filled');
                xlim(lidar_plot_h,[scene_limits(1,1), scene_limits(1,2)]);
                ylim(lidar_plot_h,[scene_limits(2,1), scene_limits(2,2)]);
            end

            % convert the cloud to a scan
            params = lidarParameters('OS2-64',1024);
            pc_org = pcorganize(pc_cloud,params);
            groundPtsIdx = segmentGroundFromLidarData(pc_org);
            nonGroundPtCloud= select(pc_org,~groundPtsIdx,OutputSize="full");
            scan = pc2scan(nonGroundPtCloud, ScanRangeLimits=scan_range_limits); 

            % insert the scan into the 2d map
            [yaw, ~, ~] = quat2angle(q);
            current_pose = [p(1),p(2),yaw];
            insertRay(map_2d,current_pose,scan,max_ray_range);
    
            % Update the planner at 1 Hz
            if (mod(frame_count,100)==0)
            %if (frame_count==0)
                % run the path planner on the current map
                sv.Map = map_2d;
                startPose = [double(p(1)) double(p(2)) yaw]; % [meters, meters, radians]
                refpath = plan(planner,startPose,goal_pose,SearchMode='exhaustive');
                waypoints = refpath.States(:,1:2);
                if (ishandle(h))
                    subplot(2,2,2);
                    show(planner)
                    drawnow 
                end
            end
        end

        % run the planner at 100 Hz
        ppc.Waypoints = waypoints;
        ppc.DesiredLinearVelocity = desired_speed;
        ppc.LookaheadDistance = look_ahead_dist;
        [goal_speed,~, desired_pos] = ppc(current_pose);
        [throttle, steering] = GetDrivingCommand(throttle, steering, ...
            vspeed, goal_speed, current_pose, desired_pos);

        % update the frame counter
        frame_count = frame_count + 1;
        elapsed_time = elapsed_time + sim_dt;
    end

    % Save the frames to a video file
    movie_frames((num_imgs-1):end) = [];
    video_writer_obj = VideoWriter('mavs_matlab_playback','MPEG-4');
    video_writer_obj.FrameRate = 20;
    open(video_writer_obj);
    for i=1:length(movie_frames)
        
        writeVideo(video_writer_obj, movie_frames{i});
    end
    close(video_writer_obj);
    if (ishandle(h))
        close(h);
    end
end

function [throttle, steering] = GetDrivingCommand(current_throttle, current_steering, ...
    current_speed, desired_speed, current_pose, desired_pos)
% this a function that adapts the output of the pure pursuit controller
% to throttle and steering commands
    throttle_step = 0.01;
    steering_step = 0.0025;
    max_steering_angle = deg2rad(22.5);
    vehicle_wheelbase = 3.0;

    % do the throttle control
    if (desired_speed>current_speed)
        throttle = current_throttle + throttle_step;
    else
        throttle = current_throttle - throttle_step;
    end
    throttle = min([1.0,max([throttle, 0.0])]);

    % do the steering
    v = desired_pos - current_pose(1:2);
    desired_yaw = atan2(v(2),v(1));
    goal_steering = ((desired_yaw-current_pose(3))/max_steering_angle)/vehicle_wheelbase;
    steering = 0.0;
    if (goal_steering>current_steering)
        steering = current_steering + steering_step;
    elseif (goal_steering<current_steering)
        steering = current_steering - steering_step;
    end
    steering = min([1.0,max([steering, -1.0])]);
end
