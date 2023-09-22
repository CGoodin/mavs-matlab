function lidar_occupancy_map_example()

    % define the scene to load
    mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
    %scene_to_load = mavs_data_path+"/scenes/cube_scene.json";
    %scene_to_load = mavs_data_path+"/scenes/odoa_scene_with_trees.json";
    scene_to_load = mavs_data_path+"/scenes/single_tree.json";

    % load the scene
    scene = MavsScene(scene_to_load);

    % create a lidar
    lidar = MavsLidar('OS2');
   
    % set the initial sensor position and orientation
    p = [-100.0, 0.0, 2.0]; % position
    q = [1.0, 0.0, 0.0, 0.0]; % orientation

    % create a map
    map_res = 2; % per meters
    map_3d = occupancyMap3D(map_res);

    map_2d = binaryOccupancyMap(200.0,200.0,map_res);
    map_2d.GridLocationInWorld = [-100.0, -100.0];

    % see: https://www.mathworks.com/help/nav/ref/plannerhybridastar.html
    ss = stateSpaceSE2;
    ss.StateBounds = [map_2d.XWorldLimits;map_2d.YWorldLimits;[-pi pi]];
    sv = validatorOccupancyMap(ss);

    % Specify limits for the player
    xlimits = [-100 100]; % meters
    ylimits = [-100 100];
    zlimits = [-1 10];

    % Create a pcplayer to visualize streaming point clouds from lidar sensor
    lidarPlayer = pcplayer(xlimits,ylimits,zlimits);
    
    % Customize player axes labels
    xlabel(lidarPlayer.Axes,'X (m)');
    ylabel(lidarPlayer.Axes,'Y (m)');
    zlabel(lidarPlayer.Axes,'Z (m)');
    title(lidarPlayer.Axes,'Lidar Sensor Data');

    % move the sensor through the world and update the map as we go
    while p(1)<100.0
        % set the pose of the sensor
        lidar.SetPose(p,q);
    
        % do a scan and get the points
        lidar.Update(scene.id);
        xyz_points = lidar.GetPoints(true);
    
        % convert the points to a matlab point cloud object
        pc_cloud = pointCloud(xyz_points');

        % create the transofrm and register the point cloud
        rot_mat = quat2rotm(q);
        tform = rigidtform3d(rot_mat,p);
        pc_reg = pctransform(pc_cloud,tform);

        % convert the cloud to a scan
        params = lidarParameters('OS2-64',1024);
        pc_org = pcorganize(pc_cloud,params);
        groundPtsIdx = segmentGroundFromLidarData(pc_org);
        nonGroundPtCloud= select(pc_org,~groundPtsIdx,OutputSize="full");
        scan = pc2scan(nonGroundPtCloud, ScanRangeLimits=[10,60]); %pc2scan(nonGroundPtCloud,tform);
        plot(scan)

        max_range = 55.0;

        % insert the scan into the 2d map
        [yaw, ~, ~] = quat2angle(q);
        insertRay(map_2d,[p(1),p(2),yaw],scan,max_range);

        % add the point cloud to the 3d map
        insertPointCloud(map_3d,[p q],pc_cloud.Location,max_range);

        % run the path planner on the curretn map
        sv.Map = map_2d;
        planner = plannerHybridAStar(sv, MinTurningRadius=4, MotionPrimitiveLength=6);
        startPose = [p(1) p(2) 0.0]; % [meters, meters, radians]
        goalPose = [100.0 0.0 0.0];
        refpath = plan(planner,startPose,goalPose,SearchMode='exhaustive');
        show(planner)

        % Update lidar display
        view(lidarPlayer,pc_reg.Location);

        % move the sensor
        p(1) = p(1) + 2.5;
    end

    % display the map
    show(map_3d)
    figure;
    show(map_2d)

end


%for pc_idx=1:num_clusters
%    pc1 = select(pc_org,find(labels == pc_idx));
%    pcshow(pc1.Location,'g')
%end
%params = lidarParameters('OS2-64',1024);
%pc_org = pcorganize(pc_reg,params);
%[labels, num_clusters] = segmentLidarData(pc_org,2.0);
%num_clusters
%groundPtsIdx = segmentGroundFromLidarData(pc_org);
%groundPtCloud = select(pc_org,groundPtsIdx);
%nonGroundPtCloud = select(pc_org,~groundPtsIdx,'OutputSize','full');
