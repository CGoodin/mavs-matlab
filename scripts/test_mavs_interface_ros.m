function test_mavs_interface_ros()
    rosinit
    CheckMavsLoaded();
    % define the scene and vehicle to load
    mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
    scene_to_load = mavs_data_path+"/scenes/single_tree.json";
    veh_to_load = mavs_data_path+"/vehicles/rp3d_vehicles/mrzr4_tires_low_gear.json";
    
    % load the scene
    scene = MavsScene(scene_to_load);

    % load the vehicle
    vehicle_node = MavsVehicleRosNode('mavs_vehicle_node','mavs/odometry',veh_to_load);

    % create the lidar
    lidar_node = MavsLidarRosNode('mavs_lidar_node','mavs/point_cloud2','OS1');
    lidar_node.SetOffset([0.0, 0.0, 2.0], [1.0, 0.0, 0.0, 0.0]);
    
    % create the camera
    camera = MavsCamera();
    camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
    camera.Display();
    camera.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);

    frame_count = 0;
    sim_dt = 0.01;
    while(camera.IsOpen())
        [throttle, steering, braking] = camera.GetDrivingCommand();
        vehicle_node.Update(scene.id, throttle, steering, braking, sim_dt);
        vehicle_node.PublishOdometry('odom');
        if (mod(frame_count,10)==0)
            [pos, ori] = vehicle_node.GetPose();
            camera.SetPose(pos,ori);
            camera.Update(scene.id);
            camera.Display();

            lidar_node.Update(scene.id);
            lidar_node.Display();
            lidar_node.PublishPointCloud2('map');
        end
        frame_count = frame_count + 1;
    end
    rosshutdown
    
end
