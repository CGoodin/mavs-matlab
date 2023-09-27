function MavsDrivingExample()
    CheckMavsLoaded();
    % define the scene and vehicle to load
    mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
    scene_to_load = mavs_data_path+"\scenes\brownfield_scene.json";
    veh_to_load = mavs_data_path+"\vehicles\rp3d_vehicles\mrzr4_tires_low_gear.json";
    
    % load the scene
    scene = MavsScene(scene_to_load);

    % load the vehicle
    vehicle = MavsVehicle(veh_to_load, [20,0, 0.0], pi/2.0);

    % create the lidar
    lidar = MavsLidar('VLP-16');
    lidar.SetOffset([0.0, 0.0, 2.0], [1.0, 0.0, 0.0, 0.0]);
    
    % create the camera
    camera = MavsCamera();
    camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
    camera.Display();
    camera.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);

    frame_count = 0;
    sim_dt = 0.01;
    while(camera.IsOpen())
        [throttle, steering, braking] = camera.GetDrivingCommand();
        vehicle.Update(scene.id, throttle, steering, braking, sim_dt);
        if (mod(frame_count,10)==0)
            [pos, ori] = vehicle.GetPose();
            camera.SetPose(pos,ori);
            camera.Update(scene.id);
            camera.Display();
            lidar.SetPose(pos,ori);
            lidar.Update(scene.id);
            lidar.Display();
        end
        frame_count = frame_count + 1;
    end

end
