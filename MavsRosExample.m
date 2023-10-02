function MavsRosExample()
% Initialize ros with a master
rosinit; 

% Add the folder with the interface scripts to the MATLAB path
addpath('scripts\');

% Make sure the MAVS DLL path is loaded
CheckMavsLoaded();

% Run the MAVS simulation with ROS, see function below
RunMavsRosSimulation();

rosshutdown;
end

function RunMavsRosSimulation()
% Run an example MAVS Simulation using ROS
% Create a sensor, scene, camera, and vehicle.
% With the camera window on top, drive the vehicle with the W-A-S-D keys.

% get the full path to mavs-matlab/mavs/data
mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();

% specify the scene to load
scene_to_load = mavs_data_path+"\scenes\cube_scene.json";

% specify the vehicle to load
veh_to_load = mavs_data_path+"\vehicles\rp3d_vehicles\mrzr4_tires_low_gear.json";

% create the scene
scene = MavsScene(scene_to_load);

% create the vehicle
vehicle = MavsVehicleRosNode('mavs_vehicle_node','mavs/odom',veh_to_load, [20,0, 0.0], pi/2.0);

% create the lidar. 
% Options are VLP-16, OS1, OS2, HDL-64E, HDL-32E, RS32, and M8
lidar = MavsLidarRosNode('mavs_lidar_node', 'mavs/points', 'VLP-16');
% set the offset of the sensor from the vehicle CG
% [x,y,z] vector and quaternion [w, x, y, z]
% forward = x, +z=up, right handed system
lidar.SetOffset([0.0, 0.0, 2.0], [1.0, 0.0, 0.0, 0.0]);

% create the camera for driving the vehicle
camera = MavsCamera();
% arguments are: 
% num_horizontal_pixels, num_vertical_pixels, 
% horiz_plane_size (m), vert_plane size (m), focal length (m)
camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
% set the camera offset from the vehicle CG
camera.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
% open the camera display window
camera.Display();

frame_count = 0; % set the frame count to zero
sim_dt = 0.01; % 100 Hz timestep

% start the main simulation loop
% end by closing the camera window
while(camera.IsOpen())
    % get the driving commands from the camera window
    [throttle, steering, braking] = camera.GetDrivingCommand();

    % update the vehicle using the driving commands
    vehicle.Update(scene.id, throttle, steering, braking, sim_dt);
    % publish the odometry to a ROS topic
    vehicle.PublishOdometry('world');

    % update the sensors every 10th step (10 Hz)
    if (mod(frame_count,10)==0)
        % Get the current vehicle pose
        [pos, ori] = vehicle.GetPose();
        camera.SetPose(pos,ori); % Set the camera pose
        camera.Update(scene.id); % Update the camera
        camera.Display(); % Display the camera
        lidar.SetPose(pos,ori); % Set the lidar pose
        lidar.Update(scene.id); % Update the lidar
        lidar.Display(); % Display the lidar
        lidar.PublishPointCloud2('world');
    end

    frame_count = frame_count + 1; % update the frame counter
end % simulation loop

end