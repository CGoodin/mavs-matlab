function MavsMobilityDemo()
% Function to calculate the mobility properties of a MRZRD4 in MAVS sim.
% Calculates the VCI1, a measure of the softest soil the vehicle can
% traverse, in PSI as measured by a cone penetrometer.
% Calculates the maximum speed of the vehicle on flat pavement.
% Finally, calculates the fuel use as a function of speed and soil
% strength, using throttle setting as a proxy for fuel use.

close all;

% Set the options of the test
mobility_test_vehicle = "\vehicles\rp3d_vehicles\mrzr4_tires_low_gear.json";
mobility_test_scene = "\scenes\surface_only.json";
% set to true to see video of tests, will slow sim down a little
display_images = false; 

% Add the folder with the MAVS interface scripts to the MATLAB path
addpath('scripts\');

% Make sure the MAVS DLL path is loaded
CheckMavsLoaded();

% Get the max speed of the vehicle on pavement
max_speed = GetMaxSpeed(display_images, mobility_test_scene, mobility_test_vehicle);
sprintf('Max speed (m/s) = %f',max_speed)

% Get the VCI1 of the vehicle
vci1 = GetVci1(display_images, mobility_test_scene, mobility_test_vehicle);
sprintf('VCI1 = %f',vci1)

% Calculate the fuel use versus soil strength for different speeds
rci_data = SpeedVsRci(display_images, mobility_test_scene, mobility_test_vehicle);

% create a structure and write the results to a json file
mobility_data.vci1 = vci1;
mobility_data.max_speed = max_speed; 
mobility_data.soil_setspeed_throttle_truespeed = rci_data;
% encode structre as a json and save the results
out_txt = jsonencode(mobility_data,PrettyPrint=true);
fid = fopen('mobility_sim_results.json','w');
fprintf(fid, '%s',out_txt);
fclose(fid);

end % main function, MavsMobilityDemo

function output_data = SpeedVsRci(display_camera, mobility_test_scene, mobility_test_vehicle)
% Function to calculate the relative fuel use and average speed of as a
% function of the soil RCI and commanded speed. The output is a Nx4 array
% where the columns are:
% [soil_strength, commanded_speed, relative_fuel_use, actual speed] 

figure(1);
figure(2);

soil_strengths = 20:2.5:45.0;
desired_speeds = 4.0:2.0:12.0;
output_data = zeros(length(soil_strengths)*length(desired_speeds),4);
for i=1:length(desired_speeds)
    fuel_use = zeros(1,length(soil_strengths));
    actual_speed = zeros(1,length(soil_strengths));
    for j=1:length(soil_strengths)
        [cumulative_throttle, avg_speed] = ...
            SpeedControlTest(desired_speeds(i), soil_strengths(j), display_camera,...
            mobility_test_scene, mobility_test_vehicle);
        fuel_use(j) = cumulative_throttle;
        actual_speed(j) = avg_speed;
        output_data((i-1)*length(soil_strengths)+j,:) = ...
            [soil_strengths(j),desired_speeds(i),cumulative_throttle,avg_speed];
    end
    speed_string = string(desired_speeds(i)).append(" m/s");
    figure(1);
    plot(soil_strengths,fuel_use,'.-','DisplayName',speed_string);
    xlabel('Soil Strength (PSI)');
    ylabel('Relative Fuel Use');
    legend;
    hold on
    figure(2);
    plot(soil_strengths,actual_speed,'.-','DisplayName',speed_string);
    xlabel('Soil Strength (PSI)');
    ylabel('Actual Speed (m/s)');
    legend;
    hold on
end % loop over desired speeds
figure(1);
saveas(gcf,'fuel_use_vs_soil_strength.png');
figure(2)
saveas(gcf,'speed_vs_soil_strength.png');
end % speed vs rci function

function vci1 = GetVci1(display_camera, mobility_test_scene, mobility_test_vehicle)
% Function to calculate the VCI1 of the vehicle. Returns the one-pass
% vehicle cone index, in PSI.

% load the scene and vehicle
mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
scene_to_load = mavs_data_path+mobility_test_scene;
veh_to_load = mavs_data_path+mobility_test_vehicle;
scene = MavsScene(scene_to_load);
vehicle = MavsVehicle(veh_to_load, [0.0, 0.0, 0.0], 0.0);

% create the camera
camera = MavsCamera();
camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
camera.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);

% initialize some sim variables
soil_strength_psi = 100.0;
sim_dt = 0.01; % 100 Hz timestep
frame_count = 0; % set the frame count to zero
sim_time = 0.0;
speed = 0.0;

% simulation loop
while(speed>0.1 || sim_time<10.0)
    % get the current speed of the vehicle
    [~,~,lin_vel, ~] = vehicle.GetState();
    speed = sqrt(lin_vel(1)*lin_vel(1)+lin_vel(2)*lin_vel(2));

    % update the vehicle, use max throttle
    vehicle.Update(scene.id, 1.0, 0.0, 0.0, sim_dt);

    % set the soil properties
    scene.SetSoilProperties('clay',soil_strength_psi);

    % Display the camera at 10 Hz
    if (mod(frame_count,10)==0 && frame_count>0 && display_camera)
        [pos, ori] = vehicle.GetPose();
        camera.SetPose(pos,ori); % Set the camera pose
        camera.Update(scene.id); % Update the camera
        camera.Display(); % Display the camera
    end
    % update the timing loop variables
    sim_time = sim_time + sim_dt;
    frame_count = frame_count + 1; 

    % slowly reduce the soil strength
    soil_strength_psi = 100.0-sim_time;
end % simulation loop

vci1 = soil_strength_psi + 1.0;
end % vci1 function 

function max_speed = GetMaxSpeed(display_camera, mobility_test_scene, mobility_test_vehicle)
% Function to calculate the maximum speed of the vehicle. Floors it until
% the vehicle is no longer accelerating.

% load the scene and vehicle
mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
scene_to_load = mavs_data_path+mobility_test_scene;
veh_to_load = mavs_data_path+mobility_test_vehicle;
scene = MavsScene(scene_to_load);
vehicle = MavsVehicle(veh_to_load, [0.0, 0.0, 0.0], 0.0);

% create the camera 
camera = MavsCamera();
camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
camera.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);

% initialize simulation variables
sim_dt = 0.01; % 100 Hz timestep
frame_count = 0; % set the frame count to zero
sim_time = 0.0;
dspeed = 0.0;
last_speed = 0.0;

% execute sim loop
while(dspeed>0.001 || sim_time<10.0)

    % get the vehicle state and speed and calculate the change in speed
    [~,~,lin_vel, ~] = vehicle.GetState();
    speed = sqrt(lin_vel(1)*lin_vel(1)+lin_vel(2)*lin_vel(2));
    dspeed = (speed-last_speed)/sim_dt;
    last_speed = speed;

    % update the vehicle using max throttle
    vehicle.Update(scene.id, 1.0, 0.0, 0.0, sim_dt);

    % update the sensors at 10 Hz if display_camera
    if (mod(frame_count,10)==0 && frame_count>0 && display_camera)
        [pos, ori] = vehicle.GetPose();
        camera.SetPose(pos,ori); % Set the camera pose
        camera.Update(scene.id); % Update the camera
        camera.Display(); % Display the camera
    end
    % update sim loop variables
    sim_time = sim_time + sim_dt;
    frame_count = frame_count + 1; 
end % simulation loop

    max_speed = last_speed;
end % max speed function 

function [cumulative_throttle, average_speed] = ...
    SpeedControlTest(desired_speed, soil_strength_psi, display_camera,...
    mobility_test_scene, mobility_test_vehicle)
% This function drives the vehicle a given course length at a desired speed
% using a PID controller to control the throttle setting for that speed. It
% logs the total throttle and uses that as a proxy for fuel use.

% set the course length and simulation time step
course_length = 200.0; % meters
sim_dt = 0.01; % 100 Hz timestep

% load the scene and set the soil strength
mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
scene_to_load = mavs_data_path+mobility_test_scene;
scene = MavsScene(scene_to_load);
scene.SetSoilProperties('clay',soil_strength_psi);

% create the vehicle and set the initial position 
veh_to_load = mavs_data_path+mobility_test_vehicle;
x = 0.0;
vehicle = MavsVehicle(veh_to_load, [x, 0.0, 0.0], 0.0);
% create the PID speed controller for the vehicle
controller = SpeedController(0.1, 0.01, 0.1/sim_dt);
controller.setpoint = desired_speed;

% create the camera 
camera = MavsCamera();
camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
camera.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);

% initialize some sim variables
frame_count = 0; % set the frame count to zero
cumulative_throttle = 0.0;
sim_time = 0.0;
avg_speed = 0.0;
num_avg_speed = 0.0;

% simulation loop
while(x<course_length)
    % get the state and x-position of the vehicle
    [pos,ori,lin_vel, ~] = vehicle.GetState();
    x = pos(1);

    % get the speed of the vehicle
    speed = sqrt(lin_vel(1)*lin_vel(1)+lin_vel(2)*lin_vel(2));

    % set the throttle command using the speed controller
    controller = controller.Update(speed);
    throttle = max(0.0, min(1.0, controller.output));
    cumulative_throttle = cumulative_throttle + throttle;

    % update the vehicle using the throttle from teh PID controller
    vehicle.Update(scene.id, throttle, 0.0, 0.0, sim_dt);

    % give the vehicle 7.5 seconds to ramp up, then log speed
    if (sim_time>7.5)
        avg_speed = avg_speed + speed;
        num_avg_speed = num_avg_speed + 1.0;
    end

    % update the sensors at 10 Hz if display_camera
    if (mod(frame_count,10)==0 && frame_count>0 && display_camera)
            camera.SetPose(pos,ori); % Set the camera pose
            camera.Update(scene.id); % Update the camera
            camera.Display(); % Display the camera
    end
    % update sim loop variables
    sim_time = sim_time + sim_dt;
    frame_count = frame_count + 1; 
end % simulation loop

% calculate the average speed and scale the throttle
average_speed = avg_speed/num_avg_speed;
cumulative_throttle = cumulative_throttle/1000.0;

end % speed control test function 