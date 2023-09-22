classdef MavsVehicle
    properties
        id = 0
    end
    properties (Hidden)
        cleanup
    end
    methods
        % vehicle constructor
        function obj = MavsVehicle(veh_to_load, initial_position, initial_heading)
            if ~exist('initial_position','var')
                initial_position=[0.0, 0.0];
            end
            if ~exist('initial_heading','var')
                initial_heading=0.0;
            end
            % veh_to_load is the full path to the vehicle file to use
            obj.id = clib.mavs_matlab_interface.mavs.matlab.LoadMavsVehicle(veh_to_load);
            clib.mavs_matlab_interface.mavs.matlab.SetMavsVehiclePose(obj.id, ...
                initial_position(1),initial_position(2),initial_heading);
            obj.cleanup = onCleanup(@()delete(obj));
        end
        % vehicle destructor
        function delete(obj)
            clib.mavs_matlab_interface.mavs.matlab.ClearMavsVehicle(obj.id);
        end
        % Update vehicle
        function Update(obj, scene_id, throttle, steering, braking, dt)
            % scene_id is the ID of the scene to update inside
            clib.mavs_matlab_interface.mavs.matlab.UpdateMavsVehicle(obj.id,scene_id, ...
                throttle, steering, braking, dt);
        end
        % get the current pose of the vehicle
        function [position,orientation]=GetPose(obj)
            pose = clib.mavs_matlab_interface.mavs.matlab.GetMavsVehiclePose(obj.id);
            position = [pose(1),pose(2),pose(3)];
            orientation=[pose(4),pose(5),pose(6),pose(7)];
        end
        function [position,orientation,lin_vel, ang_vel]=GetState(obj)
            pose = clib.mavs_matlab_interface.mavs.matlab.GetMavsVehiclePose(obj.id);
            position = [pose(1),pose(2),pose(3)];
            orientation=[pose(4),pose(5),pose(6),pose(7)];
            lin_vel = [pose(8), pose(9), pose(10)];
            ang_vel = [pose(11), pose(12), pose(13)];
        end
    end
end