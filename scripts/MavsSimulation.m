classdef MavsSimulation
    properties
        scene;
        vehicle;
        scene_loaded;
        vehicle_loaded;
        cameras = [];
        lidars = [];
        drive_cam;
        mission;
    end
    properties (Hidden)
        cleanup
    end
    methods
        % simulation constructor
        function obj = MavsSimulation()
            obj.drive_cam = MavsCamera();
            obj.drive_cam.Initialize(480,270,0.006222, 0.0035, 0.0035);
            obj.drive_cam.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
            obj.scene_loaded = false;
            obj.vehicle_loaded = false;
            obj.mission = MavsMission();
        end
        % simulation destructor
        function delete(~)

        end
    end
end