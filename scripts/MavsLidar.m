classdef MavsLidar
    properties
        id = 0
        display_out = false;
    end
    properties (Hidden)
        cleanup
    end
    methods
        % lidar constructor
        function obj = MavsLidar(lidar_type)
            % lidar_type is a string, allowed values are:
            % M8, HDL-64E, HDL-32E, VLP-16
            % LMS-291, OS1, OS1-16, OS2, or RS32
            CheckMavsLoaded();
            obj.id = clib.mavs_matlab_interface.mavs.matlab.CreateMavsLidar(lidar_type);
            obj.cleanup = onCleanup(@()delete(obj));
        end
        % lidar destructor
        function delete(obj)
            clib.mavs_matlab_interface.mavs.matlab.ClearMavsLidar(obj.id);
        end
        function SetPose(obj, position, orientation)
            % position is the vehicle coordinates in ENU
            % orientation is the vehicle orientation in ENU
            clib.mavs_matlab_interface.mavs.matlab.SetLidarPose(obj.id, ...
                position(1), position(2), position(3),...
                orientation(1), orientation(2), orientation(3), orientation(4));
        end
        % update the lidar
        function Update(obj, scene_id)
            % scene_id is the id # of the scene to update
            clib.mavs_matlab_interface.mavs.matlab.UpdateMavsLidar(obj.id,scene_id);
        end
        % Display the lidar point cloud to a X-window
        function Display(obj)
            clib.mavs_matlab_interface.mavs.matlab.DisplayMavsLidar(obj.id);
        end
        % Set the offset of the lidar from the vehicle CG
        function SetOffset(obj,p, q)
            % p is the offset in X-Y-Z
            % q is the relative orientation, as a quaternion
            clib.mavs_matlab_interface.mavs.matlab.SetLidarOffset(obj.id,...
                p(1), p(2), p(3), q(1), q(2), q(3), q(4));
        end
        % return points as a list
        function XYZ=GetPoints(obj,remove_invalid)
            if ~exist('remove_invalid','var')
                % third parameter does not exist, so default it to something
                remove_invalid=false;
            end
            X = clib.mavs_matlab_interface.mavs.matlab.GetLidarPointsX(obj.id);
            Y = clib.mavs_matlab_interface.mavs.matlab.GetLidarPointsY(obj.id);
            Z = clib.mavs_matlab_interface.mavs.matlab.GetLidarPointsZ(obj.id);
            XX = single(X);
            YY = single(Y);
            ZZ = single(Z);
            XYZ = [XX;YY;ZZ];
            if (remove_invalid)
                XYZ(:,all(XYZ == 0))=[];
            end
        end
    end
end