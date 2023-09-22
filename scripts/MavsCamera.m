classdef MavsCamera
    properties
        id = 0
    end
    properties (Hidden)
        cleanup
    end
    methods
        % camera constructor
        function obj = MavsCamera(varargin)
            cam_type = 'rgb';
            if (nargin>0)
                cam_type = string(varargin{1});
            end
            if (strcmp(cam_type,'ortho'))
                obj.id = clib.mavs_matlab_interface.mavs.matlab.CreateMavsOrthoCamera();
            else
                obj.id = clib.mavs_matlab_interface.mavs.matlab.CreateMavsCamera();
            end
            obj.cleanup = onCleanup(@()delete(obj));
        end
        % camera destructor
        function delete(obj)
            clib.mavs_matlab_interface.mavs.matlab.ClearMavsCamera(obj.id);
        end
        % initialize the camera to a certain resolution and focal length
        function Initialize(obj, num_hori_pix, num_vert_pix, hori_dim, vert_dim, flen)
            % num_hori_pix is the number of horizontal pixels
            % num_vert_pix is the number of vertical pixels
            % hori_dim is the size of the image plane in the horizontal
            % dimension (meters)
            % vert_dim is the size of the image plane in the vertical
            % dimension (meters)
            % flen is the focal length of the camera, in meters
            clib.mavs_matlab_interface.mavs.matlab.InitializeMavsCamera(obj.id,...
                num_hori_pix, num_vert_pix, hori_dim, vert_dim, flen);
        end
        function FreePose(obj)
            clib.mavs_matlab_interface.mavs.matlab.FreeCamera(obj.id);
        end
        % set the position and orientation of the camera
        function SetPose(obj, position, orientation)
            % position is the vehicle coordinates in ENU
            % orientation is the vehicle orientation in ENU
            clib.mavs_matlab_interface.mavs.matlab.SetCameraPose(obj.id, ...
                position(1), position(2), position(3),...
                orientation(1), orientation(2), orientation(3), orientation(4));
        end
        % update the camera
        function Update(obj, scene_id)
            % scene_id is the id # of the scene to update
            clib.mavs_matlab_interface.mavs.matlab.UpdateMavsCamera(obj.id,scene_id);
        end
        % Display the camera image to a X-window
        function Display(obj)
            clib.mavs_matlab_interface.mavs.matlab.DisplayMavsCamera(obj.id);
        end
        % Set the offset of the camera from the vehicle CG
        function SetOffset(obj,p, q)
            % p is the offset in X-Y-Z
            % q is the relative orientation, as a quaternion
            clib.mavs_matlab_interface.mavs.matlab.SetCameraOffset(obj.id,...
                p(1), p(2), p(3), q(1), q(2), q(3), q(4));
        end
        % Grab the driving command from a camera window
        % Drive using the W-A-S-D keys
        function [throttle, steering, braking] = GetDrivingCommand(obj)
            dc = clib.mavs_matlab_interface.mavs.matlab.GetDrivingCommandFromCamera(obj.id);
            throttle = dc(1);
            steering = dc(3);
            braking = dc(2);
        end
        function is_open=IsOpen(obj)
            is_open = clib.mavs_matlab_interface.mavs.matlab.IsCameraDisplayOpen(obj.id);
        end
        function img_data = GetImage(obj)
            mavs_img_buff = clib.mavs_matlab_interface.mavs.matlab.GetCameraImage(obj.id);
            img_buff = single(mavs_img_buff);
            dims = clib.mavs_matlab_interface.mavs.matlab.GetImageDimensions(obj.id);
            img_data = zeros(dims(2),dims(1),3);
            ndx = 1;
            for k=1:3
                for i=1:dims(2)
                    for j=1:dims(1)
                        img_data(i,j,k)=img_buff(ndx)/255.0;
                        ndx = ndx + 1;
                    end
                end
            end
        end
    end
end