classdef MavsLidarRosNode < MavsLidar
    properties
        node;
        pub;
        odom_sub;
    end
    methods
        % lidar node constructor
        function obj = MavsLidarRosNode(node_name,topic_name,lidar_type)
            % node_name is the name of the lidar node
            % lidar_type is a string, allowed values are:
            % M8, HDL-64E, HDL-32E, VLP-16
            % LMS-291, OS1, OS1-16, OS2, or RS32
            obj = obj@MavsLidar(lidar_type);
            obj.node = ros.Node(node_name);
            obj.pub = ros.Publisher(obj.node,topic_name,'sensor_msgs/PointCloud2');
            obj.odom_sub = ros.Subscriber(obj.node,...
                'mavs/odom','nav_msgs/Odometry',@obj.OdomCallback);
        end
        % publish the current point cloud to a PointCloud2 message
        function PublishPointCloud2(obj, frame_id)
            % frame_id is the frame of the lidar
            pc2_msg = obj.GetPointCloud2Msg();
            pc2_msg.Header.FrameId=frame_id;
            send(obj.pub,pc2_msg);
        end
        % callback for the odometry function
        function OdomCallback(obj, odom_msg)
            %[odom_msg,status,statustext] = receive(obj.odom_sub,10)

            pos = [odom_msg.Pose.Pose.Position.X,...
                odom_msg.Pose.Pose.Position.Y,...
                odom_msg.Pose.Pose.Position.Z];
            ori = [odom_msg.Pose.Pose.Orientation.W,...
                odom_msg.Pose.Pose.Orientation.X,...
                odom_msg.Pose.Pose.Orientation.Y,...
                odom_msg.Pose.Pose.Orientation.Z];
            obj.SetPose(pos,ori);
        end
        % get the current point cloud as a ROS PointCloud2 message
        function pc2_msg = GetPointCloud2Msg(obj)
            % see: https://www.mathworks.com/matlabcentral/answers/395620-ros-create-pointcloud2-from-matlab-pointcloud#answer_365441
            XYZ = obj.GetPoints();
            
            pc2_msg = rosmessage('sensor_msgs/PointCloud2');

            % get number of points
            numPts = size(XYZ,2);
    
            % Assign metadata
            pc2_msg.Height = uint32(1);
            pc2_msg.Width = uint32(numPts);
            pc2_msg.PointStep = uint32(12); 
            pc2_msg.RowStep = uint32(12*numPts);

            % Assign point field data
            fieldNames = {'x','y','z'};
            pc2_msg.Data = zeros(numPts*pc2_msg.RowStep,1,'uint8');
            for idx = 1:3
                pc2_msg.Fields(idx) = rosmessage('sensor_msgs/PointField');
                fName = fieldNames{idx};
                pc2_msg.Fields(idx).Name(1:numel(fName)) = uint8(fName);
                pc2_msg.Fields(idx).Offset = uint32((idx-1)*4);
                pc2_msg.Fields(idx).Datatype = uint8(7);
                pc2_msg.Fields(idx).Count = uint32(1);
            end
            xb=typecast(XYZ(1,:),'uint8');
            yb=typecast(XYZ(2,:),'uint8');
            zb=typecast(XYZ(3,:),'uint8');
            data=[xb,yb,zb];
            pc2_msg.Data = data;
        end
    end
end