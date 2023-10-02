classdef MavsVehicleRosNode < MavsVehicle
    properties
        node;
        pub;
    end
    methods
        % vehicle node constructor
        function obj = MavsVehicleRosNode(node_name,topic_name,veh_to_load, initial_position, initial_heading)
            obj = obj@MavsVehicle(veh_to_load,initial_position,initial_heading);
            obj.node = ros.Node(node_name);
            obj.pub = ros.Publisher(obj.node,topic_name,'nav_msgs/Odometry');
        end
        % publish the current point cloud to a PointCloud2 message
        function PublishOdometry(obj, frame_id)
            % frame_id is the frame of the vehicle odometry
            odom_msg = obj.GetOdometryMsg();
            odom_msg.Header.FrameId=frame_id;
            send(obj.pub,odom_msg);
        end
        function odom_msg = GetOdometryMsg(obj)
            odom_msg = rosmessage('nav_msgs/Odometry');
            [p,q,v,av] = obj.GetState();
            odom_msg.Pose.Pose.Position.X = p(1);
            odom_msg.Pose.Pose.Position.Y = p(2);
            odom_msg.Pose.Pose.Position.Z = p(3);
            odom_msg.Pose.Pose.Orientation.W = q(1);
            odom_msg.Pose.Pose.Orientation.X = q(2);
            odom_msg.Pose.Pose.Orientation.Y = q(3);
            odom_msg.Pose.Pose.Orientation.Z = q(4);
            odom_msg.Twist.Twist.Linear.X = v(1);
            odom_msg.Twist.Twist.Linear.Y = v(2);
            odom_msg.Twist.Twist.Linear.Z = v(3);
            odom_msg.Twist.Twist.Angular.X = av(1);
            odom_msg.Twist.Twist.Angular.Y = av(2);
            odom_msg.Twist.Twist.Angular.Z = av(3);
        end
    end
end