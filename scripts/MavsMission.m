classdef MavsMission
    properties
        llc;
        urc;
        goal_point;
        max_sim_time;
        goal_radius;
        save_video;
        video_fname;
    end
    properties (Hidden)
        cleanup
    end
    methods
        % MavsMission constructor
        function obj = MavsMission()
            obj.llc = [-50.0, -50.0];
            obj.urc = [50.0, 50.0];
            obj.goal_point = [45.0, 0.0];
            obj.max_sim_time = 120.0;
            obj.goal_radius = 5.0;
            obj.save_video = false;
            obj.video_fname = 'mavs_matlab_playback';
        end
        % MavsMission destructor
        function delete(~)
            
        end
        function inbounds = InBounds(obj,px,py)
            inbounds=true;
            if (px>=obj.urc(1) || px<=obj.llc(1) || py>=obj.urc(2) || py<=obj.llc(2))
                inbounds = false;
            end
        end
    end
end