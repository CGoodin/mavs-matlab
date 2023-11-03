classdef MavsPose
    properties
        position
        orientation
    end
    methods
        % pose constructor
        function obj = MavsPose(p,q)
            obj.position = p;
            obj.orientation = q;
        end
    end
end