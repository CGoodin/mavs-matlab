classdef MavsScene
    properties
        id = 0
    end
    properties (Hidden)
        cleanup
    end
    methods
        % scene constructor
        function obj = MavsScene(scene_to_load)
            CheckMavsLoaded();
            % scene_to_load is the full path to the scene file to use
            obj.id = clib.mavs_matlab_interface.mavs.matlab.LoadMavsScene(scene_to_load);
            obj.cleanup = onCleanup(@()delete(obj));
        end
        % scene destructor
        function delete(obj)
            clib.mavs_matlab_interface.mavs.matlab.ClearMavsScene(obj.id);
        end
    end
end