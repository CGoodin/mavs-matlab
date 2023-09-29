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

        function SetSoilProperties(obj, soil_type, soil_strength)
            % soil_type should be clay or sand
            % soil_strength = RCI of the soil in PSI

            ss = 6894.76*soil_strength; % convert to pascals
            clib.mavs_matlab_interface.mavs.matlab.SetTerrainProperties(obj.id, soil_type, ss);
        end
    end
end