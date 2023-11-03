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
        function Advance(obj, dt)
            clib.mavs_matlab_interface.mavs.matlab.UpdateMavsEnvironment(obj.id, dt);
        end
        function SetRainRate(obj, rain_rate)
            clib.mavs_matlab_interface.mavs.matlab.SetRainRate(obj.id, rain_rate);
        end
        function SetFog(obj, fog)
            clib.mavs_matlab_interface.mavs.matlab.SetFog(obj.id, fog);
        end
        function SetSnowRate(obj, snow_rate)
            clib.mavs_matlab_interface.mavs.matlab.SetSnowRate(obj.id, snow_rate);
        end
        function SetTurbidity(obj, turbid)
            clib.mavs_matlab_interface.mavs.matlab.SetTurbidity(obj.id, turbid);
        end
        function SetTimeOfDay(obj, hour)
            clib.mavs_matlab_interface.mavs.matlab.SetHour(obj.id, hour);
        end
        function SetCloudCover(obj, cloud_cover_frac)
            clib.mavs_matlab_interface.mavs.matlab.SetCloudCover(obj.id, cloud_cover_frac);
        end
        function SetSoilProperties(obj, soil_type, soil_strength)
            % soil_type should be clay or sand
            % soil_strength = RCI of the soil in PSI

            ss = 6894.76*soil_strength; % convert to pascals
            clib.mavs_matlab_interface.mavs.matlab.SetTerrainProperties(obj.id, soil_type, ss);
        end
    end
end