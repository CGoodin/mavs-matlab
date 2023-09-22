function CheckMavsLoaded()

try
    mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
catch
    sprintf('Adding MAVS DLL to MATLAB PATH\n')
    mavs_dll_path = '..\mavs\dll';
    addpath(mavs_dll_path);
    syspath = getenv('PATH');
    dllPath = mavs_dll_path;
    setenv('PATH',[dllPath pathsep syspath]);
    %savepath;
end

end