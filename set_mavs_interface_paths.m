function set_mavs_interface_paths()
    mavs_dll_path = '.\mavs\dll';
    addpath(mavs_dll_path);
    syspath = getenv('PATH');
    dllPath = mavs_dll_path;
    setenv('PATH',[dllPath pathsep syspath]);
    savepath;
end