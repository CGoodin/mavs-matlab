function CheckMavsLoaded()

fname = '..\mavs\dll';
curr_dir = pwd;
canon_path = strcat(curr_dir,strcat('\',fname));
full_path = string(canon_path);
current_path = string(getenv('PATH'));

if ~contains(current_path, full_path)
    sprintf('Adding MAVS DLL to MATLAB PATH')
    mavs_dll_path = full_path; 
    addpath(mavs_dll_path);
    syspath = getenv('PATH');
    dllPath = char(mavs_dll_path);
    setenv('PATH',[dllPath pathsep syspath]);
end

end