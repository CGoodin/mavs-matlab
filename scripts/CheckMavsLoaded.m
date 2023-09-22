function CheckMavsLoaded()

fname = '..\mavs\dll';
file = java.io.File(fname);
canon_path = file.getCanonicalPath();
full_path = string(canon_path);
current_path = string(getenv('PATH'));

if ~contains(current_path, full_path)
    sprintf('Adding MAVS DLL to MATLAB PATH')
    mavs_dll_path = full_path; %'..\mavs\dll';
    addpath(mavs_dll_path);
    syspath = getenv('PATH');
    dllPath = char(mavs_dll_path);
    setenv('PATH',[dllPath pathsep syspath]);
%     %savepath;
end

end