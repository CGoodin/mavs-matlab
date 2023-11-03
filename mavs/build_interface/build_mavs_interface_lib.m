function build_mavs_interface_lib()
    mavs_src = "C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_matlab_interface.h";
    mavs_lib = "C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\install\bin\mavs_matlab.dll";
    clibgen.generateLibraryDefinition(mavs_src,Libraries=mavs_lib, ReturnCArrays=true);
    build(definemavs_matlab_interface);
end