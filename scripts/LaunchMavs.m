function LaunchMavs()

% Make sure DLL path is loaded
CheckMavsLoaded()

% Determine what you want to run
answer = questdlg('What would you like to run?', ...
	'MAVS Applications', ...
	'Driving Example','MAVS GUI','No thank you','No thank you');

% Handle response
switch answer
    case 'Driving Example'
        disp('Drive with the W-A-S-D keys.')
        MavsDrivingExample()
    case 'MAVS GUI'
        MavsApp()
    case 'No thank you'
        disp('Exiting')
end

end