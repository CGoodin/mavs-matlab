function MavsApp
% set up global variables which will be used in callback functions
CheckMavsLoaded();
global mavs_sim; %#ok<*GVMIS> 
mavs_sim = MavsSimulation();

% create the main app window and set the layout
% see: https://www.mathworks.com/help/matlab/creating_guis/create-and-run-a-simple-programmatic-app.html
fig = uifigure('Name','MAVS','Scrollable','on','Position',[488,100,560,700]);

gl = uigridlayout(fig,[16,7]);

gl.RowHeight = {30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30};
gl.ColumnWidth = {'fit','1x'};

current_row = 1;

%------ Scene handling ---------------------------------------------------%
scene_panel = uipanel(gl);
scene_panel.Layout.Row = [current_row, current_row+2];
scene_panel.Layout.Column = [1 7];

scene_lbl = uilabel(gl);
scene_lbl.Text = "Scene Selection";
scene_lbl.FontSize = 20;
scene_lbl.FontName = "Helvetica";
scene_lbl.FontWeight = "bold";
scene_lbl.HorizontalAlignment = "center";
scene_lbl.Layout.Column=[1 7];
scene_lbl.Layout.Row=current_row;
current_row = current_row + 1;

scene_file_lbl = uilabel(gl,'Tooltip', ...
    'Click the "Load Scene" button to set');
scene_file_lbl.Text = 'Scene File:';
scene_file_lbl.FontSize = 16;
scene_file_lbl.FontName = "Helvetica";
scene_file_lbl.FontWeight = "bold";

scene_file_lbl.Layout.Row=current_row;
scene_file_lbl.Layout.Column=1;

scene_file_edit = uieditfield(gl,'Tooltip', ...
    'Click the "Load Scene" button to set');
scene_file_edit.Layout.Row=current_row;
scene_file_edit.Layout.Column=[2 5];

scene_file_btn = uibutton(gl,'ButtonPushedFcn', ...
    @(scene_file_btn,event)LoadScene(scene_file_btn,scene_file_edit),...
    'Tooltip','Select the MAVS scene File');
scene_file_btn.Text = "Load Scene";
scene_file_btn.FontName = "Helvetica";
scene_file_btn.FontWeight = "bold";
scene_file_btn.Layout.Row=current_row;
scene_file_btn.Layout.Column=[6 7];

current_row = current_row + 1;

scene_view_btn = uibutton(gl,'ButtonPushedFcn', ...
    @(scene_view_btn,event)ViewScene(scene_view_btn,fig),...
    'Tooltip','View the selected scene');
scene_view_btn.Text = 'View Scene';
scene_view_btn.FontName = "Helvetica";
scene_view_btn.FontWeight = "bold";
scene_view_btn.Layout.Row=current_row;
scene_view_btn.Layout.Column=[1 7];

current_row = current_row + 1;

%------ Mission handling -------------------------------------------------%
env_panel = uipanel(gl);
env_panel.Layout.Row = [current_row, current_row+7];
env_panel.Layout.Column = [5 7];

mission_panel = uipanel(gl,'Tooltip',...
    'Coordinates are in the East-North-Up (ENU) frame.');
mission_panel.Layout.Row = [current_row, current_row+5];
mission_panel.Layout.Column = [1 4];

mission_lbl = uilabel(gl);
mission_lbl.Text = "Mission Selection";
mission_lbl.FontSize = 20;
mission_lbl.FontName = "Helvetica";
mission_lbl.FontWeight = "bold";
mission_lbl.HorizontalAlignment = "center";
mission_lbl.Layout.Column=[1 4];
mission_lbl.Layout.Row=current_row;

env_lbl = uilabel(gl);
env_lbl.Text = "Environment";
env_lbl.FontSize = 20;
env_lbl.FontName = "Helvetica";
env_lbl.FontWeight = "bold";
env_lbl.HorizontalAlignment = "center";
env_lbl.Layout.Column=[5 7];
env_lbl.Layout.Row=current_row;
current_row = current_row + 1;

llc_lbl = uilabel(gl,'Tooltip','Southwest corner of the area-of-operation.');
llc_lbl.Text = 'Lower-left Corner (x,y):';
llc_lbl.Layout.Row=current_row;
%llc_lbl.FontSize = 20;
llc_lbl.FontName = "Helvetica";
llc_lbl.FontWeight = "bold";
llc_lbl.Layout.Column=1;

llc_x_edit = uieditfield(gl,'numeric','Tooltip',...
    'Lowest x-coordinate, ENU meters');
llc_x_edit.Layout.Row=current_row;
llc_x_edit.Layout.Column=2;
llc_x_edit.Value = -100.0;
llc_x_edit.FontName = "Helvetica";
llc_x_edit.FontWeight = "bold";
llc_y_edit = uieditfield(gl,'numeric','Tooltip',...
    'Lowest y-coordinate, ENU meters');
llc_y_edit.Layout.Row=current_row;
llc_y_edit.Layout.Column=3;
llc_y_edit.Value = -100.0;
llc_y_edit.FontName = "Helvetica";
llc_y_edit.FontWeight = "bold";

rain_lbl = uilabel(gl, 'Tooltip','Rain rate in mm/h, 25 is max allowed.');
rain_lbl.Text = 'Rain Rate (mm/h):';
rain_lbl.Layout.Row=current_row;
rain_lbl.FontName = "Helvetica";
rain_lbl.FontWeight = "bold";
rain_lbl.Layout.Column=[5, 6];
rain_edit = uieditfield(gl,'numeric', 'Tooltip',...
    'Rain rate in mm/h, 25 is max allowed.');
rain_edit.Layout.Row=current_row;
rain_edit.Layout.Column=7;
rain_edit.Value = 0.0;
rain_edit.FontName = "Helvetica";
rain_edit.FontWeight = "bold";

current_row = current_row + 1;

urc_lbl = uilabel(gl,'Tooltip','Northeast corner of the area-of-operation.');
urc_lbl.Text = 'Upper-right Corner (x,y):';
urc_lbl.FontName = "Helvetica";
urc_lbl.FontWeight = "bold";
urc_lbl.Layout.Row=current_row;
%urc_lbl.FontSize = 20;
urc_lbl.FontName = "Helvetica";
urc_lbl.FontWeight = "bold";
urc_lbl.Layout.Column=1;

urc_x_edit = uieditfield(gl,'numeric','Tooltip',...
    'Highest x-coordinate, ENU meters');
urc_x_edit.FontName = "Helvetica";
urc_x_edit.FontWeight = "bold";
urc_x_edit.Layout.Row=current_row;
urc_x_edit.Layout.Column=2;
urc_x_edit.Value = 100.0;
urc_y_edit = uieditfield(gl,'numeric','Tooltip',...
    'Highest y-coordinate, ENU meters');
urc_y_edit.FontName = "Helvetica";
urc_y_edit.FontWeight = "bold";
urc_y_edit.Layout.Row=current_row;
urc_y_edit.Layout.Column=3;
urc_y_edit.Value = 100.0;

fog_lbl = uilabel(gl,'Tooltip','Fog density, 100 is max');
fog_lbl.Text = 'Fog (0-100):';
fog_lbl.Layout.Row=current_row;
fog_lbl.FontName = "Helvetica";
fog_lbl.FontWeight = "bold";
fog_lbl.Layout.Column=[5, 6];
fog_edit = uieditfield(gl,'numeric','Tooltip','Fog density, 100 is max');
fog_edit.Layout.Row=current_row;
fog_edit.Layout.Column=7;
fog_edit.Value = 0.0;
fog_edit.FontName = "Helvetica";
fog_edit.FontWeight = "bold";

current_row = current_row + 1;

veh_pose_lbl = uilabel(gl,'Tooltip',...
    'Initial position and orientation of the vehicle.');
veh_pose_lbl.Text = 'Start Pose (x,y,heading):';
veh_pose_lbl.Layout.Row=current_row;
%veh_pose_lbl.FontSize = 20;
veh_pose_lbl.FontName = "Helvetica";
veh_pose_lbl.FontWeight = "bold";
veh_pose_lbl.Layout.Column=1;

veh_pose_x_edit = uieditfield(gl,'numeric','Tooltip',...
    'Initial x-coordinate of the vehicle, in ENU meters');
veh_pose_x_edit.FontName = "Helvetica";
veh_pose_x_edit.FontWeight = "bold";
veh_pose_x_edit.Layout.Row=current_row;
veh_pose_x_edit.Layout.Column=2;
veh_pose_y_edit = uieditfield(gl,'numeric','Tooltip',...
    'Initial y-coordinate of the vehicle, in ENU meters');
veh_pose_y_edit.FontName = "Helvetica";
veh_pose_y_edit.FontWeight = "bold";
veh_pose_y_edit.Layout.Row=current_row;
veh_pose_y_edit.Layout.Column=3;
veh_pose_heading_edit = uieditfield(gl,'numeric','Tooltip',...
    'Initial heading of the vehicle, radians north of east');
veh_pose_heading_edit.FontName = "Helvetica";
veh_pose_heading_edit.FontWeight = "bold";
veh_pose_heading_edit.Layout.Row=current_row;
veh_pose_heading_edit.Layout.Column=4;

snow_lbl = uilabel(gl, 'Tooltip','Snow rate, max is 25 mm/h');
snow_lbl.Text = 'Snow (mm/h):';
snow_lbl.Layout.Row=current_row;
snow_lbl.FontName = "Helvetica";
snow_lbl.FontWeight = "bold";
snow_lbl.Layout.Column=[5, 6];
snow_edit = uieditfield(gl,'numeric', 'Tooltip','Snow rate, max is 25 mm/h');
snow_edit.Layout.Row=current_row;
snow_edit.Layout.Column=7;
snow_edit.Value = 0.0;
snow_edit.FontName = "Helvetica";
snow_edit.FontWeight = "bold";

current_row = current_row + 1;

gp_lbl = uilabel(gl,'Tooltip','Desired goal point in local ENU meters');
gp_lbl.Text = 'Goal point (x,y):';
gp_lbl.Layout.Row=current_row;
%gp_lbl.FontSize = 20;
gp_lbl.FontName = "Helvetica";
gp_lbl.FontWeight = "bold";
gp_lbl.Layout.Column=1;

gp_x_edit = uieditfield(gl,'numeric','Tooltip',...
    'x-coordinate of the goal point in ENU meters');
gp_x_edit.Layout.Row=current_row;
gp_x_edit.Layout.Column=2;
gp_x_edit.Value = 75.0;
gp_x_edit.FontName = "Helvetica";
gp_x_edit.FontWeight = "bold";
gp_y_edit = uieditfield(gl,'numeric','Tooltip',...
    'y-coordinate of the goal point in ENU meters');
gp_y_edit.Layout.Row=current_row;
gp_y_edit.Layout.Column=3;
gp_y_edit.Value = 75.0;
gp_y_edit.FontName = "Helvetica";
gp_y_edit.FontWeight = "bold";

turbid_lbl = uilabel(gl,'Tooltip','Atmospheric turbidity (haziness)');
turbid_lbl.Text = 'Turbidity (2-10):';
turbid_lbl.Layout.Row=current_row;
turbid_lbl.FontName = "Helvetica";
turbid_lbl.FontWeight = "bold";
turbid_lbl.Layout.Column=[5, 6];
turbid_edit = uieditfield(gl,'numeric','Tooltip','Atmospheric turbidity (haziness)');
turbid_edit.Layout.Row=current_row;
turbid_edit.Layout.Column=7;
turbid_edit.Value = 0.0;
turbid_edit.FontName = "Helvetica";
turbid_edit.FontWeight = "bold";

current_row = current_row + 1;

mission_view_btn = uibutton(gl,'ButtonPushedFcn',...
    @(mission_view_btn,event)ViewMission(mission_view_btn,fig, llc_x_edit, llc_y_edit, urc_x_edit, ...
    urc_y_edit, veh_pose_x_edit, veh_pose_y_edit, gp_x_edit, gp_y_edit),...
    'Tooltip','View the mission layout on the map');
mission_view_btn.Text = 'View Mission';
mission_view_btn.FontName = "Helvetica";
mission_view_btn.FontWeight = "bold";
mission_view_btn.Layout.Row=current_row;
mission_view_btn.Layout.Column=[1 4];

% mission_approve_btn = uibutton(gl,'ButtonPushedFcn',...
%     @(mission_approve_btn,event)ApproveMission(mission_approve_btn,mission_view_btn,...
%     llc_x_edit, llc_y_edit, urc_x_edit, urc_y_edit, gp_x_edit, gp_y_edit));
% mission_approve_btn.Text = 'Approve Mission';
% mission_approve_btn.FontName = "Helvetica";
% mission_approve_btn.FontWeight = "bold";
% mission_approve_btn.Layout.Row=current_row;
% mission_approve_btn.Layout.Column=[3 4];

tod_lbl = uilabel(gl,'Tooltip','Hour of the day, in military (0-24) time.');
tod_lbl.Text = 'Time (0-24):';
tod_lbl.Layout.Row=current_row;
tod_lbl.FontName = "Helvetica";
tod_lbl.FontWeight = "bold";
tod_lbl.Layout.Column=[5, 6];
tod_edit = uieditfield(gl,'numeric','Tooltip',...
    'Hour of the day, in military (0-24) time.');
tod_edit.Layout.Row=current_row;
tod_edit.Layout.Column=7;
tod_edit.Value = 12;
tod_edit.FontName = "Helvetica";
tod_edit.FontWeight = "bold";

current_row = current_row + 1;

%------ Vehicle handling -------------------------------------------------%
vehicle_panel = uipanel(gl);
vehicle_panel.Layout.Row = [current_row, current_row+1];
vehicle_panel.Layout.Column = [1 4];

veh_lbl = uilabel(gl);
veh_lbl.Text = "Vehicle Selection";
veh_lbl.FontSize = 20;
veh_lbl.FontName = "Helvetica";
veh_lbl.FontWeight = "bold";
veh_lbl.HorizontalAlignment = "center";
veh_lbl.Layout.Column=[1 4];
veh_lbl.Layout.Row=current_row;

cloud_lbl = uilabel(gl,'Tooltip','Percent of the sky covered by clouds.');
cloud_lbl.Text = 'Cloud Cover (0-100):';
cloud_lbl.Layout.Row=current_row;
cloud_lbl.FontName = "Helvetica";
cloud_lbl.FontWeight = "bold";
cloud_lbl.Layout.Column=[5, 6];
cloud_edit = uieditfield(gl,'numeric','Tooltip',...
    'Percent of the sky covered by clouds.');
cloud_edit.Layout.Row=current_row;
cloud_edit.Layout.Column=7;
cloud_edit.Value = 0.0;
cloud_edit.FontName = "Helvetica";
cloud_edit.FontWeight = "bold";


current_row = current_row + 1;

%veh_file_lbl = uilabel(gl);
%veh_file_lbl.Text = 'Vehicle File:';
%veh_file_lbl.FontName = "Helvetica";
%veh_file_lbl.FontWeight = "bold";
%veh_file_lbl.Layout.Row=current_row;
%veh_file_lbl.Layout.Column=1;

veh_file_edit = uieditfield(gl,'Tooltip',...
    'Click the "Load Vehicle" button to select a MAVS vehicle.');
veh_file_edit.Layout.Row=current_row;
veh_file_edit.Layout.Column=[1 2];

veh_file_btn = uibutton(gl,'ButtonPushedFcn',...
    @(veh_file_btn,event)LoadVehicle(veh_file_btn,veh_file_edit, ...
    veh_pose_x_edit, veh_pose_y_edit, veh_pose_heading_edit),...
    'Tooltip','Select a MAVS vehicle.');
veh_file_btn.Text = 'Load Vehicle';
veh_file_btn.FontName = "Helvetica";
veh_file_btn.FontWeight = "bold";
veh_file_btn.Layout.Row=current_row;
veh_file_btn.Layout.Column=[3 4];


view_env_btn = uibutton(gl,'ButtonPushedFcn',...
    @(view_env_btn,event)ViewEnv(fig, view_env_btn, rain_edit, fog_edit, snow_edit,...
    turbid_edit, tod_edit, cloud_edit),'Tooltip',...
    'View the currently selected environmental conditions.');
view_env_btn.Text = 'View Environment';
view_env_btn.FontName = "Helvetica";
view_env_btn.FontWeight = "bold";
view_env_btn.Layout.Row=current_row;
view_env_btn.Layout.Column=[5 7];

current_row = current_row + 1;

%------ Sensor handling _-------------------------------------------------%
sensor_panel = uipanel(gl);
sensor_panel.Layout.Row = [current_row, current_row+2];
sensor_panel.Layout.Column = [1 7];

veh_lbl = uilabel(gl);
veh_lbl.Text = "Sensor Selection";
veh_lbl.FontSize = 20;
veh_lbl.FontName = "Helvetica";
veh_lbl.FontWeight = "bold";
veh_lbl.HorizontalAlignment = "center";
veh_lbl.Layout.Column=[1 7];
veh_lbl.Layout.Row=current_row;
current_row = current_row + 1;

sensor_listbox = uilistbox(gl,"Items","",'Tooltip','Sensors on the vehicle.');
sensor_listbox.Layout.Row = [current_row current_row+1];
sensor_listbox.Layout.Column = [3 7];

add_sensor_btn = uibutton(gl,'ButtonPushedFcn',...
    @(add_sensor_btn,event)AddSensor(add_sensor_btn, sensor_listbox),...
    'Tooltip','Add a sensor to the vehicle.');
add_sensor_btn.Text = 'Add Sensor';
add_sensor_btn.FontName = "Helvetica";
add_sensor_btn.FontWeight = "bold";
add_sensor_btn.Layout.Row=current_row;
add_sensor_btn.Layout.Column=[1 2];

remove_sensor_btn = uibutton(gl,'ButtonPushedFcn',...
    @(remove_sensor_btn,event)RemoveSensor(sensor_listbox),...
    'Tooltip','Remove the selected sensor from the vehicle.');
remove_sensor_btn.Text = 'Remove Sensor';
remove_sensor_btn.FontName = "Helvetica";
remove_sensor_btn.FontWeight = "bold";
remove_sensor_btn.Layout.Row=current_row+1;
remove_sensor_btn.Layout.Column=[1 2];

current_row = current_row + 2; %1;

%------ Save video? ------------------------------------------------------%
save_panel = uipanel(gl);
save_panel.Layout.Row = current_row;
save_panel.Layout.Column = [1 7];

save_file_edit = uieditfield(gl,'Tooltip','Full path of the output file.');
save_file_edit.Layout.Row=current_row;
save_file_edit.Layout.Column=[2 4];
save_file_edit.Value = 'mavs_matlab_playback';
save_file_edit.FontName = "Helvetica";
save_file_edit.FontWeight = "bold";

set(save_file_edit,'Enable','off')
        
save_file_btn = uibutton(gl,'ButtonPushedFcn',...
    @(save_file_btn,event)SaveMovieFile(save_file_btn,save_file_edit),...
     'Tooltip', ['Click this to set the output file',newline,'Do not include an extension']);
save_file_btn.Text = 'Set Output File';
save_file_btn.FontName = "Helvetica";
save_file_btn.FontWeight = "bold";
save_file_btn.Layout.Row=current_row;
save_file_btn.Layout.Column=[5 7];
set(save_file_btn,'Enable','off')

save_box = uicheckbox(gl,...
    'ValueChangedFcn',...
    @(save_box,event) SaveBoxChanged(save_box,save_file_edit, save_file_btn),...
    'Tooltip','Check this box to save video output of the simulation.');
save_box.Text = 'Save Video';
save_box.FontName = "Helvetica";
save_box.FontWeight = "bold";
save_box.Layout.Row=current_row;
save_box.Layout.Column=1;
save_box.Value = false;

current_row = current_row + 1;

%------ Run simulation ---------------------------------------------------%
run_sim_btn = uibutton(gl,'ButtonPushedFcn',...
    @(run_sim_btn,event)SimulateOdoa(run_sim_btn, fig, llc_x_edit, ...
    llc_y_edit, urc_x_edit, urc_y_edit, gp_x_edit, gp_y_edit, ...
    rain_edit, fog_edit, snow_edit,turbid_edit, tod_edit, cloud_edit),...
    'Tooltip','Run a simulation in autonomous mode.');
run_sim_btn.Text = "Run Autonomy";
run_sim_btn.FontName = "Helvetica";
run_sim_btn.FontWeight = "bold";
run_sim_btn.Layout.Row=current_row;
run_sim_btn.Layout.Column=[1 3];

drive_btn = uibutton(gl,'ButtonPushedFcn',...
    @(drive_btn,event)SimulateDrive(drive_btn, fig,...
    rain_edit, fog_edit, snow_edit,turbid_edit, tod_edit, cloud_edit),...
    'Tooltip','Drive the vehicle manually using the W-A-S-D keys.');
drive_btn.Text = "Drive Vehicle";
drive_btn.FontName = "Helvetica";
drive_btn.FontWeight = "bold";
drive_btn.Layout.Row=current_row;
drive_btn.Layout.Column=[5 7];

end

function RemoveSensor(sensor_listbox)
    global mavs_sim;
    to_remove = -1;
    for i=1:numel(sensor_listbox.Items)
        if strcmp(sensor_listbox.Value,sensor_listbox.Items(i))
            to_remove = i;
            break;
        end
    end
    if (to_remove>0)
        for j=1:numel(mavs_sim.lidars)
            if (strcmp(mavs_sim.lidars(j).sensor_name,sensor_listbox.Value))
                mavs_sim.lidars(j)=[];
                break;
            end
        end
        for j=1:numel(mavs_sim.cameras)
            if (strcmp(mavs_sim.cameras(j).sensor_name,sensor_listbox.Value))
                mavs_sim.cameras(j)=[];
                break;
            end
        end
        for i=to_remove:numel(sensor_listbox.Items)-1
            sensor_listbox.Items(i)=sensor_listbox.Items(i+1);
        end
        sensor_listbox.Items(end)='';
    end
end

function AddSensor(add_sensor_btn, sensor_listbox)
    global mavs_sim;
    %color = add_sensor_btn.BackgroundColor;
    set(add_sensor_btn,'Text','Adding sensor...','Backgroundcolor','r','visible','on');
    drawnow;

    sensor_fig = uifigure('Name','Add Sensor');
    sens_gl = uigridlayout(sensor_fig,[8,4]);
    sens_gl.RowHeight = {30,30,30,30, 30, 30, 30, 30};
    sens_gl.ColumnWidth = {'fit','1x'};
    
    type_dd = uidropdown(sens_gl, "ValueChangedFcn",...
        @(src,event) UpdateList(src,event),'Tooltip',...
        'Select the type of sensor');
    type_dd.Items=["Lidar", "Camera"];
    type_dd.Layout.Column = [1 2];
    type_dd.Layout.Row = 1;

    lidar_list = ["OS2", "M8", "HDL-64E", "HDL-32E", "VLP-16", "LMS-291", "OS1", "OS1-16", "RS32"];
    camera_list = ["XCD-V60","Raspberry Pi Camera Module 2", "Raspberry Pi Camera Module 3"];
    dd = uidropdown(sens_gl,'Tooltip','Select the sensor brand/model.');
    dd.Items = lidar_list;
    dd.Layout.Column=[3 4];
    dd.Layout.Row = 1;
    function UpdateList(~, ~)
        if (strcmp(type_dd.Value,"Lidar"))
            dd.Items = lidar_list;
        elseif (strcmp(type_dd.Value,"Camera"))
            dd.Items = camera_list;
        end
        drawnow;
    end

    off_lbl = uilabel(sens_gl,'Text',...
        'Offset from vehicle center-of-gravity (x,y,z):','Tooltip', ...
        'X is forward, Y is left, Z is up');
    off_lbl.Layout.Row=2;
    off_lbl.Layout.Column=[1 4];
    off_x_edit = uieditfield(sens_gl,'numeric','Tooltip',...
        'x (forward) offset in meters');
    off_x_edit.Layout.Row=3;
    off_x_edit.Layout.Column=1;
    off_x_edit.Value = 1.0;
    off_y_edit = uieditfield(sens_gl,'numeric','Tooltip',...
        'y (left) offset in meters');
    off_y_edit.Layout.Row=3;
    off_y_edit.Layout.Column=2;
    off_y_edit.Value = 0.0;
    off_z_edit = uieditfield(sens_gl,'numeric','Tooltip',...
        'z (up) offset in meters');
    off_z_edit.Layout.Row=3;
    off_z_edit.Layout.Column=3;
    off_z_edit.Value = 1.5;

    ori_lbl = uilabel(sens_gl,'Text','Relative orientation (w,x,y,z):',...
        'Tooltip','Specified as a quaternion.');
    ori_lbl.Layout.Row=4;
    ori_lbl.Layout.Column=[1 4];
    ori_w_edit = uieditfield(sens_gl,'numeric','Tooltip',...
        'w-component of the quaternion');
    ori_w_edit.Layout.Row=5;
    ori_w_edit.Layout.Column=1;
    ori_w_edit.Value = 1.0;
    ori_x_edit = uieditfield(sens_gl,'numeric','Tooltip',...
        'x-component of the quaternion');
    ori_x_edit.Layout.Row=5;
    ori_x_edit.Layout.Column=2;
    ori_x_edit.Value = 0.0;
    ori_y_edit = uieditfield(sens_gl,'numeric','Tooltip',...
        'y-component of the quaternion');
    ori_y_edit.Layout.Row=5;
    ori_y_edit.Layout.Column=3;
    ori_y_edit.Value = 0.0;
    ori_z_edit = uieditfield(sens_gl,'numeric','Tooltip',...
        'z-component of the quaternion');
    ori_z_edit.Layout.Row=5;
    ori_z_edit.Layout.Column=4;
    ori_z_edit.Value = 0.0;

    sensor_name_label = uilabel(sens_gl,'Text','Sensor Name:','Tooltip',...
        'Give the sensor a unique name so you can ID it later.');
    sensor_name_label.Layout.Row=6;
    sensor_name_label.Layout.Column=1;
    sensor_name_edit = uieditfield(sens_gl,'Tooltip',...
        'Give the sensor a unique name so you can ID it later.');
    sensor_name_edit.Layout.Row=6;
    sensor_name_edit.Layout.Column=[2 4];
    sensor_name_edit.Value = 'my_sensor';

    display_box = uicheckbox(sens_gl,'Tooltip',...
        'Check to display the sensor output in real time during the simulation.');
    display_box.Text = 'Display Sensor Output?';
    display_box.FontName = "Helvetica";
    display_box.FontWeight = "bold";
    display_box.Layout.Row=7;
    display_box.Layout.Column=1;
    display_box.Value = false;

    save_sens_box = uicheckbox(sens_gl,'Tooltip',...
        'Check to save the sensor output frame by frame.');
    save_sens_box.Text = 'Save Sensor Output?';
    save_sens_box.FontName = "Helvetica";
    save_sens_box.FontWeight = "bold";
    save_sens_box.Layout.Row=7;
    save_sens_box.Layout.Column=[2 3];
    save_sens_box.Value = false;

    fnsh_sensor_btn = uibutton(sens_gl,'Text','Add','ButtonPushedFcn',...
    @(fnsh_sensor_btn,event)FinishSensor(),'Tooltip',...
    'Click when finished editing sensor.');
    fnsh_sensor_btn.Layout.Row=8;
    fnsh_sensor_btn.Layout.Column=[1 2];

    cancel_btn = uibutton(sens_gl,'Text','Cancel',...
        'ButtonPushedFcn',@(cancel_btn,event)CancelSensor(),'Tooltip',...
        'Exit without adding sensor');
    cancel_btn.Layout.Row=8;
    cancel_btn.Layout.Column=[3 4];

    function FinishSensor()
        for i=1:numel(sensor_listbox.Items)
            if (strcmp(sensor_listbox.Items(i),sensor_name_edit.Value))
                uialert(sensor_fig,'The sensor name must be unique.','Naming Error');
                return;
            end
        end
        if numel(sensor_listbox.Items)==0
            sensor_listbox.Items = [sensor_listbox.Items {sensor_name_edit.Value}];
        elseif (strcmp(sensor_listbox.Items(end),""))
            sensor_listbox.Items(end) = {sensor_name_edit.Value};
        else
            sensor_listbox.Items(end+1) = {sensor_name_edit.Value};
        end
        if (strcmp(type_dd.Value,"Lidar"))
            lidar = MavsLidar(dd.Value);
            lidar.sensor_name = sensor_name_edit.Value;
            lidar.display_out = display_box.Value;
            mavs_sim.lidars = [mavs_sim.lidars lidar];
            mavs_sim.lidars(end).SetOffset([off_x_edit.Value, off_y_edit.Value, off_z_edit.Value],...
            [ori_w_edit.Value, ori_x_edit.Value, ori_y_edit.Value, ori_z_edit.Value]);
        else
            camera = MavsCamera();
            % for specs, see: https://www.raspberrypi.com/documentation/accessories/camera.html
            % and https://www.image-sensing-solutions.eu/xcd_v60.html
            if (strcmp(dd.Value,"XCD-V60"))
                camera.Initialize(640,480,0.0048,0.0036,0.0035);
            elseif(strcmp(dd.Value,"Raspberry Pi Camera Module 2"))
                camera.Initialize(640,480,0.00368, 0.00276, 0.00304);
            elseif(strcmp(dd.Value,"Raspberry Pi Camera Module 3"))
                camera.Initialize(1536,864,0.00645, 0.00363, 0.00474);
            else
                camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
            end
            camera.display_out = display_box.Value;
            camera.sensor_name = sensor_name_edit.Value;
            mavs_sim.cameras = [mavs_sim.cameras camera];
            mavs_sim.cameras(end).SetOffset([off_x_edit.Value, off_y_edit.Value, off_z_edit.Value],...
            [ori_w_edit.Value, ori_x_edit.Value, ori_y_edit.Value, ori_z_edit.Value]);
        end
        %set(add_sensor_btn,'Text','Added','Backgroundcolor',[0 1 0]);
        %set(add_sensor_btn,'Enable','off')
        set(add_sensor_btn,'Text','Add Sensor','Backgroundcolor',[0.96,0.96,0.96]);
        
        closereq();
        
    end
    function CancelSensor()
        set(add_sensor_btn,'Text','Add Sensor','Backgroundcolor',[0.96,0.96,0.96]);
        closereq();
    end
    waitfor(sensor_fig);
    drawnow;
end

function SaveMovieFile(save_file_btn,save_file_edit)
    global mavs_sim;
    color = save_file_btn.BackgroundColor;
    set(save_file_btn,'Text','Getting file...','Backgroundcolor','r','visible','on');
    %filter = {'*.mp4'};
    [file, path] = uiputfile('*.*','Output Video, No Extension','mavs_matlab_playback');
    mavs_sim.mission.video_fname=strcat(path,file);
    save_file_edit.Value = mavs_sim.mission.video_fname;
    set(save_file_btn,'Text','Set Output File','Backgroundcolor',color,'visible','on');
    drawnow;
end

function SaveBoxChanged(save_box,save_file_edit, save_file_btn)
    global mavs_sim;
    if (save_box.Value)
        set(save_file_edit,'Enable','on')
        set(save_file_btn,'Enable','on')
        mavs_sim.mission.save_video = true;
    else
        set(save_file_edit,'Enable','off')
        set(save_file_btn,'Enable','off')
    end

end

function LoadScene(scene_file_btn,scene_file_edit)
    global mavs_sim;
    set(scene_file_btn,'Text','Loading...','Backgroundcolor','r','visible','on');
    drawnow;
    mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
    [scene_to_load,full_path_to_scene] = uigetfile(mavs_data_path+...
        '\scenes\*.json','Select a scene file');
    if (scene_to_load)
        mavs_sim.scene = MavsScene(strcat(full_path_to_scene,scene_to_load));
        scene_file_edit.Value = scene_to_load;
        mavs_sim.scene_loaded = true;
        set(scene_file_btn,'Text','Load New?','Backgroundcolor',[0 1 0]);
        %set(scene_file_btn,'Enable','off')
    %else
        %set(scene_file_btn,'Text','Load Scene','Backgroundcolor',[0.96,0.96,0.96]);
        %set(scene_file_btn,'Enable','on')        
    end
    drawnow;
end

function ViewEnv(fig, view_env_btn,rain_edit, fog_edit, snow_edit,...
    turbid_edit, tod_edit, cloud_edit)
    global mavs_sim;
    if (~mavs_sim.scene_loaded)
        uialert(fig,'You must load a scene before you can view it.','Missing Scene');
        return;
    end
    color = view_env_btn.BackgroundColor;
    set(view_env_btn,'Text','Viewing, close view window to exit.',...
        'Backgroundcolor','g','visible','on');
    drawnow;

    mavs_sim.scene.SetRainRate(min(25.0,max(0.0,rain_edit.Value)));
    mavs_sim.scene.SetFog(min(100.0,max(0.0,fog_edit.Value))/1000.0);
    mavs_sim.scene.SetSnowRate(min(25.0,max(0.0,snow_edit.Value)));
    mavs_sim.scene.SetTurbidity(min(10.0,max(2.0, turbid_edit.Value)));
    mavs_sim.scene.SetTimeOfDay(min(24,max(0,tod_edit.Value)));
    mavs_sim.scene.SetCloudCover(min(1.0,max(0.0,cloud_edit.Value/100.0)));

    scene_viewer = MavsCamera();
    scene_viewer.Initialize(480,270,0.006222, 0.0035, 0.0035);
    scene_viewer.SetPose([0.0, 0.0, 2.0],[1.0, 0.0, 0.00, 0.0]);
    scene_viewer.FreePose();
    scene_viewer.Update(mavs_sim.scene.id);
    scene_viewer.Display();
    while(scene_viewer.IsOpen())
        scene_viewer.Update(mavs_sim.scene.id);
        scene_viewer.Display();
        mavs_sim.scene.Advance(1.0/30.0);
    end
    set(view_env_btn,'Text','View Scene','Backgroundcolor',color);
    drawnow;
end

function ViewScene(scene_view_btn,fig)
    global mavs_sim;
    if (~mavs_sim.scene_loaded)
        uialert(fig,'You must load a scene before you can view it.','Missing Scene');
        return;
    end
    color = scene_view_btn.BackgroundColor;
    set(scene_view_btn,'Text','Viewing, close view window to exit.',...
        'Backgroundcolor','g','visible','on');
    drawnow;
    scene_viewer = MavsCamera();
    scene_viewer.Initialize(480,270,0.006222, 0.0035, 0.0035);
    scene_viewer.SetPose([0.0, 0.0, 2.0],[1.0, 0.0, 0.00, 0.0]);
    scene_viewer.FreePose();
    scene_viewer.Update(mavs_sim.scene.id);
    scene_viewer.Display();
    while(scene_viewer.IsOpen())
        scene_viewer.Update(mavs_sim.scene.id);
        scene_viewer.Display();
    end
    set(scene_view_btn,'Text','View Scene','Backgroundcolor',color);
    drawnow;
end

function LoadVehicle(veh_file_btn,veh_file_edit, ...
    veh_pose_x_edit, veh_pose_y_edit, veh_pose_heading_edit)

    global mavs_sim;

    set(veh_file_btn,'Text','Loading...','Backgroundcolor','r','visible','on');
    drawnow;
    mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
    [veh_to_load,full_path_to_veh] = uigetfile(mavs_data_path+...
        '\vehicles\rp3d_vehicles\*.json','Select a scene file');
    if (veh_to_load)
        x = veh_pose_x_edit.Value;
        y = veh_pose_y_edit.Value;
        heading = veh_pose_heading_edit.Value;
        mavs_sim.vehicle = MavsVehicle(strcat(full_path_to_veh,veh_to_load),...
            [x y], heading);
        veh_file_edit.Value = veh_to_load;
        mavs_sim.vehicle_loaded = true;
        set(veh_file_btn,'Text','Load New?','Backgroundcolor',[0 1 0]);
        %set(veh_file_btn,'Enable','off')
    else
        set(veh_file_btn,'Text','Load Vehicle','Backgroundcolor',[0.96,0.96,0.96]);
        set(veh_file_btn,'Enable','on') 
    end
    drawnow;
end

% function ApproveMission(mission_approve_btn,mission_view_btn,llc_x_edit, llc_y_edit, urc_x_edit, ...
%     urc_y_edit, gp_x_edit, gp_y_edit)
%     global mavs_sim;
%     mavs_sim.mission.urc = [urc_x_edit.Value, urc_y_edit.Value];
%     mavs_sim.mission.llc = [llc_x_edit.Value, llc_y_edit.Value]; 
%     mavs_sim.mission.goal_point = [gp_x_edit.Value, gp_y_edit.Value];
%     set(mission_approve_btn,'Text','Approved','Backgroundcolor',[0 1 0]);
%     set(mission_approve_btn,'Enable','off');
%     set(mission_view_btn,'Enable','off');
% end

function ViewMission(~,fig, llc_x_edit, llc_y_edit, urc_x_edit, ...
    urc_y_edit, veh_pose_x_edit, veh_pose_y_edit, gp_x_edit, gp_y_edit)
    
    global mavs_sim;

    if (~mavs_sim.scene_loaded)
        uialert(fig,'You must load a scene the to view the mission.','Missing Scene');
        return;
    end

    if (mavs_sim.scene_loaded)
        scene_viewer = MavsCamera('ortho');
        scene_width = urc_x_edit.Value - llc_x_edit.Value;
        scene_height = urc_y_edit.Value - llc_y_edit.Value;
        pix_res = 0.25;
        nx = int16(scene_width/pix_res);
        ny = int16(scene_height/pix_res);
        scene_viewer.Initialize(nx,ny,pix_res*nx, pix_res*ny, 1.0);
        xp = 0.5*(llc_x_edit.Value + urc_x_edit.Value);
        yp = 0.5*(llc_y_edit.Value + urc_y_edit.Value);
        scene_viewer.SetPose([xp, yp, 100.0],[0.7071, 0.0, 0.7071, 0.0]);
        scene_viewer.Update(mavs_sim.scene.id);
        img = scene_viewer.GetImage();
        img = flip(img, 2);
        img = imrotate(img,-90);
        RA = imref2d(size(img),[llc_x_edit.Value,urc_y_edit.Value],[llc_y_edit.Value, urc_y_edit.Value]);
        imshow(img,RA);
        set(gca,'YDir','normal');
        gx = int16(gp_x_edit.Value);
        gy = int16(gp_y_edit.Value);
        sx = int16(veh_pose_x_edit.Value);
        sy = int16(veh_pose_y_edit.Value);
        hold on;
        plot(gx,gy, 'g+', 'MarkerSize', 5, 'LineWidth', 1);
        plot(sx,sy, 'y+', 'MarkerSize', 5, 'LineWidth', 1);
        drawnow;
    end
end

function SimulateDrive(drive_btn, fig,...
    rain_edit, fog_edit, snow_edit,turbid_edit, tod_edit, cloud_edit)

    global mavs_sim

    btn_color = drive_btn.BackgroundColor;
    set(drive_btn,'Text','Driving...','Backgroundcolor','r','visible','on');
    drawnow;

    if (~mavs_sim.scene_loaded)
        uialert(fig,'You must load a scene the for simulation.','Missing Scene');
        set(drive_btn,'Text','Drive Vehicle','Backgroundcolor',btn_color);
        return;
    end

    if (~mavs_sim.vehicle_loaded)
        uialert(fig,'You must load a vehicle for the simulation.','Missing Vehicle');
        set(run_sim_btn,'Text','Run Simulation','Backgroundcolor',btn_color);
        return;
    end

    mavs_sim.scene.SetRainRate(min(25.0,max(0.0,rain_edit.Value)));
    mavs_sim.scene.SetFog(min(100.0,max(0.0,fog_edit.Value))/1000.0);
    mavs_sim.scene.SetSnowRate(min(25.0,max(0.0,snow_edit.Value)));
    mavs_sim.scene.SetTurbidity(min(10.0,max(2.0, turbid_edit.Value)));
    mavs_sim.scene.SetTimeOfDay(min(24,max(0,tod_edit.Value)));
    mavs_sim.scene.SetCloudCover(min(1.0,max(0.0,cloud_edit.Value/100.0)));

    % create the camera for driving the vehicle
    camera = MavsCamera();
    % arguments are: 
    % num_horizontal_pixels, num_vertical_pixels, 
    % horiz_plane_size (m), vert_plane size (m), focal length (m)
    camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
    % set the camera offset from the vehicle CG
    camera.SetOffset([-10.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
    % open the camera display window
    camera.Display();
    
    frame_count = 0; % set the frame count to zero
    sim_dt = 0.01; % 100 Hz timestep

    while(camera.IsOpen())
        % get the driving commands from the camera window
        [throttle, steering, braking] = camera.GetDrivingCommand();
    
        % update the vehicle using the driving commands
        mavs_sim.vehicle.Update(mavs_sim.scene.id, throttle, steering, braking, sim_dt);
    
        % update the sensors every 10th step (10 Hz)
        if (mod(frame_count,10)==0)
            % Get the current vehicle pose
            [pos, ori] = mavs_sim.vehicle.GetPose();
            camera.SetPose(pos,ori); % Set the camera pose
            camera.Update(mavs_sim.scene.id); % Update the camera
            camera.Display(); % Display the camera
        end
    
        frame_count = frame_count + 1; % update the frame counter
    end % simulation loop
    set(drive_btn,'Text','Drive Vehicle','Backgroundcolor',[0.96,0.96,0.96]);
    set(drive_btn,'Enable','on')
    drawnow;
end

function SimulateOdoa(run_sim_btn, fig, llc_x_edit, llc_y_edit, ...
    urc_x_edit, urc_y_edit, gp_x_edit, gp_y_edit,rain_edit, fog_edit, ...
    snow_edit,turbid_edit, tod_edit, cloud_edit)

    global mavs_sim

    mavs_sim.mission.urc = [urc_x_edit.Value, urc_y_edit.Value];
    mavs_sim.mission.llc = [llc_x_edit.Value, llc_y_edit.Value]; 
    mavs_sim.mission.goal_point = [gp_x_edit.Value, gp_y_edit.Value];

    btn_color = run_sim_btn.BackgroundColor;
    set(run_sim_btn,'Text','Running simulation...','Backgroundcolor','r','visible','on');
    drawnow;

    if (~mavs_sim.scene_loaded)
        uialert(fig,'You must load a scene the for simulation.','Missing Scene');
        set(run_sim_btn,'Text','Run Simulation','Backgroundcolor',btn_color);
        return;
    end

    if (~mavs_sim.vehicle_loaded)
        uialert(fig,'You must load a vehicle for the simulation.','Missing Vehicle');
        set(run_sim_btn,'Text','Run Simulation','Backgroundcolor',btn_color);
        return;
    end

    if (length(mavs_sim.lidars)<1)
        uialert(fig,'You must add a lidar sensor to the simulation.','Missing Sensor');
        set(run_sim_btn,'Text','Run Simulation','Backgroundcolor',btn_color);
        return;
    end

    mavs_sim.scene.SetRainRate(min(25.0,max(0.0,rain_edit.Value)));
    mavs_sim.scene.SetFog(min(100.0,max(0.0,fog_edit.Value))/1000.0);
    mavs_sim.scene.SetSnowRate(min(25.0,max(0.0,snow_edit.Value)));
    mavs_sim.scene.SetTurbidity(min(10.0,max(2.0, turbid_edit.Value)));
    mavs_sim.scene.SetTimeOfDay(min(24,max(0,tod_edit.Value)));
    mavs_sim.scene.SetCloudCover(min(1.0,max(0.0,cloud_edit.Value/100.0)));

    % control parameters
    desired_speed = 7.5;
    look_ahead_dist = 1.5*desired_speed;

    % perception parameters
    map_res = 2; % per meters
    scan_range_limits=[8.0,60.0]; % set the range of lidar points to use
    max_ray_range = scan_range_limits(2)-1;  % set the max range of points to put into scan

    % map parameters
    % dimensions of the scene [[llx, urx], [lly, ury]];
    scene_limits = [[mavs_sim.mission.llc(1), mavs_sim.mission.urc(1)];...
        [mavs_sim.mission.llc(2),mavs_sim.mission.urc(2)]];
    % goal pose in local enu
    goal_pose = [mavs_sim.mission.goal_point(1),mavs_sim.mission.goal_point(2), 0.0]; 
    waypoints = [mavs_sim.mission.goal_point(1), mavs_sim.mission.goal_point(2)];

    % create a map
    map_2d = binaryOccupancyMap(scene_limits(1,2)-scene_limits(1,1),...
        scene_limits(2,2)-scene_limits(2,1),map_res);
    map_2d.GridLocationInWorld = [scene_limits(1,1), scene_limits(2,1)];

    % create a state space for the planner
    % see: https://www.mathworks.com/help/nav/ref/plannerhybridastar.html
    ss = stateSpaceSE2;
    ss.StateBounds = [map_2d.XWorldLimits;map_2d.YWorldLimits;[-pi pi]];
    sv = validatorOccupancyMap(ss);

    % create a planner
    planner = plannerHybridAStar(sv, MinTurningRadius=6.5, MotionPrimitiveLength=10.0);

    % create a controller
    ppc = controllerPurePursuit;

    % set the initial throttle, steering, and braking
    throttle = 0.0;
    steering = 0.0;
    braking = 0.0;

    % create a figure window for plotting results
    h=figure;
    h.Position = [100 100 1024 512];
    lidar_plot_h = subplot(2,2,3:4);
    
    % create the array for saving frames, may not get used
    movie_frames = cell((mavs_sim.mission.max_sim_time+1)*20);

    % do the check for the distance to goal
    [p,~]=mavs_sim.vehicle.GetPose();
    dist_to_goal = norm([p(1),p(2)]-[goal_pose(1),goal_pose(2)]);

    % start the main simulation loop
    frame_count = 0;
    elapsed_time = 0.0;
    num_imgs = 1;
    sim_dt = 1.0/100.0;
    while(ishandle(h) && elapsed_time<mavs_sim.mission.max_sim_time && ...
             dist_to_goal>mavs_sim.mission.goal_radius)

        % update the vehicle and render the camera at 100 Hz
        mavs_sim.vehicle.Update(mavs_sim.scene.id, throttle, steering, braking, sim_dt);
        [p,q, veh_vel, ~] = mavs_sim.vehicle.GetState();
        dist_to_goal = norm([p(1),p(2)]-[goal_pose(1),goal_pose(2)]);
        vspeed = sqrt(veh_vel(1)*veh_vel(1)+ veh_vel(2)*veh_vel(2));
            
        if (~mavs_sim.mission.InBounds(p(1),p(2)))
            sprintf('Vehicle left map at (%f, %f) \n',[p(1),p(2)]);
            break;
        end

        %update the camera at 20 Hz
        if (mod(frame_count,5)==0)
            mavs_sim.drive_cam.SetPose(p,q);
            mavs_sim.drive_cam.Update(mavs_sim.scene.id);
            img = mavs_sim.drive_cam.GetImage();
            subplot(2,2,1);
            image(img);
            drawnow;
            if (mavs_sim.mission.save_video)
                if (ishandle(h))
                    movie_frames{num_imgs} = getframe(h);
                    num_imgs = num_imgs+1;
                end
            end
        end

        % update the lidar at 10 Hz
        if (mod(frame_count,10)==0)
            % set the pose of the sensor
            mavs_sim.lidars(1).SetPose(p,q);
        
            % do a scan and get the points
            mavs_sim.lidars(1).Update(mavs_sim.scene.id);
            xyz_points = mavs_sim.lidars(1).GetPoints(true);
        
            % convert the points to a matlab point cloud object
            pc_cloud = pointCloud(xyz_points');

            % register the point cloud and plot it
            if (ishandle(h))
                rot_mat = quat2rotm(q);
                tform = rigidtform3d(rot_mat,p);
                pc_reg = pctransform(pc_cloud,tform);
                subplot(2,2,3:4);
                scatter3(pc_reg.Location(:,1),pc_reg.Location(:,2),pc_reg.Location(:,3),4,pc_reg.Location(:,3),'filled');
                xlim(lidar_plot_h,[scene_limits(1,1), scene_limits(1,2)]);
                ylim(lidar_plot_h,[scene_limits(2,1), scene_limits(2,2)]);
            end

            % convert the cloud to a scan
            params = lidarParameters('OS2-64',1024);
            pc_org = pcorganize(pc_cloud,params);
            groundPtsIdx = segmentGroundFromLidarData(pc_org);
            nonGroundPtCloud= select(pc_org,~groundPtsIdx,OutputSize="full");
            scan = pc2scan(nonGroundPtCloud, ScanRangeLimits=scan_range_limits); 

            % insert the scan into the 2d map
            [yaw, ~, ~] = quat2angle(q);
            current_pose = [p(1),p(2),yaw];
            insertRay(map_2d,current_pose,scan,max_ray_range);
    
            % Update the planner at 1 Hz
            if (mod(frame_count,100)==0 && frame_count>0)
                % run the path planner on the current map
                sv.Map = map_2d;
                startPose = [double(p(1)) double(p(2)) yaw]; % [meters, meters, radians]
                refpath = plan(planner,startPose,goal_pose,SearchMode='exhaustive');
                waypoints = refpath.States(:,1:2);
                if (ishandle(h))
                    subplot(2,2,2);
                    show(planner)
                    drawnow 
                end
            end % run the path planner at 1 Hz
            
            % update the other sensors
            for ln=2:length(mavs_sim.lidars)
                mavs_sim.lidars(ln).SetPose(p,q);
                mavs_sim.lidars(ln).Update(mavs_sim.scene.id);
                if (mavs_sim.lidars(ln).display_out)
                    mavs_sim.lidars(ln).Display();
                end
            end
            for cn=1:length(mavs_sim.cameras)
                mavs_sim.cameras(cn).SetPose(p,q);
                mavs_sim.cameras(cn).Update(mavs_sim.scene.id);
                if (mavs_sim.cameras(cn).display_out)
                    mavs_sim.cameras(cn).Display();
                end
            end
        end

        % run the planner at 100 Hz
        ppc.Waypoints = waypoints;
        ppc.DesiredLinearVelocity = desired_speed;
        ppc.LookaheadDistance = look_ahead_dist;
        [goal_speed,~, desired_pos] = ppc(current_pose);
        [throttle, steering] = GetDrivingCommand(throttle, steering, ...
            vspeed, goal_speed, current_pose, desired_pos);

        % update the frame counter
        frame_count = frame_count + 1;
        elapsed_time = elapsed_time + sim_dt;
    end

    if (mavs_sim.mission.save_video)
        % Save the frames to a video file
        movie_frames((num_imgs-1):end) = [];
        video_writer_obj = VideoWriter(mavs_sim.mission.video_fname,'MPEG-4');
        video_writer_obj.FrameRate = 20;
        open(video_writer_obj);
        for i=1:length(movie_frames)
            writeVideo(video_writer_obj, movie_frames{i});
        end
        close(video_writer_obj);
    end

    if (ishandle(h))
        close(h);
    end
    set(run_sim_btn,'Text','Run Simulation','Backgroundcolor',[0.96,0.96,0.96]);
    set(run_sim_btn,'Enable','on')
    drawnow;
end

function [throttle, steering] = GetDrivingCommand(current_throttle, current_steering, ...
    current_speed, desired_speed, current_pose, desired_pos)
% this a function that adapts the output of the pure pursuit controller
% to throttle and steering commands
    throttle_step = 0.01;
    steering_step = 0.0025;
    max_steering_angle = deg2rad(22.5);
    vehicle_wheelbase = 3.0;

    % do the throttle control
    if (desired_speed>current_speed)
        throttle = current_throttle + throttle_step;
    else
        throttle = current_throttle - throttle_step;
    end
    throttle = min([1.0,max([throttle, 0.0])]);

    % do the steering
    v = desired_pos - current_pose(1:2);
    desired_yaw = atan2(v(2),v(1));
    goal_steering = ((desired_yaw-current_pose(3))/max_steering_angle)/vehicle_wheelbase;
    steering = 0.0;
    if (goal_steering>current_steering)
        steering = current_steering + steering_step;
    elseif (goal_steering<current_steering)
        steering = current_steering - steering_step;
    end
    steering = min([1.0,max([steering, -1.0])]);
end