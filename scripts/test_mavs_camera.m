function test_mavs_camera()

    % define the scene to load
    mavs_data_path = clib.mavs_matlab_interface.mavs.matlab.GetMavsDataPath();
    scene_to_load = mavs_data_path+"/scenes/cube_scene.json";

    % load the scene
    scene = MavsScene(scene_to_load);

    % render an image
    camera = MavsCamera();
    camera.Initialize(480,270,0.006222, 0.0035, 0.0035);
    camera.SetOffset([0.0, 0.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
    camera.SetPose([0.0, 0.0, 0.0],[1.0, 0.0, 0.0, 0.0]);
    camera.Update(scene.id);
    img_data = camera.GetImage();
    image(img_data);
end
