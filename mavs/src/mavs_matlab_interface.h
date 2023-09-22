#ifndef MAVS_MATLAB_INTERFACE_H_
#define MAVS_MATLAB_INTERFACE_H_

#include <string>
#include <vector>

#if defined(_WIN32) || defined(WIN32)
#define EXPORT_CMD __declspec(dllexport)
#else
#define EXPORT_CMD 
#endif

namespace mavs {
namespace matlab {

//----- Generalized MAVS functions ---------------------//
EXPORT_CMD std::string GetMavsDataPath();

//----- Environment functions --------------------------//

//----- Scene functions --------------------------------//
EXPORT_CMD int LoadMavsScene(std::string scene_file);

EXPORT_CMD void ClearMavsScene(int scene_num);

//----- Lidar functions --------------------------------//
EXPORT_CMD int CreateMavsLidar(std::string lidar_type);

EXPORT_CMD void ClearMavsLidar(int lidar_num);

EXPORT_CMD void UpdateMavsLidar(int lidar_num, int env_num);

EXPORT_CMD void DisplayMavsLidar(int lidar_num);

EXPORT_CMD void SetLidarPose(int lidar_num, float px, float py, float pz, float ow, float ox, float oy, float oz);

EXPORT_CMD void SetLidarOffset(int lidar_num, float px, float py, float pz, float ow, float ox, float oy, float oz);

EXPORT_CMD std::vector<float> GetLidarPointsXyzi(int lidar_num);
EXPORT_CMD std::vector<float> GetLidarPointsX(int lidar_num);
EXPORT_CMD std::vector<float> GetLidarPointsY(int lidar_num);
EXPORT_CMD std::vector<float> GetLidarPointsZ(int lidar_num);
EXPORT_CMD int GetLidarNumPoints(int lidar_num);

//---- Camera functions -------------------------------//
EXPORT_CMD int CreateMavsCamera();

EXPORT_CMD int CreateMavsOrthoCamera();

EXPORT_CMD void ClearMavsCamera(int camera_num);

EXPORT_CMD void InitializeMavsCamera(int camera_num, int nx, int ny, float fa_x, float fa_y, float flen);

EXPORT_CMD void UpdateMavsCamera(int camera_num, int env_num);

EXPORT_CMD std::vector<float> GetCameraImage(int camera_num);

EXPORT_CMD void DisplayMavsCamera(int camera_num);

EXPORT_CMD bool IsCameraDisplayOpen(int camera_num);

EXPORT_CMD void SetCameraPose(int camera_num, float px, float py, float pz, float ow, float ox, float oy, float oz);

EXPORT_CMD void SetCameraOffset(int camera_num, float px, float py, float pz, float ow, float ox, float oy, float oz);

EXPORT_CMD void FreeCamera(int camera_num);

EXPORT_CMD std::vector<float> GetDrivingCommandFromCamera(int camera_num);

EXPORT_CMD std::vector<int> GetImageDimensions(int camera_num);

//---- Vehicle functions --------------------------------//
EXPORT_CMD int LoadMavsVehicle(std::string veh_file);

EXPORT_CMD void ClearMavsVehicle(int veh_num);

EXPORT_CMD void UpdateMavsVehicle(int veh_num, int env_num, float throttle, float steering, float braking, float dt);

EXPORT_CMD std::vector<float> GetMavsVehiclePose(int veh_num);

EXPORT_CMD void SetMavsVehiclePose(int veh_num, float px, float py, float heading);

} // namespace mavs
} // namespace matlab

#endif