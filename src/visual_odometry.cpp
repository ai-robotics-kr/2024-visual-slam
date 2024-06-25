#include "visual_odometry.hpp"
#include "config.hpp"
#include <opencv2/opencv.hpp>

bool VisualOdometry::Init() {
  // read parameter from config file
  if (Config::SetParameterFile(config_file_path_) == false)
    return false;

  // @TODO type casting bool type
  auto use_camera = Config::Get<int>("use_camera");
  if (use_camera) {
    dataset_ = Dataset::Ptr(new Dataset());
    
    Vec3 left_t;
    left_t << Config::Get<double>("left_x"), Config::Get<double>("left_y"), Config::Get<double>("left_z");
    Camera::Ptr left_camera(
        new Camera(
            Config::Get<double>("left_fx"),
            Config::Get<double>("left_fy"),
            Config::Get<double>("left_cx"),
            Config::Get<double>("left_cy"),
            Config::Get<double>("left_baseline"),
            SE3(SO3(),left_t)));

    Vec3 right_t;
    right_t << Config::Get<double>("right_x"), Config::Get<double>("right_y"), Config::Get<double>("right_z");
    Camera::Ptr right_camera(
        new Camera(
            Config::Get<double>("right_fx"),
            Config::Get<double>("right_fy"),
            Config::Get<double>("right_cx"),
            Config::Get<double>("right_cy"),
            Config::Get<double>("right_baseline"),
            SE3(SO3(),right_t)));

    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(left_camera, right_camera);

    backend_->SetMap(map_);
    backend_->SetCameras(left_camera, right_camera);

    viewer_->SetMap(map_);
    return true;
  } 
  else {
    dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);
    return true;
  }
}

void VisualOdometry::Run() {
  while (1) {
    LOG(INFO) << "VO is running";
    if (Step() == false)
      break;
  }
  backend_->Stop();
  viewer_->Close();

  LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
  auto new_frame = dataset_->NextFrame();
  if (new_frame == nullptr)
    return false;

  auto t1 = std::chrono::steady_clock::now();
  bool success = frontend_->AddFrame(new_frame);
  auto t2 = std::chrono::steady_clock::now();
  auto elapsed_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  LOG(INFO) << "VO cost time : " << elapsed_time.count() << " seconds";
  return success;
}

void VisualOdometry::Shutdown() {
  backend_->Stop();
  viewer_->Close();
  LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step(cv::Mat left_img, cv::Mat right_img, double resize_scale) {
  std::cout << " " << left_img.size() << std::endl;

  auto new_frame = Frame::CreateFrame();
  cv::Mat left_img_resized, right_img_resized;
  if (resize_scale != 1) {
    cv::resize(left_img, left_img_resized, cv::Size(), resize_scale, resize_scale,
               cv::INTER_NEAREST);
    cv::resize(right_img, right_img_resized, cv::Size(), resize_scale, resize_scale,
               cv::INTER_NEAREST);
    new_frame->left_img_ = left_img_resized;
    new_frame->right_img_ = right_img_resized;
  } else {
    new_frame->left_img_ = left_img;
    new_frame->right_img_ = right_img;
  }
  if (new_frame == nullptr)
    return false;

  auto t1 = std::chrono::steady_clock::now();
  bool success = frontend_->AddFrame(new_frame);
  auto t2 = std::chrono::steady_clock::now();
  auto elapsed_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  LOG(INFO) << "VO cost time : " << elapsed_time.count() << " seconds";
  return success;
}