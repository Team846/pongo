#pragma once

#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "autos/GenericAuto.h"
#include "frc846/robot/GenericRobot.h"
#include "subsystems/robot_container.h"

class FunkyRobot : public frc846::robot::GenericRobot {
public:
  FunkyRobot();

  void OnInitialize() override;

  void OnDisable() override;
  void OnEnable() override;

  void OnPeriodic() override;

  void InitTeleop() override;
  void InitTest() override;

private:
  RobotContainer container_;

  static void VisionThread(RobotContainer* container) {
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(320, 240);
    camera.SetFPS(30);

    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    cs::CvSource outputStream =
        frc::CameraServer::PutVideo("ClimbStream", 320, 240);

    cv::Mat mat;

    while (true) {
      if (cvSink.GrabFrame(mat) != 0) {
        if (container->control_input_.GetReadings().camera_stream)
          outputStream.PutFrame(mat);
      } else {
        outputStream.NotifyError(cvSink.GetError());
      }
    }
  }

  frc::DigitalInput home_switch_{0};
  frc::DigitalInput coast_switch_{1};
  frc::DigitalInput gyro_switch_{2};

  int coast_count_{0};
  int homing_count_gyro{0};
  int homing_count_{0};
};
