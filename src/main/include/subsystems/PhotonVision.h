#pragma once
#ifndef PHOTON_VISION_H
#define PHOTON_VISION_H

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <memory>
#include <vector>

class PhotonVision {
public:
    PhotonVision(const frc::AprilTagFieldLayout& fieldLayout, 
                 const std::string& camera1Name, const frc::Transform3d& camera1Transform,
                 const std::string& camera2Name, const frc::Transform3d& camera2Transform);

    void Update();
    std::vector<photon::PhotonPipelineResult> GetLatestResults() const;
    std::vector<frc::Pose3d> GetRobotPoses() const;

private:
    std::shared_ptr<photon::PhotonCamera> m_camera1;
    std::shared_ptr<photon::PhotonCamera> m_camera2;

    photon::PhotonPoseEstimator m_poseEstimator1;
    photon::PhotonPoseEstimator m_poseEstimator2;

    photon::PhotonPipelineResult m_latestResult1;
    photon::PhotonPipelineResult m_latestResult2;

    frc::Pose3d m_robotPose1;
    frc::Pose3d m_robotPose2;

    frc::AprilTagFieldLayout m_fieldLayout;  // Ensure this is declared last
};

#endif // PHOTON_VISION_H