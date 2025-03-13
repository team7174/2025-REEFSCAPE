#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <limits>
#include <memory>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Constants.h"
#include <string>

class Vision
{
public:
  Vision()
  {
    frontRightEstimator.SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);

    backLeftEstimator.SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);
  }

  photon::PhotonPipelineResult GetLatestResult() { return m_latestResult; }

  std::optional<photon::EstimatedRobotPose> GetBackLeftEstimatedGlobalPose()
  {
    std::optional<photon::EstimatedRobotPose> visionEst;

    // Run each new pipeline result through our pose estimator
    for (const auto &result : backLeftCamera.GetAllUnreadResults())
    {
      // cache result and update pose estimator
      auto visionEst = backLeftEstimator.Update(result);
      m_latestResult = result;
    }

    return visionEst;
  }

  std::optional<photon::EstimatedRobotPose> GetFrontRightEstimatedGlobalPose()
  {
    std::optional<photon::EstimatedRobotPose> visionEst;

    // Run each new pipeline result through our pose estimator
    for (const auto &result : frontRightCamera.GetAllUnreadResults())
    {
      // cache result and update pose estimator
      auto visionEst = frontRightEstimator.Update(result);
      m_latestResult = result;
    }

    return visionEst;
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose, std::string cameraName)
  {
    Eigen::Matrix<double, 3, 1> estStdDevs =
        VisionConstants::kSingleTagStdDevs;
    auto targets = GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto &tgt : targets)
    {
      auto tagPose = frontRightEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (cameraName == "BackLeft")
      {
        tagPose = backLeftEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      }
      if (tagPose)
      {
        numTags++;
        avgDist += tagPose->ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    if (numTags == 0)
    {
      return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1)
    {
      estStdDevs = VisionConstants::kMultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m)
    {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    }
    else
    {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
  }

private:
  photon::PhotonPoseEstimator frontRightEstimator{
      VisionConstants::kTagLayout,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      VisionConstants::kRobotToFrontRightCam};

  photon::PhotonPoseEstimator backLeftEstimator{
      VisionConstants::kTagLayout,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      VisionConstants::kRobotToBackLeftCam};

  photon::PhotonCamera frontRightCamera{"FrontRight"};
  photon::PhotonCamera backLeftCamera{"BackLeft"};

  // The most recent result, cached for calculating std devs
  photon::PhotonPipelineResult m_latestResult;
};