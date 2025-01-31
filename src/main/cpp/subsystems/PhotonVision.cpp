#include "subsystems/PhotonVision.h"

PhotonVision::PhotonVision(const frc::AprilTagFieldLayout& fieldLayout,
                           const std::string& camera1Name, const frc::Transform3d& camera1Transform,
                           const std::string& camera2Name, const frc::Transform3d& camera2Transform)
    : m_camera1(std::make_shared<photon::PhotonCamera>(camera1Name)),
      m_camera2(std::make_shared<photon::PhotonCamera>(camera2Name)),
      m_poseEstimator1(fieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, camera1Transform),
      m_poseEstimator2(fieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, camera2Transform),
      m_latestResult1(),
      m_latestResult2(),
      m_robotPose1(),
      m_robotPose2(),
      m_fieldLayout(fieldLayout) // Moved to last position to match declaration order
{
}

void PhotonVision::Update()
{
    // Get the latest vision results from both cameras
    auto results1 = m_camera1->GetAllUnreadResults();
    if (!results1.empty())
    {
        m_latestResult1 = results1.back(); // Store the latest result
    }

    auto results2 = m_camera2->GetAllUnreadResults();
    if (!results2.empty())
    {
        m_latestResult2 = results2.back(); // Store the latest result
    }

    // Update the pose estimator if there are valid targets
    if (m_latestResult1.HasTargets())
    {
        auto visionEst1 = m_poseEstimator1.Update(m_latestResult1);
        if (visionEst1)
        {
            m_robotPose1 = visionEst1.value().estimatedPose;
        }
    }

    if (m_latestResult2.HasTargets())
    {
        auto visionEst2 = m_poseEstimator2.Update(m_latestResult2);
        if (visionEst2)
        {
            m_robotPose2 = visionEst2.value().estimatedPose;
        }
    }
}

std::vector<photon::PhotonPipelineResult> PhotonVision::GetLatestResults() const
{
    return {m_latestResult1, m_latestResult2};
}

std::vector<frc::Pose3d> PhotonVision::GetRobotPoses() const
{
    return {m_robotPose1, m_robotPose2};
}