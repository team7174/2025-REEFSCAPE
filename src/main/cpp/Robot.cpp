// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandScheduler.h>

Robot::Robot()
{
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();

  frc::SmartDashboard::PutNumber("Match Time", double(timer.GetMatchTime()));

  /*
   * This example of adding Limelight is very simple and may not be sufficient for on-field use.
   * Users typically need to provide a standard deviation that scales with the distance to target
   * and changes with number of tags available.
   *
   * This example is sufficient to show that vision integration is possible, though exact implementation
   * of how to use vision should be tuned per-robot and to the team's specification.
   */

  if (LimelightPose("limelight-threeg") == 0) {
    LimelightPose("limnelight-three");
  }

  if (kUseLimelight)
  {
    auto const driveState = m_container.drivetrain.GetState();
    auto const heading = driveState.Pose.Rotation().Degrees();
    auto const omega = driveState.Speeds.omega;

    LimelightHelpers::SetRobotOrientation("limelight-threeg", heading.value(), 0, 0, 0, 0, 0);
    auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-threeg");
    if (llMeasurement && llMeasurement->tagCount > 0 && units::math::abs(omega) < 2_tps && llMeasurement->pose.X() != 8.7741252_m)
    {
      m_container.drivetrain.AddVisionMeasurement(llMeasurement->pose, llMeasurement->timestampSeconds);
    }
    else
    {
      LimelightPose("limelight-threeg");
    }

    if (llMeasurement->tagCount == 0)
    {
      LimelightHelpers::SetRobotOrientation("limelight-three", heading.value(), 0, 0, 0, 0, 0);
      auto llMeasurement3 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-three");
      if (llMeasurement3 && llMeasurement3->tagCount > 0 && units::math::abs(omega) < 2_tps && llMeasurement3->pose.X() != 8.7741252_m)
      {
        m_container.drivetrain.AddVisionMeasurement(llMeasurement3->pose, llMeasurement3->timestampSeconds);
      }
      else
      {
        LimelightPose("limelight-three");
      }
    }
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
  timer.Start();
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

int Robot::LimelightPose(std::string llname)
{
  double xyStds;
  units::angle::radian_t degStds;

  std::shared_ptr<nt::NetworkTable> ll = nt::NetworkTableInstance::GetDefault().GetTable(llname);
  auto llBotPoseEntry = ll->GetEntry("botpose_wpiblue");
  auto llBotPose = llBotPoseEntry.GetDoubleArray({});

  // weirdness
  if (llBotPose.size() < 6)
  {
    return 0;
  }
  frc::Pose2d visionBotPose = frc::Pose2d(
      frc::Translation2d(units::length::meter_t(llBotPose[0]), units::length::meter_t(llBotPose[1])),
      frc::Rotation2d(units::angle::radian_t(llBotPose[5] * (M_PI / 180.0))));

  // getlastchange() in microseconds, ll latency in milliseconds
  auto visionTime = units::time::second_t((llBotPoseEntry.GetLastChange() / 1000000.0) - (llBotPose[6] / 1000.0));
  // auto visionTime = frc::Timer::GetFPGATimestamp() - (llBotPose[6]/1000.0)

  // distance from current pose to vision estimated pose
  units::meter_t poseDifference = m_container.drivetrain.GetRobotPose().Translation().Distance(visionBotPose.Translation());

  int tagCount = (int)llBotPose[7];
  double tagArea = llBotPose[10];
  // multiple targets detected
  if (tagCount >= 2)
  {
    xyStds = 0.5;
    degStds = units::angle::radian_t(6_deg);
  }
  // 1 target with large area and close to estimated pose
  else if (tagArea > 0.8 && poseDifference < 0.5_m)
  {
    xyStds = 1.0;
    degStds = units::angle::radian_t(12_deg);
  }
  // 1 target farther away and estimated pose is close
  else if (tagArea > 0.1 && poseDifference < 0.3_m)
  {
    xyStds = 2.0;
    degStds = units::angle::radian_t(30_deg);
  }
  else {
    return 0;
  }

  m_container.drivetrain.SetVisionMeasurementStdDevs({xyStds, xyStds, degStds.value()});
  m_container.drivetrain.AddVisionMeasurement(visionBotPose, visionTime);

  return tagCount;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
