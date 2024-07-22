// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class GoToX extends Command {
  /** Creates a new GoToX. */
  private Drivetrain m_drivetrain;
  private double m_distance;
  private DifferentialDriveOdometry m_odometry;
  public Pose2d m_pose = new Pose2d();
  private DifferentialDriveKinematics m_kinematics;

  public GoToX(Drivetrain drivetrain, double distance) {
    m_distance = distance;
    m_drivetrain = drivetrain;
    m_kinematics =
      new DifferentialDriveKinematics(Units.inchesToMeters(6.0));
    m_odometry = new DifferentialDriveOdometry(
      new Rotation2d(),
      m_drivetrain.m_leftEncoder.getDistance(), m_drivetrain.m_rightEncoder.getDistance(),
      new Pose2d(0, 0, new Rotation2d()));
    var gyroAngle = new Rotation2d(Math.toRadians(m_drivetrain.m_gyro.getAngleZ()));
    m_pose = m_odometry.update(gyroAngle,
      m_drivetrain.m_leftEncoder.getDistance(),
      m_drivetrain.m_rightEncoder.getDistance());

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var gyroAngle = new Rotation2d(Math.toRadians(m_drivetrain.m_gyro.getAngleZ()));
    m_pose = m_odometry.update(gyroAngle, m_drivetrain.m_leftEncoder.getDistance(), m_drivetrain.m_rightEncoder.getDistance());

    var chassisSpeeds = new ChassisSpeeds();
    if ((m_drivetrain.m_gyro.getAngleZ() > 85 && m_drivetrain.m_gyro.getAngleZ() < 95) == false && (m_drivetrain.m_gyro.getAngleZ() > 265 && m_drivetrain.m_gyro.getAngleZ() < 275) == false) {
      chassisSpeeds = new ChassisSpeeds(0, 0, 2);
    }
    else {
      // If statement for if the robot should move forward or backward
      if (m_pose.getX() > m_distance) {
        chassisSpeeds = new ChassisSpeeds(-.6, 0, 0);
      } 
      else if (m_pose.getX() < m_distance) {
        chassisSpeeds = new ChassisSpeeds(.6, 0, 0);
      }
    }
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    m_drivetrain.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.floor(m_pose.getX()) > m_distance - Constants.GO_TO_X_IS_FINISHED_ACCEPTABLE_DISTANCE_MARGIN 
      && Math.floor(m_pose.getX()) < m_distance + Constants.GO_TO_X_IS_FINISHED_ACCEPTABLE_DISTANCE_MARGIN;
  }
}
