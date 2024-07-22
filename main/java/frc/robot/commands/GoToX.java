// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class GoToX extends Command {
  /** Creates a new GoToX. */
  private Drivetrain m_drivetrain;
  private double m_distance;
  private DifferentialDriveOdometry m_odometry;
  public Pose2d m_pose = new Pose2d();

  public GoToX(Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_distance = distance;
    m_drivetrain = drivetrain;
    m_odometry = new DifferentialDriveOdometry(
    new Rotation2d(),
    m_drivetrain.m_leftEncoder.getDistance(), m_drivetrain.m_rightEncoder.getDistance(),
    new Pose2d(0, 0, new Rotation2d()));
    var gyroAnglee = new Rotation2d(Math.toRadians(m_drivetrain.m_gyro.getAngleZ()));
    m_pose = m_odometry.update(gyroAnglee,
    m_drivetrain.m_leftEncoder.getDistance(),
    m_drivetrain.m_rightEncoder.getDistance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* var gyroAngle = new Rotation2d(Math.toRadians(m_drivetrain.m_gyro.getAngleZ()));
    m_pose = m_odometry.update(gyroAngle,
    m_drivetrain.m_leftEncoder.getDistance(),
    m_drivetrain.m_rightEncoder.getDistance());
    new TurnDegrees(.75, 90 - m_drivetrain.m_gyro.getAngleZ(), m_drivetrain); */
    System.out.println(m_drivetrain.m_gyro.getAngleZ());
    if (!(m_drivetrain.m_gyro.getAngleZ() > 85 && m_drivetrain.m_gyro.getAngleZ() < 95) && !(m_drivetrain.m_gyro.getAngleZ() > 265 && m_drivetrain.m_gyro.getAngleZ() < 275)){
      m_drivetrain.arcadeDrive(.1, .5);
    } else {
      if (m_pose.getX() > m_distance){
        m_drivetrain.arcadeDrive(-.75, 0);
      } else {
        m_drivetrain.arcadeDrive(.75, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.floor(m_pose.getX()) == m_distance;
  }
}
