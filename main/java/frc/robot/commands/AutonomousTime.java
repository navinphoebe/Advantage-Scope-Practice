// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousTime extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param drivetrain The drive subsystem on which this command will run
   */
  public AutonomousTime(Drivetrain drivetrain) {
    addCommands(
        new DriveTime(.1, 1.0, drivetrain),
        new DriveTime(.2, 1.0, drivetrain),
        new DriveTime(.3, 1.0, drivetrain),
        new DriveTime(.4, 1.0, drivetrain),
        new DriveTime(.5, 1.0, drivetrain),
        new DriveTime(.6, 1.0, drivetrain),
        new DriveTime(.7, 1.0, drivetrain),
        new DriveTime(.8, 1.0, drivetrain),
        new DriveTime(.9, 1.0, drivetrain),
        new DriveTime(1.0, 1.0, drivetrain),
        new DriveTime(-1.0, 1.0, drivetrain),
        new DriveTime(-0.9, 1.0, drivetrain),
        new DriveTime(-0.8, 1.0, drivetrain),
        new DriveTime(-0.7, 1.0, drivetrain),
        new DriveTime(-0.6, 1.0, drivetrain),
        new DriveTime(-0.5, 1.0, drivetrain),
        new DriveTime(-0.4, 1.0, drivetrain),
        new DriveTime(-0.3, 1.0, drivetrain),
        new DriveTime(-0.2, 1.0, drivetrain),
        new DriveTime(-0.1, 1.0, drivetrain));

  }
}
