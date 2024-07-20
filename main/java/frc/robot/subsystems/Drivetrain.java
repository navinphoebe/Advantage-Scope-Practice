// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Derivative;

public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  private Timer timer = new Timer();

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);
  private final Derivative m_rightEncoderVelocity = new Derivative(30);
  private final Derivative m_leftEncoderVelocity = new Derivative(30);
   private final Derivative m_rightEncoderAccel = new Derivative(30);
  private final Derivative m_leftEncoderAccel = new Derivative(30);
  private final Derivative m_rightEncoderVelocity1 = new Derivative(50);
  private final Derivative m_leftEncoderVelocity1 = new Derivative(50);
   private final Derivative m_rightEncoderAccel1 = new Derivative(50);
  private final Derivative m_leftEncoderAccel1 = new Derivative(50);
  private final Derivative m_rightEncoderVelocity2 = new Derivative(10);
  private final Derivative m_leftEncoderVelocity2 = new Derivative(10);
   private final Derivative m_rightEncoderAccel2 = new Derivative(10);
  private final Derivative m_leftEncoderAccel2 = new Derivative(10);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  private final DifferentialDriveOdometry m_odometry;

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

   m_odometry = new DifferentialDriveOdometry(
    new Rotation2d(),
    m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
    new Pose2d(0, 0, new Rotation2d()));
    timer.start();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    System.out.println(timer.get());
    m_rightEncoderVelocity.periodic(m_rightEncoder.getDistance());
    m_leftEncoderVelocity.periodic(m_leftEncoder.getDistance());
    m_rightEncoderVelocity1.periodic(m_rightEncoder.getDistance());
    m_leftEncoderVelocity1.periodic(m_leftEncoder.getDistance());
    m_rightEncoderVelocity2.periodic(m_rightEncoder.getDistance());
    m_leftEncoderVelocity2.periodic(m_leftEncoder.getDistance());
    var gyroAngle = new Rotation2d(Math.toRadians(m_gyro.getAngleZ()));
    var m_pose = m_odometry.update(gyroAngle,
    m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());

    // This method will be called once per scheduler run
    double x = SmartDashboard.getNumber("PositionX", 0);
    x =  m_pose.getX();
    SmartDashboard.putNumber("PositionX", x);
    double y = SmartDashboard.getNumber("PositionY", 0);
    y = m_pose.getY();
    SmartDashboard.putNumber("PositionY", y);

    double left = SmartDashboard.getNumber("Position Left", 0);
    left =  m_leftEncoder.getDistance();
    SmartDashboard.putNumber("Position Left", left);
    double right = SmartDashboard.getNumber("Position Right", 0);
    right = m_rightEncoder.getDistance();
    SmartDashboard.putNumber("Position Right", right);

    double velocityLeft = SmartDashboard.getNumber("Velocity Left", 0);
    velocityLeft = m_leftEncoderVelocity.getNetAcceleration();
    SmartDashboard.putNumber("Velocity Left", velocityLeft);
    double velocityRight = SmartDashboard.getNumber("Velocity Right", 0);
    velocityRight = m_rightEncoderVelocity.getNetAcceleration();
    SmartDashboard.putNumber("Velocity Right", velocityRight);

    m_leftEncoderAccel.periodic(velocityLeft);
    m_rightEncoderAccel.periodic(velocityRight);

    double accelerationLeft = SmartDashboard.getNumber("Acceleration Left", 0);
    accelerationLeft = m_leftEncoderAccel.getNetAcceleration();
    SmartDashboard.putNumber("Acceleration Left", accelerationLeft);

    double accelerationRight = SmartDashboard.getNumber("Acceleration Right", 0);
    accelerationRight = m_rightEncoderAccel.getNetAcceleration();
    SmartDashboard.putNumber("Acceleration Right", accelerationRight);

    // five cycles

    double velocityLeft1 = SmartDashboard.getNumber("Velocity Left 5", 0);
    velocityLeft1 = m_leftEncoderVelocity1.getNetAcceleration();
    SmartDashboard.putNumber("Velocity Left 5", velocityLeft1);
    double velocityRight1 = SmartDashboard.getNumber("Velocity Right 5", 0);
    velocityRight1 = m_rightEncoderVelocity1.getNetAcceleration();
    SmartDashboard.putNumber("Velocity Right 5", velocityRight1);

    m_leftEncoderAccel1.periodic(velocityLeft1);
    m_rightEncoderAccel1.periodic(velocityRight1);

    double accelerationLeft1 = SmartDashboard.getNumber("Acceleration Left 5", 0);
    accelerationLeft1 = m_leftEncoderAccel1.getNetAcceleration();
    SmartDashboard.putNumber("Acceleration Left 5", accelerationLeft1);

    double accelerationRight1 = SmartDashboard.getNumber("Acceleration Right 5", 0);
    accelerationRight1 = m_rightEncoderAccel1.getNetAcceleration();
    SmartDashboard.putNumber("Acceleration Right 5", accelerationRight1);

    // ten cycles 

    double velocityLeft2 = SmartDashboard.getNumber("Velocity Left 10", 0);
    velocityLeft2 = m_leftEncoderVelocity2.getNetAcceleration();
    SmartDashboard.putNumber("Velocity Left 10", velocityLeft2);
    double velocityRight2 = SmartDashboard.getNumber("Velocity Right 10", 0);
    velocityRight2 = m_rightEncoderVelocity2.getNetAcceleration();
    SmartDashboard.putNumber("Velocity Right 10", velocityRight2);

    m_leftEncoderAccel2.periodic(velocityLeft2);
    m_rightEncoderAccel2.periodic(velocityRight2);

    double accelerationLeft2 = SmartDashboard.getNumber("Acceleration Left 10", 0);
    accelerationLeft2 = m_leftEncoderAccel2.getNetAcceleration();
    SmartDashboard.putNumber("Acceleration Left 10", accelerationLeft2);

    double accelerationRight2 = SmartDashboard.getNumber("Acceleration Right 10", 0);
    accelerationRight2 = m_rightEncoderAccel2.getNetAcceleration();
    SmartDashboard.putNumber("Acceleration Right 10", accelerationRight2);
  }
}
