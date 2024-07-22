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


  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  public final Encoder m_leftEncoder = new Encoder(4, 5);
  public final Encoder m_rightEncoder = new Encoder(6, 7);
  private final Derivative m_rightEncoderVelocity = new Derivative(3);
  private final Derivative m_leftEncoderVelocity = new Derivative(3);
   private final Derivative m_rightEncoderAccel = new Derivative(3);
  private final Derivative m_leftEncoderAccel = new Derivative(3);
  private final Derivative m_rightEncoderVelocity1 = new Derivative(5);
  private final Derivative m_leftEncoderVelocity1 = new Derivative(5);
   private final Derivative m_rightEncoderAccel1 = new Derivative(5);
  private final Derivative m_leftEncoderAccel1 = new Derivative(5);
  private final Derivative m_rightEncoderVelocity2 = new Derivative(10);
  private final Derivative m_leftEncoderVelocity2 = new Derivative(10);
   private final Derivative m_rightEncoderAccel2 = new Derivative(10);
  private final Derivative m_leftEncoderAccel2 = new Derivative(10);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  public final XRPGyro m_gyro = new XRPGyro();

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
    // Uses (I think) encoder ticks
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

   m_odometry = new DifferentialDriveOdometry(
    new Rotation2d(),
    m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
    new Pose2d(0, 0, new Rotation2d()));
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    System.out.println("tank");
    System.out.println(leftSpeed);
    System.out.println(rightSpeed);
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
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

  private void updateEncoderPeriodics(){
    m_rightEncoderVelocity.periodic(m_rightEncoder.getDistance());
    m_leftEncoderVelocity.periodic(m_leftEncoder.getDistance());
    m_rightEncoderVelocity1.periodic(m_rightEncoder.getDistance());
    m_leftEncoderVelocity1.periodic(m_leftEncoder.getDistance());
    m_rightEncoderVelocity2.periodic(m_rightEncoder.getDistance());
    m_leftEncoderVelocity2.periodic(m_leftEncoder.getDistance());
  }

  public void updateEncoderDashboardValue(String string, Encoder encoder){
    double num = SmartDashboard.getNumber(string, 0);
    num =  encoder.getDistance();
    SmartDashboard.putNumber(string, num);
  }

  private void updateEncoderAccelAndVelocity(String string, Derivative m_leftEncoderVelocity3,
      Derivative m_rightEncoderVelocity3, Derivative m_leftEncoderAccel3, Derivative m_rightEncoderAccel3) {
    double velocityLeft = SmartDashboard.getNumber("Velocity Left" + string, 0);
    velocityLeft = m_leftEncoderVelocity3.getNetAcceleration();
    SmartDashboard.putNumber("Velocity Left" + string, velocityLeft);
    double velocityRight = SmartDashboard.getNumber("Velocity Right" + string, 0);
    velocityRight = m_rightEncoderVelocity3.getNetAcceleration();
    SmartDashboard.putNumber("Velocity Right" + string, velocityRight);

    m_leftEncoderAccel3.periodic(velocityLeft);
    m_rightEncoderAccel3.periodic(velocityRight);

    double accelerationLeft = SmartDashboard.getNumber("Acceleration Left" + string, 0);
    accelerationLeft = m_leftEncoderAccel3.getNetAcceleration();
    SmartDashboard.putNumber("Acceleration Left" + string, accelerationLeft);

    double accelerationRight = SmartDashboard.getNumber("Acceleration Right" + string, 0);
    accelerationRight = m_rightEncoderAccel3.getNetAcceleration();
    SmartDashboard.putNumber("Acceleration Right" + string, accelerationRight);
  }

  @Override
  public void periodic() {
    updateEncoderPeriodics();
    var gyroAngle = new Rotation2d(Math.toRadians(m_gyro.getAngleZ()));
    var m_pose = m_odometry.update(gyroAngle,
    m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());

    double x = SmartDashboard.getNumber("PositionX", 0);
    x =  m_pose.getX();
    SmartDashboard.putNumber("PositionX", x);
    double y = SmartDashboard.getNumber("PositionY", 0);
    y = m_pose.getY();
    SmartDashboard.putNumber("PositionY", y);

    updateEncoderDashboardValue("Position Left", m_leftEncoder);
    updateEncoderDashboardValue("Position Right", m_rightEncoder);
    
    updateEncoderAccelAndVelocity("", m_leftEncoderVelocity, m_rightEncoderVelocity,
     m_leftEncoderAccel, m_rightEncoderAccel);
    
    updateEncoderAccelAndVelocity("5", m_leftEncoderVelocity1, m_rightEncoderVelocity1,
     m_leftEncoderAccel1, m_rightEncoderAccel1);
    
    updateEncoderAccelAndVelocity("10", m_leftEncoderVelocity2, m_rightEncoderVelocity2,
     m_leftEncoderAccel2, m_rightEncoderAccel2);
    }
}
