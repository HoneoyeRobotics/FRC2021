/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final SpeedController leftMotorGroup;
  private final SpeedController rightMotorGroup;

  private final WPI_VictorSPX leftFrontMotor;
  private final WPI_VictorSPX leftRearMotor;
  private final WPI_VictorSPX rightFrontMotor;
  private final WPI_VictorSPX rightRearMotor;
  private final DifferentialDrive drive;
  private final Encoder leftEncoder;
  private final Encoder rightEncoder;
  private final AHRS navx;
  private final DifferentialDriveOdometry odometry;
  private boolean reverseDirection=   false;

  public DriveTrain() {


    leftFrontMotor = new WPI_VictorSPX(Constants.CANID_FrontLeftDriveMotor);
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftRearMotor = new WPI_VictorSPX(Constants.CANID_RearLeftDriveMotor);
    leftRearMotor.setNeutralMode(NeutralMode.Brake);

    rightFrontMotor = new WPI_VictorSPX(Constants.CANID_FrontRightDriveMotor);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightRearMotor = new WPI_VictorSPX(Constants.CANID_RearRightDriveMotor);
    rightRearMotor.setNeutralMode(NeutralMode.Brake);
   
    if(Constants.RampWaitMode > 0) {
      leftFrontMotor.configOpenloopRamp(Constants.RampWaitMode);
      leftRearMotor.configOpenloopRamp(Constants.RampWaitMode);
      rightRearMotor.configOpenloopRamp(Constants.RampWaitMode);
      rightFrontMotor.configOpenloopRamp(Constants.RampWaitMode);
   
    }
    
    leftMotorGroup = new SpeedControllerGroup(leftFrontMotor, leftRearMotor);
    rightMotorGroup = new SpeedControllerGroup(rightFrontMotor, rightRearMotor);
    drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    // AHRS ahrs = new AHRS(SerialPort.Port.kMXP); /* Alternatives: SPI.Port.kMXP,
    // I2C.Port.kMXP or SerialPort.Port.kUSB */
    navx = new AHRS(SPI.Port.kMXP);
    leftEncoder = new Encoder(Constants.leftEncoderA, Constants.leftEncoderB);
    rightEncoder = new Encoder(Constants.rightEncoderA, Constants.rightEncoderB);
    leftEncoder.setDistancePerPulse(Constants.EncoderDistancePerPulse);
    leftEncoder.setReverseDirection(true);
    rightEncoder.setDistancePerPulse(Constants.EncoderDistancePerPulse);
    resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));


        Shuffleboard.getTab("Stats").addNumber("Pitch", () -> navx.getPitch());
    Shuffleboard.getTab("Stats").addNumber("Roll", () -> navx.getRoll());
    Shuffleboard.getTab("Stats").addNumber("Drive Left Encoder", () -> leftEncoder.get());
    Shuffleboard.getTab("Stats").addNumber("Drive Right Encoder", () -> rightEncoder.get());
    Shuffleboard.getTab("Stats").addNumber("Drive Left Encoder Dist", () -> leftEncoder.getDistance());
    Shuffleboard.getTab("Stats").addNumber("Drive Right Encoder Dist", () -> rightEncoder.getDistance());
    
    Shuffleboard.getTab("Stats").addNumber("Last Speed", () -> lastSpeed);
    Shuffleboard.getTab("Stats").addNumber("Last Rotation", () -> lastRotation);
  }

  double last_world_linear_accel_x;
  double last_world_linear_accel_y;

  final double kCollisionThreshold_DeltaG = 0.5f;

  public boolean collisionDetected() {
    boolean collisionDetected = false;

    double curr_world_linear_accel_x = navx.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = navx.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
        || (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
      collisionDetected = true;
    }
    SmartDashboard.putBoolean("CollisionDetected", collisionDetected);

    return collisionDetected;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry() {
    navx.reset();
    Pose2d zeroPose = new Pose2d(0, 0, new Rotation2d());
    resetOdometry(zeroPose);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorGroup.setVoltage(leftVolts);
    rightMotorGroup.setVoltage(-rightVolts);
    drive.feed();
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance());
    boolean motionDetected = navx.isMoving();
    SmartDashboard.putNumber("Rotation", navx.getAngle());
    SmartDashboard.putBoolean("Motion Detected", motionDetected);
  }

  private double lastSpeed = 0;
  private double lastRotation = 0;

  public void drive(double xSpeed, double zRotation) {
    lastSpeed = xSpeed;
    lastRotation = zRotation;
    drive.arcadeDrive(xSpeed, zRotation);
  }

  // public void drive(double leftSpeed, double rightSpeed) {
  // drive.tankDrive(leftSpeed,rightSpeed);
  // SmartDashboard.putNumber("rightSpeed", rightSpeed);
  // SmartDashboard.putNumber("leftSpeed", leftSpeed);

  // }

  public double getAngle() {
    return navx.getAngle();
  }

  public double rightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public double leftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  private void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void log() {
    // pee pee poo poo why are you reading this
    // did you know that cameras have a limited fov
  }


  public void switchDriveDirection(){
    this.reverseDirection = !reverseDirection;
  }

  public boolean isReversed(){
    return reverseDirection;
  }
}