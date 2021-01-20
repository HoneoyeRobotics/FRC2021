/*----------------------------------------------------------------------------*/
/* CopyzRotation (c) 2018-2019 FIRST. All zRotations Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.util.function.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutoDriveForward extends CommandBase {
  private final DriveTrain m_drivetrain;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveForward(DriveTrain drivetrain, double distance, double speed) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    this.distance = distance;
    this.speed = speed;
  }
  private final double speed;
  private double distance = 0;
  private double startEncoder = 0;
  private double endEncoder = 0;

  @Override
  public void initialize() {
    startEncoder = m_drivetrain.leftEncoderDistance();
    endEncoder = startEncoder + distance;
    kAngleSetpoint = m_drivetrain.getAngle();
  }

  private double kAngleSetpoint = 0;
  private double  kP = 0.05;
  @Override
  public void execute() {

    double turningValue = (kAngleSetpoint - m_drivetrain.getAngle()) * kP;
    // Invert the direction of the turn if we are going backwards
    turningValue = Math.copySign(turningValue, speed);
    m_drivetrain.drive(speed, turningValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_drivetrain.leftEncoderDistance() > endEncoder;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }
}