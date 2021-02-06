/*----------------------------------------------------------------------------*/
/* CopyzRotation (c) 2018-2019 FIRST. All zRotations Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.util.function.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutoRotate extends CommandBase {
  private final DriveTrain m_drivetrain;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoRotate(DriveTrain drivetrain, double degrees, double speed) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    this.degrees = degrees;
    this.speed = speed;
    if(degrees < 0)
      modifier = -1;
  }
  private final double speed;
  private double degrees = 0;
  private double startDegrees = 0;
  private double endDegrees = 0;

  private int modifier = 1;
  

  @Override
  public void initialize() {
    
    startDegrees = m_drivetrain.getAngle();
    endDegrees = startDegrees + degrees;
    SmartDashboard.putNumber("startDegrees", startDegrees);
    SmartDashboard.putNumber("endDegrees", endDegrees);
  }

  private double kAngleSetpoint = 0;
  private double  kP = 0.05;
  @Override
  public void execute() {

    double turningValue = (endDegrees - m_drivetrain.getAngle()) * kP;
    // Invert the direction of the turn if we are going backwards
    turningValue = Math.copySign(turningValue, speed) * modifier;
    m_drivetrain.drive(0, turningValue);
    
    SmartDashboard.putNumber("turningValue", turningValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    double current = m_drivetrain.getAngle();
    
    SmartDashboard.putNumber("current", current);
    //s=100; e=20; c=80
    if(endDegrees <= startDegrees)
      return current <= endDegrees;
    else
      return current >= endDegrees;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }
}