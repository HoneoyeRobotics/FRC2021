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
    this.degreesChanged = degrees;
    this.speed = speed * 1.7;
    if(degrees < 0)
      modifier = -1;
  }

  private final double speed;
  private double degreesChanged = 90;
  private double startDegrees = 0;
  private double endDegrees = 0;
  //turning value is equal to the difference


  private int modifier = 1;
  

  @Override
  public void initialize() {
    
    startDegrees = m_drivetrain.getAngle();
    endDegrees = startDegrees + degreesChanged;
    SmartDashboard.putNumber("startDegrees", startDegrees);
    SmartDashboard.putNumber("endDegrees", endDegrees);
  }

  private double kAngleSetpoint = 0;
  private double  kP = 0.05;
  @Override
  public void execute() {

    double current = m_drivetrain.getAngle();
    double turningValue = (endDegrees - current) * kP;

    // if(current <= endDegrees) 
    //   turningValue = (endDegrees - current) * kP;
    // else
    //   (endDegrees + current) * kP;
    // Invert the direction of the turn if we are going backwards
    turningValue = Math.copySign(turningValue, speed) ;

    //turningValue = 0.625 * modifier;
    m_drivetrain.drive(0, turningValue);
    
    SmartDashboard.putNumber("turningValue", turningValue);
  }

  private final double diff = 5;

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    double current = m_drivetrain.getAngle();
    
    SmartDashboard.putNumber("current", current);
    //s=100; e=20; c=80
    //if(endDegrees <= startDegrees)
      return current <= endDegrees + diff  
          && current >= endDegrees - diff  ;
    // else
    // //s = 100; e=150; c=130; 147-153
    //   return current >= endDegrees + diff  
    //   && endDegrees >= endDegrees - diff  ;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }
}