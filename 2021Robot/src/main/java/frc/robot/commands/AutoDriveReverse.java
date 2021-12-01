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
public class AutoDriveReverse extends CommandBase {
  private final DriveTrain m_drivetrain;

  //start of test 

//   static final double kP = 0.024;
//   static final double kI = 0.04;
//   static final double kD = 0.00;
//   private DriveTrain driveTrain;
//   private double setAngle;

//   /** Creates a new RotatePID. 
//  * @return */
//   public void RotatePID(DriveTrain m_DriveTrain, double m_setAngle) {
//     super(
//         // The controller that the command will use
//         new PIDController(m_DriveTrain.getP(), m_DriveTrain.getI(), m_DriveTrain.getD()),
//         // This should return the measurement
//         () -> m_DriveTrain.getAngle(),
//         // This should return the setpoint (can also be a constant)
//         () -> m_setAngle,
//         // This uses the output
//         output -> {
//           // Use the output here
//           SmartDashboard.putNumber("pid output", output);
//           m_DriveTrain.drive(0, output);
//         });

//     driveTrain = m_DriveTrain;
//     setAngle = m_setAngle;
    
//     addRequirements(driveTrain);
//         //end of test 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveReverse(DriveTrain drivetrain, double distance, double speed) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    this.distance = distance;
    this.speed = speed;
    this.stephenSpeed = speed * 1.5;
  }
  
  private final double speed;
  private final double stephenSpeed;
  private double distance = 0;
  private double startEncoder = 0;
  private double endEncoder = 0;

  @Override
  public void initialize() {
    startEncoder = m_drivetrain.leftEncoderDistance();
    endEncoder = startEncoder - distance;
    kAngleSetpoint = m_drivetrain.getAngle();
  }

  private double kAngleSetpoint = 0;
  private double kP = 0.05;

  @Override
  public void execute() {

    double turningValue = (kAngleSetpoint - m_drivetrain.getAngle()) * kP;
    // Invert the direction of the turn if we are going backwards
    // if(speed <= 0) {

    // }
    
    double driveSpeed = stephenSpeed * -1;
    turningValue = Math.copySign(turningValue, driveSpeed);
    m_drivetrain.drive(driveSpeed, turningValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_drivetrain.leftEncoderDistance() < endEncoder;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }
}