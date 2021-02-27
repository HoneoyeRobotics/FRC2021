// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotatePID extends PIDCommand {
  
  static final double kP = 0.024;
  static final double kI = 0.04;
  static final double kD = 0.00;
  private DriveTrain driveTrain;
  private double setAngle;

  /** Creates a new RotatePID. */
  public RotatePID(DriveTrain m_DriveTrain, double m_setAngle) {
    super(
        // The controller that the command will use
        new PIDController(m_DriveTrain.getP(), m_DriveTrain.getI(), m_DriveTrain.getD()),
        // This should return the measurement
        () -> m_DriveTrain.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> m_setAngle,
        // This uses the output
        output -> {
          // Use the output here
          // if(output != 0)
          //   {
          //     if(output < 0 && output > -0.4) 
          //       output = -0.4;
          //     else if(output > 0 && output < 0.4)
          //       output = 0.4;
          //   }
            SmartDashboard.putNumber("pid output", output);
          m_DriveTrain.drive(0, output);
        });

    driveTrain = m_DriveTrain;
    setAngle = m_setAngle;
    getController().setTolerance(1);
    addRequirements(driveTrain);
    
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    
  }



  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    this.getController().setP(driveTrain.getP());
    
    this.getController().setI(driveTrain.getI());
  
    this.getController().setD(driveTrain.getD());
    atSetpointCheck = 0;
    ticks = 0;
  }
  private int atSetpointCheck = 0;
private int ticks = 0;
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean done = getController().atSetpoint();
    ticks++;
    SmartDashboard.putNumber("Ticks", ticks);
    if(done)
      atSetpointCheck++;

    SmartDashboard.putBoolean("At Setpoint?", done);
      
    SmartDashboard.putNumber("atSetpointCheck", atSetpointCheck);
    if (atSetpointCheck == 10) {
      return true;
    } 
    if(done == false) {
      atSetpointCheck = 0;
    }
    
    return false;
  }
}
