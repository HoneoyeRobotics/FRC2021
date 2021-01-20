/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;
// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.cscore.VideoMode.PixelFormat;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.Preferences;
// import edu.wpi.first.wpilibj.Sendable;
// import edu.wpi.first.wpilibj.controller.*;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import java.util.ResourceBundle.Control;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


//all command and subsystem imports
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain drivetrain;
  private final Climber climber;
  private final Camera camera;
  private final PowercellSystem powerCellSystem;
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick coDriverjoystick = new Joystick(1);
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {

    // construct subsystems
    camera = new Camera();
    drivetrain = new DriveTrain();
    climber = new Climber();
    powerCellSystem = new PowercellSystem();
    // default commands
    ArcadeDrive arcadeDrive = new ArcadeDrive(() -> driverJoystick.getRawAxis(1) * (drivetrain.isReversed() ? 1 : -1),
        () -> driverJoystick.getRawAxis(3) - driverJoystick.getRawAxis(2), drivetrain);
    drivetrain.setDefaultCommand(arcadeDrive);

    // Configure the button bindings
    configureCoDriverButtonBindings();
    configureDriverButtonBindings();

    powerCellSystem.setArmUp(true);

    // dashboard stuff
    Shuffleboard.getTab("Main").add(new ResetOdometry(drivetrain));
    
    Shuffleboard.getTab("Main").add(drivetrain);
    Shuffleboard.getTab("Commands").add(new StoreArm(powerCellSystem));
    Shuffleboard.getTab("Commands").add(new RaiseConveyer(powerCellSystem));
    Shuffleboard.getTab("Commands").add(new LowerConveyer(powerCellSystem));
    Shuffleboard.getTab("Commands").add(new DeployArm(powerCellSystem));
    Shuffleboard.getTab("Commands").add("ADF", new AutoDriveForward(drivetrain, -36, -0.75).withTimeout(6));

    // setup autonomous commands
    m_chooser.setDefaultOption("Score in Front", new AutoScoreInFront(drivetrain, powerCellSystem));
    m_chooser.addOption("Score from Left", new AutoScoreFromLeftSide(drivetrain, powerCellSystem));
    m_chooser.addOption("Score from Right", new AutoScoreFromRightSide(drivetrain, powerCellSystem));
    m_chooser.addOption("Drive past Base line", new AutoDrivePastBaseLine(drivetrain));

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Commands").add(m_chooser);

    }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureCoDriverButtonBindings() {
    // buttons for codriver joystick
    final JoystickButton leftBumper = new JoystickButton(coDriverjoystick, 5);
    final JoystickButton rightBumper = new JoystickButton(coDriverjoystick, 6);
    final JoystickButton backButton = new JoystickButton(coDriverjoystick, 7);
    final JoystickButton startButton = new JoystickButton(coDriverjoystick, 8);
    final JoystickButton yButton = new JoystickButton(coDriverjoystick, 4);

    // Connect the buttons to commands
    leftBumper.whileHeld(new GatherPowercells(powerCellSystem));
    rightBumper.whileHeld(new DepositPowercells(powerCellSystem));
    startButton.whileHeld(new ElevateClimber(climber));
    backButton.whileHeld(new RetractClimber(climber));
    yButton.whileHeld(new SpitPowercells(powerCellSystem));
  }

  private void configureDriverButtonBindings() {
    // bubttons being used on controller
    final JoystickButton aButton = new JoystickButton(driverJoystick, 1);
    final JoystickButton bButton = new JoystickButton(driverJoystick, 2);
    final JoystickButton xButton = new JoystickButton(driverJoystick, 3);
    xButton.whenPressed(new SwitchDriveAndCamera(drivetrain, camera));

    // buttons assigned to commands
    aButton.whileHeld(new EngageExtraLift(climber));
    bButton.whileHeld(new DisengageExtraLift(climber));
  }

  public void teleopInit() {
    // ConveyerPistonInitilize ConveyerPistonInitilize = new
    // ConveyerPistonInitilize(powerCellSystem);
    // ConveyerPistonInitilize.withTimeout(0.2).schedule(true);
  }

  public void teleopDisable() {
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}