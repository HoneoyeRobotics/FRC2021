/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // can bus IDs
    public static final int CANID_PCM = 10;
    public static final int CANID_FrontLeftDriveMotor = 27;
    public static final int CANID_RearLeftDriveMotor = 28;

    public static final int CANID_FrontRightDriveMotor = 26;
    public static final int CANID_RearRightDriveMotor = 25;
    public static final int CANID_ConveyerMotor = 34;
    public static final int CANID_IntakeMotor = 33;
    public static final int CANID_ArmWheelMotor = 23;
    public static final int CANID_Spare = 22;

    // PCM relay IDS 0-7
    public static final int PCMID_ConveyerSoleniodForward = 2;
    public static final int PCMID_ConveyerSoleniodBackward = 3;
    public static final int PCMID_ControlPanelSoleniodForward = 3;
    public static final int PCMID_ContorlPanelSoleniodBackward = 5;
    public static final int PCMID_ScissorLiftForward = 0;
    public static final int PCMID_ScissorLiftBackward = 1;
    public static final int PCMID_ExtraLiftEngaged = 4;
    public static final int PCMID_ExtraLiftDisengaged = 5;

    public static final int leftEncoderA = 0;
    public static final int leftEncoderB = 1;
    public static final int rightEncoderA = 4;
    public static final int rightEncoderB = 5;

    public static final double RampWaitMode = 0;//0.35;

    public static final double autoParkFwdSpeed = -0.40;
    public static final double autoParkRotateSpeed = 0.52;
    public static final double autoParkDuration = 1.5;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.
    public static final double Analysis_ks = 1.02;
    public static final double Analysis_kv = 0.0199;
    public static final double Analysis_ka = 0.00127;
    public static final double Analysis_kTrackwidth = 21.5; // 89.74643536965448;
    public static final double ControllerGain_kP = 0.0541;// 17.9;//3.79;
    public static final double ControllerGain_kD = 0;
    public static final double wheelDiameter = 6;

    public static final double EncoderDistancePerPulse = (wheelDiameter * Math.PI) / 360.0;
    public static final double kMaxSpeed = 12;
    public static final double kMaxAcceleration = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // first run at it
    // public static final double Analysis_ks = 0.833;
    // public static final double Analysis_kv = 3.37;
    // public static final double Analysis_ka = 0.6;
    // public static final double Analysis_kTrackwidth = 0.56529628924866;
    // public static final double ControllerGain_kP = 0.05;//17.9;//3.79;
    // public static final double ControllerGain_kD = 0;
    // public static final double wheelDiameter = 0.152;

    // public static final double EncoderDistancePerPulse = (wheelDiameter *
    // Math.PI) / 360.0;
    // public static final double kMaxSpeed = 2;
    // public static final double kMaxAcceleration = 0.5;
    // public static final double kRamseteB = 2;
    // public static final double kRamseteZeta = 0.7;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            Constants.Analysis_kTrackwidth);
}