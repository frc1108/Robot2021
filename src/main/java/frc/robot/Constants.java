/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants{

    // CAN IDs for Drive Motors
    public static final int CAN_ID_LEFT_DRIVE = 1;
    public static final int CAN_ID_RIGHT_DRIVE = 2;
    public static final int CAN_ID_LEFT_DRIVE_2 = 3;
    public static final int CAN_ID_RIGHT_DRIVE_2 = 4;

    // Encoder in NEO motors calculation
    public static final double kGearRatio = 8.45;
    public static final double kWheelDiameterMeters = 0.1525;
  }

  public static final class BallLauncherConstants {
    public static final int CAN_ID_BALL_LAUNCH_LEFT = 7;
    public static final int CAN_ID_BALL_LAUNCH_RIGHT = 8;
    public static final double ballLaunchSpeed = 0.5;
    public static final double kLaunchspeed = 4000; //in rpm
    
  }
  public static final class HopperConstants {
    public static final int KEL_LIMIT_SWITCH = 1; //upper limit switch
    public static final int GUS_LIMIT_SWITCH = 2; //lower limit switch
    public static final int CAN_ID_Hopper_Axle = 5; //Axle
  }

  public static final class IntakeConstants {
    public static final double hopperIntakeSpeed = -0.60; //hopper intake speed
    public static final double launcherIntakeSpeed = .38; //middle intake speed
    public static final int CAN_ID_Hopper_Intake = 6; //Intake
    public static final int CAN_ID_Launcher_Intake = 9; // launcher intake
  }

  public static final class ClimberConstants {
    public static final int CAN_ID_WINCH = 10; //Intake
    public static final double servoAngle = 90; //Released Servo angle
    public static final double servoCloseAngle = 120; //Closed Servo Angle
  }


  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class PWMConstants {
    public static final int PWM_ID_WINCH_SERVO = 1;
  }
}
