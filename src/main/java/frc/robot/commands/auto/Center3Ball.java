/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToWall;
import frc.robot.commands.hopper.HopperShift;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.commands.shoot.WaitThenShoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Center3Ball extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandGroup.
   */
  public Center3Ball(DriveSubsystem drive, LauncherSubsystem ball, FeederSubsystem feeder, HopperSubsystem hopper) {
    super(new DriveToWall(drive), new WaitThenShoot(ball, feeder));
  }
  // new HopperShift(hopper), 
}
