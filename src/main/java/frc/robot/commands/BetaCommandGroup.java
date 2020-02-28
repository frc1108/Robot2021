/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToWall;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.HopperShift;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallLauncher;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BetaCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandGroup.
   */
  public BetaCommandGroup(DriveSubsystem drive, BallLauncher ball, FeederSubsystem feeder, HopperSubsystem hopper) {
    
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DriveToWall(drive), new HopperShift(hopper), new ShootBalls(ball, feeder));

  }
}
