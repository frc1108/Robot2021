/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.feeder.WaitThenFeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class WaitThenShoot extends ParallelCommandGroup {
  /**
   * Creates a new ShootBallsGroup.
   */

  public WaitThenShoot(LauncherSubsystem launcher, FeederSubsystem feeder) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(new WaitThenFeed(feeder), new ShootBalls(launcher));
  }
}
