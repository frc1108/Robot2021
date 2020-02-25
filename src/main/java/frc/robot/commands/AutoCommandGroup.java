/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToWall;
import frc.robot.commands.AutoBalls;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallLauncher;
import frc.robot.subsystems.FeederSubsystem;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoCommandGroup extends SequentialCommandGroup {
  private static final DriveSubsystem m_drive = new DriveSubsystem();
  private static final BallLauncher m_launcher = new BallLauncher();
  private static final FeederSubsystem m_feeder = new FeederSubsystem();
  /**
   * Creates a new AutoCommandGroup.
   */
  public AutoCommandGroup() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DriveToWall(m_drive), new AutoBalls(m_launcher, m_feeder));
  }
}
