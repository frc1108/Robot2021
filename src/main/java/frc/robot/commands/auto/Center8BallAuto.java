/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SneakyTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.BallLauncher;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drive.FieldOrientedTurn;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Center8BallAuto extends SequentialCommandGroup {
  /**
   * Creates a new Center8BallAuto.
   */
  public Center8BallAuto(
    SneakyTrajectory s_trajectory,
    DriveSubsystem drive, 
    HopperSubsystem m_hopper, 
    BallLauncher m_shooter
  ) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(
          //new InstantCommand(() -> m_hopper.down(),m_hopper).withTimeout(0.1).withInterrupt(m_hopper::isHighSwitchNotSet),
          s_trajectory.getInitialPose(s_trajectory.centerAuto8Cell[0]),
          s_trajectory.getRamsete(s_trajectory.centerAuto8Cell[0]),
          new RunCommand(() -> m_hopper.down(),m_hopper).withTimeout(0.3)
          .andThen(() -> m_hopper.stop()),
          //.withInterrupt(() -> true),
          s_trajectory.getRamsete(s_trajectory.centerAuto8Cell[1]),
          new FieldOrientedTurn(180, drive)
          //s_trajectory.getRamsete(s_trajectory.centerAuto8Cell[2])
          .andThen(() -> drive.arcadeDrive(0, 0))
         );
  }
}
