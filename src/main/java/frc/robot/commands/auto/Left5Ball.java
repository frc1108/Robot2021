package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Left5Ball extends SequentialCommandGroup {
  public Left5Ball(DriveSubsystem m_robotDrive) {        
      TrajectoryConfig forwardConfig = new TrajectoryConfig(1, 2);
      TrajectoryConfig reverseConfig = new TrajectoryConfig(1, 2).setReversed(true);
      
      
      Trajectory left5Ball1 = m_robotDrive.generateTrajectory("Left5Ball1", forwardConfig);
      Trajectory left5Ball2 = m_robotDrive.generateTrajectory("Left5Ball2", reverseConfig);

      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(left5Ball1.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(left5Ball1, false).withTimeout(50).withName("Left5Ball1"),
          m_robotDrive.createCommandForTrajectory(left5Ball2, false).withTimeout(50).withName("Left5Ball2")
      );
  }
}