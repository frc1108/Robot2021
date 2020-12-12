/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {
        public Trajectory[] centerAuto8Cell = new Trajectory[4];
        private DriveSubsystem m_drive;

        public SneakyTrajectory(DriveSubsystem drive) {
                m_drive = drive;

                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, 10); 

                TrajectoryConfig configReversed = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint)
                                                .setReversed(true);

                TrajectoryConfig configForward = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);

                double divisor = 1.0;
                           
                centerAuto8Cell[0] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(2.11/divisor,5.82/divisor,new Rotation2d(3.14)),
                                        new Pose2d(0/divisor,5.82/divisor,new Rotation2d(3.14))), 
                                configForward);
                centerAuto8Cell[1] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(0/divisor,5.82/divisor,new Rotation2d(3.14)),
                                        new Pose2d(1.55/divisor,8.23/divisor,new Rotation2d(4.71))), 
                                configReversed);
                centerAuto8Cell[2] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(3.05/divisor,8.23/divisor,new Rotation2d(4.71)),
                                        new Pose2d(4.13/divisor,7.52/divisor,new Rotation2d(6))), 
                                configForward);
                centerAuto8Cell[3] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(9.53/divisor,7.52/divisor,new Rotation2d(3.14)),
                                        new Pose2d(4.83/divisor,7.52/divisor,new Rotation2d(3.14)),
                                        new Pose2d(1.78/divisor,6.71/divisor,new Rotation2d(3.56))), 
                                configForward);
        }

        public RamseteCommand getRamsete(Trajectory traj) {
                return new RamseteCommand(traj, m_drive::getPose,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drive::tankDriveVolts, m_drive);
        }

        public Command getInitialPose(Trajectory traj){
                return new InstantCommand(()->m_drive.resetOdometry(traj.getInitialPose()),m_drive).withTimeout(0.05);              
        }
}