/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToWall extends CommandBase {
  private final DriveSubsystem m_drive;

  public DriveToWall(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.arcadeDrive(-0.4,0);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();

  }

  @Override
  public boolean isFinished() {
    return (m_drive.getSonarDistanceInches() < 7);
  }
}
