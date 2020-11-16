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
  /* private long TimeToRun = 3000;
  private long initTime = RobotController.getFPGATime(); */

  /**
   * Creates a new DriveToWall.
   */
  public DriveToWall(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(-0.4,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //withTimeout(15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0,0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_drive.getSonarDistanceInches()<7);
    //return false;
    /* if (RobotController.getFPGATime() - initTime <= TimeToRun) {
      return false;
    } else {
      return true;
    } */
  }
}
