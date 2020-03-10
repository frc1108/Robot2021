/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCustom extends CommandBase {
  private final DriveSubsystem m_drive;
  private double forward;
  private double rotation;
  private long TimeToRun; //microseconds
  private long initTime;
  /**
   * Creates a new DriveCustom
   */
  public DriveCustom(DriveSubsystem subsystem, double fwd, double rot, long milleseconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    forward = fwd;
    rotation = rot;
    TimeToRun = milleseconds*1000;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = RobotController.getFPGATime();
    m_drive.arcadeDrive(forward, rotation);
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
    if (RobotController.getFPGATime() - initTime <= TimeToRun) {
      return false;
    } else {
      return true;
    }
  }
}
