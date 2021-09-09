/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Add your docs here.
 */
public class ShootBalls extends CommandBase {

  private final LauncherSubsystem m_launcher;
  private long TimeToRun = 6000000; //microseconds
  private long initTime;


  public ShootBalls(LauncherSubsystem launcher){
    m_launcher = launcher;
    addRequirements(m_launcher);
  }
  
  @Override
  public void initialize(){
    initTime = RobotController.getFPGATime();
    m_launcher.start();   
  }

  @Override
  public void execute(){
  }

  @Override
  public void end(boolean interrupt){
    m_launcher.stop();
  }

  @Override
  public boolean isFinished(){
    if (RobotController.getFPGATime() - initTime <= TimeToRun) {
      return false;
    } else {
      return true;
    }
  } 

}
