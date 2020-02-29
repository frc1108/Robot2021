/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

/**
 * Add your docs here.
 */
public class FeederTimed extends CommandBase {

  private final FeederSubsystem m_feeder;
  private long TimeToRun = 4000000; //microseconds
  private long initTime;


  public FeederTimed(FeederSubsystem feeder){
    m_feeder = feeder;
    addRequirements(m_feeder);
  }
  
  @Override
  public void initialize(){
    initTime = RobotController.getFPGATime();
    m_feeder.fastInFeeder();
  }

  @Override
  public void execute(){
  }

  @Override
  public void end(boolean interrupt){
    m_feeder.stopFeeder();
  }

  @Override
  public boolean isFinished(){
    //return false;
    if (RobotController.getFPGATime() - initTime <= TimeToRun) {
      return false;
    } else {
      return true;
    }
  } 

}
