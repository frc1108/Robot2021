/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BallLauncher;
import frc.robot.subsystems.FeederSubsystem;

/**
 * Add your docs here.
 */
public class ShootBalls extends CommandBase {

  private final BallLauncher m_launcher;
  private final FeederSubsystem m_feeder;

  public ShootBalls(BallLauncher launcher, FeederSubsystem feeder){
    m_launcher = launcher;
    m_feeder = feeder;
    addRequirements(m_launcher, m_feeder);
  }
  
  @Override
  public void initialize(){
    m_launcher.startLauncher();
    new WaitCommand(0.5);
    m_feeder.startFeeder();    
  }

  @Override
  public void execute(){
    withTimeout(4);
  }

  @Override
  public void end(boolean interrupt){
    m_launcher.stopLauncher();
    m_feeder.stopFeeder();
  }

  @Override
  public boolean isFinished(){
    return false;
  } 

}
