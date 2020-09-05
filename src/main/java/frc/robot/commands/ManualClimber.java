/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimberSubsystem;

public class ManualClimber extends CommandBase {
  private final ClimberSubsystem m_climber;
  private final DoubleSupplier m_winchSpeed;
  /**
   */
  public ManualClimber(ClimberSubsystem subsystem, DoubleSupplier speed) {
    m_climber = subsystem;
    m_winchSpeed = speed;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
   m_climber.setWinchSpeed(m_winchSpeed.getAsDouble());
   m_climber.startWinch();
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  @Override
  public boolean isFinished() {
   return false;
  }
}
