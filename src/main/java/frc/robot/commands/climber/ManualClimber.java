// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;
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
    m_climber.stopWinch();
  }

  @Override
  public boolean isFinished() {
   return false;
  }
}