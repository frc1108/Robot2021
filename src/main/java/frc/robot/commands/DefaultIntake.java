/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * A command to drive the robot with joystick input
 */
public class DefaultIntake extends CommandBase {
  private final IntakeSubsystem m_ballcatcher;
  private final DoubleSupplier m_intakespeed;
  private final DoubleSupplier m_launcherintakespeed;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public DefaultIntake(IntakeSubsystem subsystem, DoubleSupplier intakemotor, DoubleSupplier launcherintakemotor) {
    m_ballcatcher = subsystem;
    m_intakespeed = intakemotor;
    m_launcherintakespeed = launcherintakemotor;
    addRequirements(m_ballcatcher);
  }

  @Override
  public void execute() {
    m_ballcatcher.IntakeMotor(m_intakespeed.getAsDouble());
    m_ballcatcher.LauncherIntakeMotor(m_launcherintakespeed.getAsDouble());
  }
}
