/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightsSubsystem;

public class SetRainbow extends CommandBase {
  /**
   * Creates a new SetRainbow. This command will make the underglow leds cycle through a rainbow.
   */
  private LightsSubsystem m_lightsSubsystem;

  
  public SetRainbow(LightsSubsystem lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lightsSubsystem = lights;
    addRequirements(m_lightsSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lightsSubsystem.rainbow();
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
