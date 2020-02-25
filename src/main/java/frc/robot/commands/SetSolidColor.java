/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightsSubsystem;

public class SetSolidColor extends CommandBase {
  /**
   * Creates a new SetLightsRed
   */
  private LightsSubsystem m_lightsSubsystem = new LightsSubsystem();
  private int m_red;
  private int m_green;
  private int m_blue;
  
  public SetSolidColor(int red, int green, int blue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_lightsSubsystem);
    m_red = red;
    m_green = green;
    m_blue = blue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lightsSubsystem.setSolidColor(m_red, m_green, m_blue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
