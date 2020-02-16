/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.Loggable;
import frc.robot.subsystems.ClimberSubsystem;

public class MoveServo extends CommandBase implements Loggable {
  private final ClimberSubsystem m_climber;
  private final DoubleSupplier m_angle; 
  public MoveServo(ClimberSubsystem subsystem, DoubleSupplier angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_climber = subsystem;
    m_angle = angle;
    addRequirements(m_climber);
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_climber.WinchServo(m_angle.getAsDouble());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if((m_climber.AtAngle() >= (m_angle.getAsDouble()-1.5)) || (m_climber.AtAngle() <= (m_angle.getAsDouble()+1.5))) {
      return true;
    } else {
      return false;
    }
    
    
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

}
