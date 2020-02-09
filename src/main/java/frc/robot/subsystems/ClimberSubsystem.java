/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.PWMConstants.PWM_ID_WINCH_SERVO;

/**
 * Add your docs here.
 */
public class ClimberSubsystem extends SubsystemBase implements Loggable {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private Servo m_winchServo = new Servo(PWM_ID_WINCH_SERVO);
  private double m_servoAngle;
  public ClimberSubsystem() {
    m_winchServo.setAngle(80);
  }

  @Config(name="Servo Angle",defaultValueNumeric = 110)
  public void setServoAngle(double servoAngle){
    m_servoAngle = servoAngle;
  }
  public void changeAngle(){
    m_winchServo.setAngle(m_servoAngle);
  }
/*   @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  } */
}
