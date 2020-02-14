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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.PWMConstants.PWM_ID_WINCH_SERVO;
import static frc.robot.Constants.ClimberConstants.CAN_ID_WINCH;
/**
 * Add your docs here.
 */
public class ClimberSubsystem extends SubsystemBase implements Loggable {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  Servo m_winchServo = new Servo(PWM_ID_WINCH_SERVO);
  WPI_TalonSRX _WinchMotor = new WPI_TalonSRX(CAN_ID_WINCH);
  public ClimberSubsystem() {

  }

  public void WinchServo(double angle) {
    m_winchServo.setAngle(angle);
  }
  public double AtAngle() {
    return m_winchServo.getAngle();
  }
  public void WinchMotor(double Winch_spd){
       
    // temporary max speed
    double spd = -Winch_spd;

    if (Math.abs(spd) > 0.3){
        spd = 0.3*Math.signum(spd);  
    } 

    // temporary deadband
    if (Math.abs(spd) < 0.1){
        spd = 0;
    }

    _WinchMotor.set(spd);
   }
/*   @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  } */
}
