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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import static frc.robot.Constants.PWMConstants.PWM_ID_WINCH_SERVO;
import static frc.robot.Constants.ClimberConstants.CAN_ID_WINCH;
/**
 * Add your docs here.
 */
public class ClimberSubsystem extends SubsystemBase implements Loggable {
  @Log
  Servo m_winchServo = new Servo(PWM_ID_WINCH_SERVO);
  @Log.SpeedController(name = "Winch Motor")
  private final WPI_TalonSRX m_winch = new WPI_TalonSRX(CAN_ID_WINCH);

  private double m_winchSpeed;

  public ClimberSubsystem() {
    // Initialize Talon SRX
    m_winch.set(0);
    m_winch.configFactoryDefault();
    m_winch.setNeutralMode(NeutralMode.Brake);
    m_winch.configContinuousCurrentLimit(40);
    m_winch.configPeakCurrentLimit(60);
    m_winch.setInverted(false);
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

    m_winch.set(spd);
   }

  public void startWinch(){
    m_winch.set(m_winchSpeed);
  }

  public void stopWinch(){
    m_winch.set(0);
  }

  @Config(name = "Winch Speed",defaultValueNumeric = 0.5)
  public void setWinchSpeed(double winchSpeed){
    m_winchSpeed = winchSpeed;
  }
  
  @Log
  public double getWinchCurrent(){
    return m_winch.getStatorCurrent();
  }
}
