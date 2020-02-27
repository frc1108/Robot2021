/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import static frc.robot.Constants.ClimberConstants.*;
/**
 * Add your docs here.
 */
public class ClimberSubsystem extends SubsystemBase implements Loggable {
  //Servo m_winchServo = new Servo(PWM_ID_WINCH_SERVO);

  private final DigitalInput m_switch = new DigitalInput(TURNER_SWITCH_PORT);

  @Log.SpeedController(name = "Winch Motor")
  private final WPI_TalonSRX m_winch = new WPI_TalonSRX(CAN_ID_WINCH);

  @Log.SpeedController(name = "Turner Motor")
  private final WPI_VictorSPX m_turner = new WPI_VictorSPX(CAN_ID_TURNER);


  private double m_winchSpeed = 0.75;
  private double m_turnSpeed  = 0.9;

  public ClimberSubsystem() {
    // Initialize Talon SRX
    m_winch.set(0);
    m_winch.configFactoryDefault();
    m_winch.setNeutralMode(NeutralMode.Brake);
    m_winch.configContinuousCurrentLimit(40);
    m_winch.configPeakCurrentLimit(60);
    m_winch.setInverted(false);
  }

  /* public void WinchServo(double angle) {
    m_winchServo.setAngle(angle);
  }
  public double AtAngle() {
    return m_winchServo.getAngle();
  } */
  public void WinchMotor(double Winch_spd){
       
    // temporary max speed
    double spd = -Winch_spd;

    if (Math.abs(spd) > 0.75){
        spd = 0.75*Math.signum(spd);  
    } 

    // temporary deadband
    if (Math.abs(spd) < 0.1){
        spd = 0;
    }

    m_winch.set(spd);
   }

  public void startTurn(){
    m_turner.set(m_turnSpeed);
  }

  public void stopTurn(){
    m_turner.set(0);
  }

  public void reverseTurn(){
    m_turner.set(-m_turnSpeed);
  }

  public void startWinch(){
    m_winch.set(m_winchSpeed);
  }

  public void stopWinch(){
    m_winch.set(0);
  }

  public void setWinchSpeed(double winchSpeed){
    m_winchSpeed = winchSpeed;
  }
  
  @Log
  public double getWinchCurrent(){
    return m_winch.getStatorCurrent();
  }

  @Log.BooleanBox(name= "Turner Switch",tabName = "Match View")
  public boolean isTurnerVertical(){
    return m_switch.get();
  }
}
