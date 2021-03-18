// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ClimberSubsystem extends SubsystemBase implements Loggable {
  /** Creates a new ClimberSubsystem. */
  private final WPI_TalonSRX m_winch = new WPI_TalonSRX(ClimberConstants.CAN_ID_Climber_Motor);

  private double m_winchSpeed = 0.75;
  private boolean isEnabled;


  public ClimberSubsystem() {
// Initialize Talon SRX
m_winch.set(0);
m_winch.configFactoryDefault();
m_winch.setNeutralMode(NeutralMode.Brake);
m_winch.configContinuousCurrentLimit(40);
m_winch.configPeakCurrentLimit(60);
m_winch.setInverted(false);

/* // PID and Sensor values for Motion Magic
m_winch.setSensorPhase(true);
m_winch.config_kP(0,20);
m_winch.config_kI(0,0);
m_winch.config_kD(0,200);
m_winch.config_kF(0,0);
m_winch.configMotionAcceleration(1200);
m_winch.configMotionCruiseVelocity(900); */

isEnabled = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

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

  public void enable(){
    isEnabled = true;
  }


  public void startWinch(){
    if (true){
      m_winch.set(m_winchSpeed);
    }
  }

  public void stopWinch(){
    m_winch.set(0);
  }

  public void setWinchSpeed(double winchSpeed){
    m_winchSpeed = winchSpeed;
  }
 
  
  @Log(name = "Winch current",tabName = "Match View")
  public double getWinchCurrent(){
    return m_winch.getStatorCurrent();
  }
}
