/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class FeederSubsystem extends SubsystemBase {

  private WPI_VictorSPX m_feeder = new WPI_VictorSPX(FeederConstants.CAN_ID_Launcher_Intake);

  private double m_fastInSpeed = 0.7;
  private double m_slowInSpeed = 0.4;
  private double m_slowOutSpeed = -0.6;
  private final double m_feederRPM = -500;

  public FeederSubsystem(){
    m_feeder.set(0);
    m_feeder.configFactoryDefault();
    m_feeder.setNeutralMode(NeutralMode.Brake);

    m_feeder.config_kF(0, 0.1225);
    m_feeder.config_kP(0,1);
    m_feeder.config_kI(0,0);
    m_feeder.config_kD(0,0);

    // Config Talon Tach
    final int kTimeoutMs = 30;
    m_feeder.configSelectedFeedbackSensor(FeedbackDevice.Tachometer, 0, kTimeoutMs);
    int edgesPerCycle = 12;
    m_feeder.configPulseWidthPeriod_EdgesPerRot(edgesPerCycle, kTimeoutMs);
    int filterWindowSize = 1;
    m_feeder.configPulseWidthPeriod_FilterWindowSz(filterWindowSize, kTimeoutMs);
  }
 
  @Log.Dial(name = "Launcher RPM", tabName = "Match View", max = 200)
  public double getTachRPM(){
      double tachVel_UnitsPer100ms = m_feeder.getSelectedSensorVelocity(0);
      return -1*tachVel_UnitsPer100ms*600/1024;
  }


  public void setFastInSpeed(double fastInSpeed){
    m_fastInSpeed = fastInSpeed;
  }

  public void setSlowOutSpeed(double slowOutSpeed){
    m_slowOutSpeed = slowOutSpeed;
  }

  public void setSlowInSpeed(double slowInSpeed){
    m_slowInSpeed = slowInSpeed;
  }

  public void fastInFeeder(){
    m_feeder.set(m_fastInSpeed);
  }

  public void stopFeeder(){
    m_feeder.set(0);
  }

  public void slowInFeeder(){
    m_feeder.set(m_slowInSpeed);
  }

  public void slowOutFeeder(){
    m_feeder.set(m_slowOutSpeed);
  }

  public void startPIDLauncher(){      
    m_feeder.set(ControlMode.Velocity,m_feederRPM*1024/600);
  }

}
