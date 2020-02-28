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

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class FeederSubsystem extends SubsystemBase {

  private WPI_VictorSPX m_feeder = new WPI_VictorSPX(FeederConstants.CAN_ID_Launcher_Intake);

  private double m_fastInSpeed = 0.7;
  private double m_slowInSpeed = 0.4;
  private double m_slowOutSpeed = -0.6;

  public FeederSubsystem(){
    m_feeder.set(0);
    m_feeder.configFactoryDefault();
    m_feeder.setNeutralMode(NeutralMode.Brake);
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

}
