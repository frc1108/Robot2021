/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class FeederSubsystem extends SubsystemBase implements Loggable {

  @Log.SpeedController(name = "Feeder Motor")
  private WPI_VictorSPX m_feeder = new WPI_VictorSPX(FeederConstants.CAN_ID_Launcher_Intake);

  private double m_feederSpeed;
  private double m_slowOutSpeed;

  public FeederSubsystem(){
    m_feeder.set(0);
    m_feeder.configFactoryDefault();
    m_feeder.setNeutralMode(NeutralMode.Brake);
  }

  @Config.NumberSlider(name = "Feeder Speed", defaultValue = 0.38)
  public void setFeederSpeed(double feederSpeed){
    m_feederSpeed = feederSpeed;
  }

  @Config.NumberSlider(name = "Slow Out Speed", defaultValue = -0.2)
  public void setSlowOutSpeed(double slowOutSpeed){
      m_slowOutSpeed = slowOutSpeed;
  }

  public void startFeeder(){
    m_feeder.set(m_feederSpeed);
  }

  public void stopFeeder(){
    m_feeder.set(0);
  }

  public void slowOutFeeder(){
    m_feeder.set(m_slowOutSpeed);
}

}
