/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.Loggable;


public class LightStrip extends SubsystemBase implements Loggable {
  // Create a new Light
  private AddressableLED m_strip;
  private AddressableLEDBuffer m_lightColors;
  private int ledCount;
  private DriverStation m_ds;
  private Timer m_t;

  /**
   * 
   * @param port
   * @param ledCount
   */
  public LightStrip(int port, int ledCount){
    this.m_ds = DriverStation.getInstance();
    this.m_strip = new AddressableLED(port);
    this.ledCount = ledCount;
    this.m_lightColors = new AddressableLEDBuffer(ledCount);
    m_strip.setLength(m_lightColors.getLength());
    m_strip.setData(m_lightColors);
    m_strip.start();
    this.m_t = new Timer();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set the lights to a alliance color
   */
  public void allianceColors(){
    int red = 0;
    int green = 0;
    int blue = 0;

    if(m_ds.getAlliance() != Alliance.Blue){
      red = 255;
      blue = 0;
    }
    else{
      blue = 255;
      red = 0;
    }
    for (int i=0; i<10; i++){
      m_lightColors.setRGB(i, red, green, blue);
      this.m_strip.setData(m_lightColors);
    }
  }
}
