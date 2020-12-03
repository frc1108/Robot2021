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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LightsSubsystem extends SubsystemBase{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private int ledCount;
    private DriverStation m_ds;
   
    public LightsSubsystem(int port, int ledCount) {
        this.m_ds = DriverStation.getInstance();
        this.m_led = new AddressableLED(port);
        this.m_ledBuffer = new AddressableLEDBuffer(ledCount);
        this.m_led.setLength(m_ledBuffer.getLength());
        this.m_led.setData(m_ledBuffer);
        this.m_led.start();
    }

    public void allianceColors(){
        int red, blue, green = 0;
        
        if(m_ds.getAlliance() != Alliance.Blue) {
            red = 255;
            blue = 0;
        } else {
            blue = 255;
            red = 0;
        }

        for (int i=0; i<10; i++) {
            m_ledBuffer.setRGB(i, red, green, blue);
            this.m_led.setData(m_ledBuffer);
        }
    }

    public void setLights (AddressableLEDBuffer Buffer) {
      m_led.setData(m_ledBuffer);
      m_led.start();
    }

    public void setSolidColor (int red, int green, int blue) {
      for (int i=0; i<m_ledBuffer.getLength();i++){
        m_ledBuffer.setRGB(i,red,green,blue);
        m_led.setData(m_ledBuffer);
        m_led.start();  
      }
    }

      public void rainbow(){
      for (var i=0; i<m_ledBuffer.getLength();i++){
          final var hue = (m_rainbowFirstPixelHue + (i*180/m_ledBuffer.getLength()))%180;
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
      } 
}
