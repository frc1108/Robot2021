/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import static frc.robot.Constants.PWMConstants.PWM_ID_LEDS;

/**
 * Add your docs here.
 */
public class LightsSubsystem extends SubsystemBase implements Loggable {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private AddressableLED m_led = new AddressableLED(PWM_ID_LEDS);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(240);
  private int m_rainbowFirstPixelHue;
  

  public LightsSubsystem () {
    m_led.setLength(m_ledBuffer.getLength());
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

  public void setRainbow () {

  }


}
