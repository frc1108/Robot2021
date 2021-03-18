
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import static frc.robot.Constants.HopperConstants.UPPER_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.LOWER_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.CAN_ID_Hopper_Axle;

public class HopperSubsystem extends SubsystemBase implements Loggable {

  private WPI_TalonSRX m_motor = new WPI_TalonSRX(CAN_ID_Hopper_Axle);

  DigitalInput m_highLimit=new DigitalInput(UPPER_LIMIT_SWITCH); 
  DigitalInput m_lowLimit=new DigitalInput(LOWER_LIMIT_SWITCH); 

  public HopperSubsystem(){
    stop();
    m_motor.configFactoryDefault();
    m_motor.configContinuousCurrentLimit(30);
    m_motor.configPeakCurrentLimit(40);
    m_motor.setInverted(false);
    m_motor.setNeutralMode(NeutralMode.Brake);
    // Default command to stop() 
    this.setDefaultCommand(new RunCommand(() -> stop(), this).withName("Stop"));
  }

  /**
   \* 
   * @param speed
   */
  public void set(double speed) {
    speed = ((speed < 0 && getLowSwitch()) || (speed > 0 && getHighSwitch()))?0:speed;
    m_motor.set(speed);
  }
     
  @Log.BooleanBox(name = "Hopper Up", tabName = "Live") 
  public boolean getHighSwitch() {
    return !m_highLimit.get();  // inverted switch logic
  }

  @Log.BooleanBox(name = "Hopper Down", tabName = "Live")
  public boolean getLowSwitch() {
    return !m_lowLimit.get();   // inverted switch logic
  }

  public void up(){
      set(0.7);
  }

  public void down(){
    set(-0.5);
  }

  public void stop(){
    m_motor.stopMotor();
  }
}
