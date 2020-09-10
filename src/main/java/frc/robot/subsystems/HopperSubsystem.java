
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import static frc.robot.Constants.HopperConstants.UPPER_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.LOWER_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.CAN_ID_Hopper_Axle;

public class HopperSubsystem extends SubsystemBase implements Loggable {

    private WPI_TalonSRX m_hopper = new WPI_TalonSRX(CAN_ID_Hopper_Axle);

    DigitalInput m_highLimit=new DigitalInput(UPPER_LIMIT_SWITCH); 
    DigitalInput m_lowLimit=new DigitalInput(LOWER_LIMIT_SWITCH); 

    Counter UpperCounter = new Counter(m_highLimit);
    Counter LowerCounter = new Counter(m_lowLimit);

    public HopperSubsystem(){
        stop();
        m_hopper.configFactoryDefault();
        m_hopper.configContinuousCurrentLimit(30);
        m_hopper.configPeakCurrentLimit(40);
        m_hopper.setInverted(false);
        m_hopper.setNeutralMode(NeutralMode.Brake);

        // Default command to stop() 
        this.setDefaultCommand(new RunCommand(() -> stop(), this));
    }
    
    @Log.BooleanBox(name = "Hopper Up", tabName = "Match View") 
    public boolean isHighSwitchSet() {
        return !m_highLimit.get();  // inverted switch logic
    }

    public void initializeHighCounter() {
        UpperCounter.reset();
    }
   
    @Log.BooleanBox(name = "Hopper Down", tabName = "Match View")
    public boolean isLowSwitchSet() {
        return !m_lowLimit.get();   // inverted switch logic
    }

    public void up(){
        if (m_highLimit.get()){
            m_hopper.set(0.7);
        }
    }

    public void down(){
        if (m_lowLimit.get()){
            m_hopper.set(-0.5);
        }
    }

    public void initializeLowCounter() {
        LowerCounter.reset();
    }

    public void HopperMotor(double hopper_spd){
       
        // temporary max speed
        double spd = -hopper_spd;

        if (Math.abs(spd) > 0.85){
            spd = 0.85*Math.signum(spd);  
        } 

        // temporary deadband
        if (Math.abs(spd) < 0.1){
            spd = 0;
        }
        
        // Logic needs to be tested to verify polarity is correct     
        if ((m_lowLimit.get() && spd < 0) || m_highLimit.get() && spd > 0 ) {
            m_hopper.set(spd);
        } else {
            m_hopper.set(0);
        }
    }

    public void stop(){
        m_hopper.stopMotor();
    }
}
