
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    WPI_TalonSRX _HopperAxle = new WPI_TalonSRX(CAN_ID_Hopper_Axle);

    DigitalInput _HighSwitch=new DigitalInput(UPPER_LIMIT_SWITCH); 
    DigitalInput _LowSwitch=new DigitalInput(LOWER_LIMIT_SWITCH); 

    Counter UpperCounter = new Counter(_HighSwitch);
    Counter LowerCounter = new Counter(_LowSwitch);

    public HopperSubsystem(){
        _HopperAxle.set(0);
        _HopperAxle.configFactoryDefault();
        _HopperAxle.configContinuousCurrentLimit(30);
        _HopperAxle.configPeakCurrentLimit(40);
        _HopperAxle.setInverted(false);
        _HopperAxle.setNeutralMode(NeutralMode.Brake);
    }
    
    @Log.BooleanBox(name = "Hopper Up", tabName = "Match View") 
    public boolean isHighSwitchSet() {
        return !_HighSwitch.get();
    }

    public void initializeHighCounter() {
        UpperCounter.reset();
    }
   
    @Log.BooleanBox(name = "Hopper Down", tabName = "Match View")
    public boolean isLowSwitchSet() {
        return !_LowSwitch.get();
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
        if ((_LowSwitch.get() && spd < 0) || _HighSwitch.get() && spd > 0 ) {
            _HopperAxle.set(spd);
        } else {
            _HopperAxle.set(0);
        }
    }
}
