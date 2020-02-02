
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.HopperConstants.KEL_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.GUS_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.CAN_ID_Hopper_Axle;

public class HopperSubsystem extends SubsystemBase {

    WPI_TalonSRX _HopperAxle = new WPI_TalonSRX(CAN_ID_Hopper_Axle);

     DigitalInput _HighSwitch=new DigitalInput(KEL_LIMIT_SWITCH); 
     DigitalInput _LowSwitch=new DigitalInput(GUS_LIMIT_SWITCH); 

     //Counter UpperCounter = new Counter(_HighSwitch);
     //Counter LowerCounter = new Counter(_LowSwitch);

     
    public boolean isHighSwitchSet() {
        return _HighSwitch.get();
        //return UpperCounter.get() > 0;
    }

    public void initializeHighCounter() {
      //  UpperCounter.reset();
    }
   
    public boolean isLowSwitchSet() {
        return _LowSwitch.get();
        //return LowerCounter.get() > 0;
    }

    public void initializeLowCounter() {
        //LowerCounter.reset();
    }

    public void HopperMotor(double hopper_spd){
        if ((!_LowSwitch.get() && hopper_spd < 0) || !_HighSwitch.get() && hopper_spd > 0 ) {
            _HopperAxle.set(-hopper_spd);
        } else {
            _HopperAxle.set(0);
        }

    }
}