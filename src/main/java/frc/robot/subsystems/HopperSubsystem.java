
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.DriveConstants.CAN_ID_BALL_LAUNCH_LEFT;
import static frc.robot.Constants.DriveConstants.CAN_ID_BALL_LAUNCH_RIGHT;

import static frc.robot.Constants.HopperConstants.KEL_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.GUS_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.CAN_ID_Hopper_Axle;
import static frc.robot.Constants.HopperConstants.CAN_ID_Hopper_Intake;

public class HopperSubsystem extends SubsystemBase {

    WPI_VictorSPX _HopperAxle = new WPI_VictorSPX(CAN_ID_Hopper_Axle);
    WPI_VictorSPX _HopperIntake = new WPI_VictorSPX(CAN_ID_Hopper_Intake);

     DigitalInput _HighSwitch=new DigitalInput(KEL_LIMIT_SWITCH); 
     DigitalInput _LowSwitch=new DigitalInput(GUS_LIMIT_SWITCH); 

     Counter UpperCounter = new Counter(_HighSwitch);
     Counter LowerCounter = new Counter(_LowSwitch);

     
    public boolean isHighSwitchSet() {
        return UpperCounter.get() > 0;
    }

    public void initializeHighCounter() {
        UpperCounter.reset();
    }
   
    public boolean isLowSwitchSet() {
        return LowerCounter.get() > 0;
    }

    public void initializeLowCounter() {
        LowerCounter.reset();
    }

    public void HopperMotor(double hopper_spd){
        _HopperAxle.set(hopper_spd);
    }

    public void IntakeMotor(double intake_spd){
        _HopperIntake.set(intake_spd);
    }
}