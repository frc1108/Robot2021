
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.IntakeConstants.CAN_ID_Hopper_Intake;
import static frc.robot.Constants.IntakeConstants.CAN_ID_Launcher_Intake;

public class IntakeSubsystem extends SubsystemBase implements Loggable{

    WPI_VictorSPX _HopperIntake = new WPI_VictorSPX(CAN_ID_Hopper_Intake);
    WPI_VictorSPX _LauncherIntake = new WPI_VictorSPX(CAN_ID_Launcher_Intake);

    @Config.NumberSlider(name="Hopper Intake Speed", defaultValue = 1, tabName = "Other",
                         width = 5, height = 5, columnIndex = 2, rowIndex = 2, min = 0)
    public void IntakeMotor(double intake_spd){
        _HopperIntake.set(intake_spd);
        _LauncherIntake.set(-1);
    }

}