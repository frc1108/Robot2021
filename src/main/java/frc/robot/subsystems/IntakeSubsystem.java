
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase implements Loggable{

    @Log.SpeedController(name="Intake Motor")
    private WPI_VictorSPX m_intake = new WPI_VictorSPX(IntakeConstants.CAN_ID_Hopper_Intake);

    private double m_intakeSpeed;
    private double m_slowOutSpeed;

    public IntakeSubsystem(){
        m_intake.set(0);
        m_intake.configFactoryDefault();
        m_intake.setNeutralMode(NeutralMode.Coast);
    }

    @Config.NumberSlider(name = "Intake Speed",defaultValue = -0.6)
    public void setIntakeSpeed(double intakeSpeed){
        m_intakeSpeed = intakeSpeed;
    }

    @Config.NumberSlider(name = "Slow Out Speed", defaultValue = 0.2)
    public void setSlowOutSpeed(double slowOutSpeed){
        m_slowOutSpeed = slowOutSpeed;
    }

    public void startIntake(){
        m_intake.set(m_intakeSpeed);
    }

    public void stopIntake(){
        m_intake.set(0);
    }

    public void slowOutIntake(){
        m_intake.set(m_slowOutSpeed);
    }
}