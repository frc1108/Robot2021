
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase{

    //@Log.SpeedController(name="Intake Motor", tabName = "Testing")
    private WPI_VictorSPX m_intake = new WPI_VictorSPX(IntakeConstants.CAN_ID_Hopper_Intake);

    private double m_intakeSpeed = -0.75;
    private double m_slowOutSpeed = 0.4;

    public IntakeSubsystem(){
        stop();
        m_intake.configFactoryDefault();
        m_intake.setNeutralMode(NeutralMode.Coast);

        // Default command to stop() 
        this.setDefaultCommand(new RunCommand(() -> stop(), this).withName("Stop Intake"));
    }

    public void setIntakeSpeed(double intakeSpeed){
        m_intakeSpeed = intakeSpeed;
    }

    public void setSlowOutSpeed(double slowOutSpeed){
        m_slowOutSpeed = slowOutSpeed;
    }

    public void startIntake(){
        m_intake.set(m_intakeSpeed);
    }

    public void stop(){
        m_intake.stopMotor();
    }

    public void slowOutIntake(){
        m_intake.set(m_slowOutSpeed);
    }
}