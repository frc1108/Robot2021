
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.BallLauncherConstants.CAN_ID_BALL_LAUNCH_LEFT;
import static frc.robot.Constants.BallLauncherConstants.CAN_ID_BALL_LAUNCH_RIGHT;

public class BallLauncher extends SubsystemBase implements Loggable {

    @Log.SpeedController(name = "Launcher L Motor")
    private WPI_TalonSRX m_leftBallThrow = new WPI_TalonSRX(CAN_ID_BALL_LAUNCH_LEFT);

    @Log.SpeedController(name = "Launcher R Motor")
    private WPI_TalonSRX m_rightBallThrow = new WPI_TalonSRX(CAN_ID_BALL_LAUNCH_RIGHT);

    private double m_rightMotorSpeed;
    private double m_leftMotorSpeed;

    public BallLauncher(){
        m_leftBallThrow.set(0);
        m_leftBallThrow.configFactoryDefault();
        m_leftBallThrow.setNeutralMode(NeutralMode.Coast);
        m_leftBallThrow.configContinuousCurrentLimit(30);
        m_rightBallThrow.set(0);
        m_rightBallThrow.configFactoryDefault();
        m_rightBallThrow.setNeutralMode(NeutralMode.Coast);
        m_rightBallThrow.configContinuousCurrentLimit(30);
    }

    @Config
    public void setLauncherSpeed(
                @Config.NumberSlider(name = "Left Motor Speed", defaultValue = 0.75) double leftMotorSpeed,
                @Config.NumberSlider(name = "Right Motor Speed", defaultValue = -0.75) double rightMotorSpeed){
                    m_leftMotorSpeed = leftMotorSpeed;
                    m_rightMotorSpeed = rightMotorSpeed;
                }

    public void startLauncher(){
        m_leftBallThrow.set(m_leftMotorSpeed);
        m_rightBallThrow.set(m_rightMotorSpeed);
    }

    public void stopLauncher(){
        m_leftBallThrow.set(0);
        m_rightBallThrow.set(0);
    }
}