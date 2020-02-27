
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import static frc.robot.Constants.BallLauncherConstants.CAN_ID_BALL_LAUNCH_LEFT;
import static frc.robot.Constants.BallLauncherConstants.CAN_ID_BALL_LAUNCH_RIGHT;

public class BallLauncher extends SubsystemBase implements Loggable {

    private WPI_TalonSRX m_leftBallThrow = new WPI_TalonSRX(CAN_ID_BALL_LAUNCH_LEFT);

    // Talon Tach
    private WPI_TalonSRX m_rightBallThrow = new WPI_TalonSRX(CAN_ID_BALL_LAUNCH_RIGHT);

    private double m_launcherSpeed = -0.8;
    private double m_launcherRPM = -3000;

    public BallLauncher(){
        m_leftBallThrow.configFactoryDefault();
        m_leftBallThrow.setNeutralMode(NeutralMode.Coast);
        m_leftBallThrow.follow(m_rightBallThrow);
        m_leftBallThrow.setInverted(true);

        m_rightBallThrow.set(0);
        m_rightBallThrow.configFactoryDefault();
        m_rightBallThrow.setNeutralMode(NeutralMode.Coast);
        m_rightBallThrow.configContinuousCurrentLimit(30);
        m_rightBallThrow.setInverted(false);

        m_rightBallThrow.config_kF(0, 0.1225);
        m_rightBallThrow.config_kP(0,1);
        m_rightBallThrow.config_kI(0,0);
        m_rightBallThrow.config_kD(0,0);

        // Config Talon Tach
        final int kTimeoutMs = 30;
        m_rightBallThrow.configSelectedFeedbackSensor(FeedbackDevice.Tachometer, 0, kTimeoutMs);
        int edgesPerCycle = 1;
        m_rightBallThrow.configPulseWidthPeriod_EdgesPerRot(edgesPerCycle, kTimeoutMs);
        int filterWindowSize = 1;
        m_rightBallThrow.configPulseWidthPeriod_FilterWindowSz(filterWindowSize, kTimeoutMs);
    }

    public void setLauncherSpeed(double rightMotorSpeed){
        m_launcherSpeed = rightMotorSpeed;
    }

    public void startLauncher(){
        m_rightBallThrow.set(m_launcherSpeed);
    }

    public void stopLauncher(){
        m_rightBallThrow.set(0);
    }

    @Log.Dial(name = "Launcher RPM", tabName = "Match View")
    public double getTachRPM(){
        double tachVel_UnitsPer100ms = m_rightBallThrow.getSelectedSensorVelocity(0);
        return -1*tachVel_UnitsPer100ms*600/1024;
    }

    public void setLauncherRPM (double launcherRPM){  
        m_launcherRPM = -launcherRPM;
    }

    public void startPIDLauncher(){      
        m_rightBallThrow.set(ControlMode.Velocity,m_launcherRPM*1024/600);
    }
}