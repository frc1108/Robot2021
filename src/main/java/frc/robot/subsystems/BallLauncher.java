
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.BallLauncherConstants.CAN_ID_BALL_LAUNCH_LEFT;
import static frc.robot.Constants.BallLauncherConstants.CAN_ID_BALL_LAUNCH_RIGHT;

public class BallLauncher extends SubsystemBase implements Loggable {

    @Log.SpeedController(name = "Launcher L Motor")
    WPI_TalonSRX _leftBallThrow = new WPI_TalonSRX(CAN_ID_BALL_LAUNCH_LEFT);

    @Log.SpeedController(name = "Launcher R Motor")
    WPI_TalonSRX _rightBallThrow = new WPI_TalonSRX(CAN_ID_BALL_LAUNCH_RIGHT);

    public void throwBall(double left_spd, double right_spd){
        _leftBallThrow.set(left_spd);
        _rightBallThrow.set(-right_spd);
    }
}