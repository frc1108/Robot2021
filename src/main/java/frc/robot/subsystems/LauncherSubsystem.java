
package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import static frc.robot.Constants.BallLauncherConstants.*;

import java.util.function.Supplier;

public class LauncherSubsystem extends SubsystemBase implements Loggable {
    @Log.SpeedController
    private WPI_TalonSRX m_rightMain = new WPI_TalonSRX(CAN_ID_BALL_LAUNCH_RIGHT);
    private WPI_TalonSRX m_leftFollow = new WPI_TalonSRX(CAN_ID_BALL_LAUNCH_LEFT);

    Supplier<Double> encoderPosition;
    Supplier<Double> encoderRate;

      // Volts per (radian per second)
  private static final double kFlywheelKv = (0.128/(2*Math.PI));

  // Volts per (radian per second squared)
  private static final double kFlywheelKa = (0.005/(2*Math.PI));

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  //
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.identifyVelocitySystem(
        kFlywheelKv, kFlywheelKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        m_flywheelPlant,
        VecBuilder.fill(3.0), // How accurate we think our model is
        VecBuilder.fill(0.01), // How accurate we think our encoder data is
        0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller
        = new LinearQuadraticRegulator<>(m_flywheelPlant,
        VecBuilder.fill(8.0),  // Velocity error tolerance
        VecBuilder.fill(12.0), // Control effort (voltage) tolerance
        0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
      m_flywheelPlant,
        m_controller,
        m_observer,
        12.0,
        0.020);
    
    private static final int ENCODER_EDGES_PER_REV = 1;
    private static final int PIDIDX = 0;
    private static final int kTimeoutMs = 30;
    private static final int filterWindowSize = 1;

    private static double kSpinupRadPerSec;
    
    public LauncherSubsystem(){
        stop();
        m_rightMain.configFactoryDefault();
        m_leftFollow.configFactoryDefault();

        m_rightMain.setNeutralMode(NeutralMode.Coast);
        m_leftFollow.setNeutralMode(NeutralMode.Coast);

        m_rightMain.setInverted(true);
        m_leftFollow.setInverted(false);
        
        m_rightMain.setSensorPhase(false);
        
        m_leftFollow.follow(m_rightMain);

        m_rightMain.configContinuousCurrentLimit(30);
        
        // m_rightMain.config_kF(0,0); //0,0.1225
        // m_rightMain.config_kP(0,0); //0,1
        // m_rightMain.config_kI(0,0);
        // m_rightMain.config_kD(0,0);

        double encoderConstant = (1 / ENCODER_EDGES_PER_REV) * 1;

        m_rightMain.configSelectedFeedbackSensor(FeedbackDevice.Tachometer, PIDIDX, kTimeoutMs);
        m_rightMain.configPulseWidthPeriod_EdgesPerRot(ENCODER_EDGES_PER_REV, kTimeoutMs);
        m_rightMain.configPulseWidthPeriod_FilterWindowSz(filterWindowSize, kTimeoutMs);

        encoderPosition = ()
            -> m_rightMain.getSelectedSensorPosition(PIDIDX) * encoderConstant/1024;
        encoderRate = ()
            -> (m_rightMain.getSelectedSensorVelocity(PIDIDX) * encoderConstant * 10/1024);

        m_loop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(60*encoderRate.get())));
        
        // Default command to stop() 
        this.setDefaultCommand(new RunCommand(() -> stop(), this));
    }

    public void stop(){
        m_loop.setNextR(VecBuilder.fill(0.0));
        m_rightMain.stopMotor();
    }
   
    @Log.Dial(name = "Tachomter RPM",
              tabName = "Live",
              min = 0,
              max = 4000)
    public double getTachRPM(){
        return m_rightMain.getSelectedSensorVelocity(0)*600/1024;
    }

    // public void startPIDLauncher(){      
    //     m_rightMain.set(ControlMode.Velocity,m_launcherRPM*1024/600);
    // }

    public void start(){
        m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));
    }

    @Override
    public void periodic(){
    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(60*encoderRate.get())));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    m_rightMain.setVoltage(nextVoltage);
    }

    @Config.NumberSlider(name="SpinupRPM",
                         tabName = "Live",
                         defaultValue = 3000,
                         min = 0,
                         max=4000,
                         blockIncrement = 250)
    public void setSpinupRotPerMin(double spinupRotPerMin){
        kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(spinupRotPerMin);
    }

    @Log
    public double getEncoderRate(){
        return 60*encoderRate.get();
    }
}