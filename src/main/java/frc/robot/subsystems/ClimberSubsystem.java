/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import static frc.robot.Constants.ClimberConstants.*;
/**
 * Add your docs here.
 */
public class ClimberSubsystem extends SubsystemBase implements Loggable {
  private final WPI_TalonSRX m_winch = new WPI_TalonSRX(CAN_ID_WINCH);
  private double m_maxSpeed = 0;
  
  public ClimberSubsystem() {
    stop();
    m_winch.configFactoryDefault();
    m_winch.setNeutralMode(NeutralMode.Brake);
    m_winch.configContinuousCurrentLimit(40);
    m_winch.configPeakCurrentLimit(60);
    m_winch.setInverted(false);

    // Default command to stop() 
    this.setDefaultCommand(new RunCommand(() -> stop(), this));
  }

  public void setSpeedMax(){
    m_maxSpeed = 0.75;
  }

  public void manualControl(double speed){
          m_winch.set(speed*m_maxSpeed);
  }

  public void stop(){
    m_winch.stopMotor();
  }
}
