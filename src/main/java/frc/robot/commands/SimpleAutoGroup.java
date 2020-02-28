/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTimed;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleAutoGroup extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAutoGroup. This Auto commmand just drives forward for 3 secodns and stops.
   */
  public SimpleAutoGroup(DriveSubsystem drive) {
    super(new DriveTimed(drive).withTimeout(1));
  }
}
