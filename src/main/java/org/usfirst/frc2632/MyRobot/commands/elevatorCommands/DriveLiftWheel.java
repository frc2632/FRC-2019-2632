/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc2632.MyRobot.commands.elevatorCommands;

import org.usfirst.frc2632.MyRobot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveLiftWheel extends Command {
  public DriveLiftWheel() {
    requires(Robot.blueDriveWheels);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.blueDriveWheels.driveBlueWheels(.75);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.blueDriveWheels.driveBlueWheels(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.blueDriveWheels.driveBlueWheels(0);
  }
}