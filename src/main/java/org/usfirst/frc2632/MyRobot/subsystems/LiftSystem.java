/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc2632.MyRobot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc2632.MyRobot.Robot;
import org.usfirst.frc2632.MyRobot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */

public class LiftSystem extends Subsystem {

  WPI_TalonSRX frontLiftMotor;
  WPI_TalonSRX midLiftMotor;
  WPI_TalonSRX rearLiftMotor;
  
  public LiftSystem(){

    frontLiftMotor = new WPI_TalonSRX(RobotMap.FRONT_LIFT_MOTOR);
    midLiftMotor = new WPI_TalonSRX(RobotMap.MID_LIFT_MOTOR);
    rearLiftMotor = new WPI_TalonSRX(RobotMap.BACK_LIFT_MOTOR);


    frontLiftMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    midLiftMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    rearLiftMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

    frontLiftMotor.config_kF(1, .4);
    frontLiftMotor.config_kD(1, 1);
    frontLiftMotor.config_kP(1, 10);

    midLiftMotor.config_kF(1, .2);
    midLiftMotor.config_kD(1, 1);
    midLiftMotor.config_kP(1, 5);

    rearLiftMotor.config_kF(1, -.2);
    rearLiftMotor.config_kD(1, 1);
    rearLiftMotor.config_kP(1, -5);
    


    WPI_TalonSRX wheelMotor = new WPI_TalonSRX(RobotMap.LIFT_WHEEL_MOTOR);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setFront(double setpoint){
    frontLiftMotor.set(ControlMode.Position, setpoint);
  }

  public void setMid(double setpoint){
    midLiftMotor.set(ControlMode.Position, setpoint);
  }

  public void setRear(double setpoint) {
    rearLiftMotor.set(ControlMode.Position, setpoint);
  }

  public boolean getHeight(double safeValue){
     boolean safePos = frontLiftMotor.get() <= safeValue && midLiftMotor.get() <= safeValue && rearLiftMotor.get() <= safeValue;
    return safePos;
  }




}