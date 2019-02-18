/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc2632.MyRobot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc2632.MyRobot.RobotMap;
import org.usfirst.frc2632.MyRobot.commands.LiftInit;
import org.usfirst.frc2632.MyRobot.commands.PIDLiftOverride;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */

public class LiftSystem extends Subsystem {

  WPI_TalonSRX frontLiftMotor;
  WPI_TalonSRX midLiftMotor;
  WPI_TalonSRX rearLiftMotor;
  WPI_TalonSRX wheelMotor;
  DigitalInput configLimitSwitch;
  
  
  public LiftSystem(){

    frontLiftMotor = new WPI_TalonSRX(RobotMap.FRONT_LIFT_MOTOR);
    midLiftMotor = new WPI_TalonSRX(RobotMap.MID_LIFT_MOTOR);
    rearLiftMotor = new WPI_TalonSRX(RobotMap.BACK_LIFT_MOTOR);

    wheelMotor = new WPI_TalonSRX(RobotMap.LIFT_WHEEL_MOTOR);
    configLimitSwitch = new DigitalInput(RobotMap.LIFT_CONFIG_LIMITSWITCH);
    /*
    frontLiftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    midLiftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rearLiftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    

    frontLiftMotor.config_kF(1, RobotMap.kfFrontLift);
    frontLiftMotor.config_kD(1, RobotMap.kdFrontLift);
    frontLiftMotor.config_kP(1, RobotMap.kpFrontLift);

    midLiftMotor.config_kF(1, RobotMap.kfMidLift);
    midLiftMotor.config_kD(1, RobotMap.kdMidLift);
    midLiftMotor.config_kP(1, RobotMap.kpMidLift);

    rearLiftMotor.config_kF(1, RobotMap.kfRearLift);
    rearLiftMotor.config_kD(1, RobotMap.kdRearLift);
    rearLiftMotor.config_kP(1, RobotMap.kpRearLift);
    */

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new PIDLiftOverride());
  }
/*
  public void setFront(double setpoint){
    frontLiftMotor.set(ControlMode.Position, setpoint);
  }

  public void setMid(double setpoint){
    midLiftMotor.set(ControlMode.Position, setpoint);
  }

  public void setRear(double setpoint) {
    rearLiftMotor.set(ControlMode.Position, setpoint);
  }
  */
  public double getFront() {
    return frontLiftMotor.get();
  }

  public double getMid(){
    return midLiftMotor.get();
  }

  public double getRear(){
    return rearLiftMotor.get();
  }

  public boolean getHeight(double safeValue){
     boolean safePos = frontLiftMotor.get() <= safeValue && midLiftMotor.get() <= safeValue && rearLiftMotor.get() <= safeValue;
    return safePos;
  }
  
  public void liftElevator(double value){
    midLiftMotor.set(ControlMode.Position, value/2);
    rearLiftMotor.set(ControlMode.Position, -value/2);
  }
  public void liftElevatorInches(double inches){
    double pulses = inches * 21.41;
    midLiftMotor.set(ControlMode.Position, pulses / 2);
    rearLiftMotor.set(ControlMode.Position, -pulses / 2);
  }

  public void liftElevatorManual(Joystick joystick){
    midLiftMotor.set(ControlMode.PercentOutput, joystick.getY());
    rearLiftMotor.set(ControlMode.PercentOutput, -joystick.getY());
  }

  public double getElevatorHeight(){
    return midLiftMotor.get();
  }

  public void driveLiftSystem(XboxController controller){
    wheelMotor.set(controller.getX(Hand.kRight));
  }

  public boolean getLimitSwitch(){
    return configLimitSwitch.get();
  }




}
