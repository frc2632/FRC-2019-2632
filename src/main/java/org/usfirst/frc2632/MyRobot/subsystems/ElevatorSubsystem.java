/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc2632.MyRobot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc2632.MyRobot.RobotMap;
import org.usfirst.frc2632.MyRobot.commands.elevatorCommands.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends Subsystem {
  TalonSRX frontMotor;
  TalonSRX midMotor;
  TalonSRX rearMotor;
  TalonSRX liftWheelMotor;
  DigitalInput configLimitSwitch;

  
  public double ticksPerInch = 21.41;

  public ElevatorSubsystem(){
    frontMotor = new TalonSRX(RobotMap.FRONT_LIFT_MOTOR);
    midMotor = new TalonSRX(RobotMap.MID_LIFT_MOTOR);
    rearMotor = new TalonSRX(RobotMap.BACK_LIFT_MOTOR);
    liftWheelMotor = new TalonSRX(RobotMap.LIFT_WHEEL_MOTOR);
    configLimitSwitch = new DigitalInput(RobotMap.LIFT_CONFIG_LIMITSWITCH);


    //set all of the motor controller sensors to Quad Encoders
    frontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    midMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rearMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    frontMotor.config_kP(1, .5);

    midMotor.config_kP(1, .5);

    rearMotor.config_kP(1, .5);

  }

  //-----------------------------------------------------CONFIG---------------------------------------------------------------------
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorDriveCommand());
  }

  public void zeroHeight(){
    midMotor.getSensorCollection().setQuadraturePosition(0, 0);  
    rearMotor.getSensorCollection().setQuadraturePosition(0, 0);
  }

  //--------------------------------------------------------LIFT ELEVATOR-------------------------------------------------------------
  public void elevatorLift(XboxController controller){
    midMotor.set(ControlMode.PercentOutput, controller.getY(Hand.kRight)*.75);
    rearMotor.set(ControlMode.PercentOutput, controller.getY(Hand.kRight));
  }
  public void elevatorLift(double value){
    midMotor.set(ControlMode.PercentOutput, value*.75);
    rearMotor.set(ControlMode.PercentOutput, value);
  }

  public void elevatorLiftInches(double inches){
    double ticks = inches * ticksPerInch;
    midMotor.set(ControlMode.Position, ticks/2);
    rearMotor.set(ControlMode.Position, -ticks/2);
  }

  public void elevatorLiftTicks(double ticks){
    midMotor.set(ControlMode.Position, ticks/2);
    rearMotor.set(ControlMode.Position, -ticks/2);
  }
  public void incrementElevator(){
    midMotor.set(ControlMode.Position, midMotor.getSelectedSensorPosition() + 21.41);
    rearMotor.set(ControlMode.Position, midMotor.getSelectedSensorPosition() - 21.41);
  }


  //----------------------------------------------------------CLIMB------------------------------------------------------------------

  public void climb(XboxController controller){
    frontMotor.set(ControlMode.PercentOutput, controller.getY(Hand.kRight)*.9);
    midMotor.set(ControlMode.PercentOutput, -controller.getY(Hand.kRight));
    rearMotor.set(ControlMode.PercentOutput, controller.getY(Hand.kRight));
  }
  public void climbFront(double value){
    frontMotor.set(ControlMode.PercentOutput, value);
  }
  public void climbRear(double value){
    midMotor.set(ControlMode.PercentOutput, -value);
    rearMotor.set(ControlMode.PercentOutput, value);
  }

  public void climbInches(double inches){
    double ticks = inches * ticksPerInch;
    frontMotor.set(ControlMode.Position, -ticks);
    midMotor.set(ControlMode.Position, ticks/2);
    rearMotor.set(ControlMode.Position, -ticks/2);
  }


  public void climbTop(){
    climbInches(19);
  }

  public void climbBottom(){
    climbInches(13);
  }


  public boolean getConfigLimitSwitch() {
    return configLimitSwitch.get();
  }




}
