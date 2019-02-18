// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2632.MyRobot;

import org.usfirst.frc2632.MyRobot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;



/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static XboxController controller;
    public static Joystick secondaryController;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        controller = new XboxController(0);
        secondaryController = new Joystick(1);
        //all the buttons are initialized so commands can be associated with them 
        Button aButton = new JoystickButton(controller, 1);
        Button bButton = new JoystickButton(controller, 2);
        Button xButton = new JoystickButton(controller, 3);
        Button yButton = new JoystickButton(controller, 4);


        xButton.whenPressed(new ZachHatchGrab());
        bButton.whileHeld(new SpeedRacer());
        aButton.whileHeld(new SlowDrive());


        Button button1 = new JoystickButton(secondaryController, 1);
        Button button2 = new JoystickButton(secondaryController, 2);
        Button button3 = new JoystickButton(secondaryController, 3);
        Button button4 = new JoystickButton(secondaryController, 4);
        Button button5 = new JoystickButton(secondaryController, 5);
        Button button6 = new JoystickButton(secondaryController, 6);
        Button button7 = new JoystickButton(secondaryController, 7);
        Button button8 = new JoystickButton(secondaryController, 8);
        Button button9 = new JoystickButton(secondaryController, 9);

        button6.whenPressed(new MoveHatchCatcher());
        button7.whenPressed(new ZachHatchRelease());
        button2.whileHeld(new BallDrop());
        //button4.whileHeld(new LiftElevator(24));
        //button3.whenPressed(new LiftElevator(56));
        //button5.whenPressed(new LiftElevator(83));
        //button9.whileHeld(new PIDLiftOverride());

        // SmartDashboard Buttons

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public XboxController getController() {
        return controller;
    }

    public Joystick getJoystick(){
        return secondaryController;
    }



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

