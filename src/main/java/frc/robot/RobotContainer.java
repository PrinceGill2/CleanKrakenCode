package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    //
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    //Programmer note: some of the shit I wrote for this could be wrong, dont fault me. Most of its right though that is confirmed.
                    // I try to update what 
                     //I remember to *shoulder shrug*
    public RobotContainer() {
        s_Swerve.setDefaultCommand( //extends subSystem class, commandScheduler in the init runs this default command by the executer (look in commands section)
            new TeleopSwerve( //teleopSwerve object
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), //this part works, wheels spin forwards and backwards 
                () -> -driver.getRawAxis(strafeAxis), //the wheels will turn 90 degrees to strafe
                () -> -driver.getRawAxis(rotationAxis), //the wheels will turn either 45 degrees or if strafe/translation is active a combined vector
                () -> robotCentric.getAsBoolean() //this is set by the driver, do they want forward dependent on the robot heading or the 
                                                 //pre set forward direction like postive x or something
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }

    public Swerve getSwerveObject() { 
        return s_Swerve;
    }
}
