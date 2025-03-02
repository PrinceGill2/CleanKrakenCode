package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//I'll have to see if this defualt command has to be canceled 
//When autos in teleop are ran. Since the s_Swerve subsystem will have to be
//used 

//this just passes the swerve values with the deadband applied ott he swerve object
public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSupX;
    private BooleanSupplier robotCentricSup;
    private DoubleSupplier rotationSupY;

    private ProfiledPIDController headingController;
                                                        //forward and back          side to side                 rotates the robot by setting wheel angle
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSupX, DoubleSupplier rotationSupY, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve; //a weird circular depency going on here with the object using this default command 
                                  // also being in the default.... code runs though
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSupX = rotationSupX;
        this.rotationSupY = rotationSupY;
        this.robotCentricSup = robotCentricSup;

        //Change the HeadingController trapezoid constraints
        headingController = new ProfiledPIDController(Constants.Swerve.headingKP, Constants.Swerve.headingKI, Constants.Swerve.headingKD, new TrapezoidProfile.Constraints(2, 1));

    }

    @Override
    public void initialize() { 
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }
    //This command is intended to be used with joySticks and will do angle calculations for you
    public double robotHeading() { 
        double rotationValX = MathUtil.applyDeadband(rotationSupX.getAsDouble(), Constants.stickDeadband);
        double rotationValY = MathUtil.applyDeadband(rotationSupY.getAsDouble(), Constants.stickDeadband);

        //magnitude is intended to speeds up the turning speed
        double magnitudeOfRotation = Math.pow(rotationValX, 2) + Math.pow(rotationValY, 2);
        double angleDegrees = Math.atan(rotationValY/rotationValX);

        double rotationVal = headingController.calculate(s_Swerve.getHeading().getDegrees(), angleDegrees);

        return rotationVal;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = robotHeading();
       //makes sure no drift
        /* Drive */
        s_Swerve.drive( //calls drive function of swerve object
            //Those values between 0 and 1.0 are passed as changees to x and y 
            //Those values are then multiplied to ensure that the robot travels fast enough
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),  //this is the first time the values are being significantly 
                                                                                            //converted
            rotationVal * Constants.Swerve.maxAngularVelocity, //the rotation value is still about the same
            !robotCentricSup.getAsBoolean(), //if its field or driver centric
            true //This means the wheels operate without knowing where it is. So no feedforward. Good for teleop not good for auto
        );

    }
}