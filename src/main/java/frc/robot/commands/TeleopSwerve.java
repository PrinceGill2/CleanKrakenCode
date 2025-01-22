package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

//this just passes the swerve values with the deadband applied ott he swerve object
public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
                                                        //forward and back          side to side                 rotates the robot by setting wheel angle
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve; //a weird circular depency going on here with the object using this default command 
                                  // also being in the default.... code runs though
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
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