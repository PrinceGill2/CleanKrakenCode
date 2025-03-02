package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.Supplier;

import javax.sound.midi.Track;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0); //the pigeons values arent used for anything right now
    
        
        //this object is used at the end of the drive method 
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants), //this object is defined in another file
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants), 
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        }; //these are "permanent" as in created in the initialization of both teleopoerated/autonomous 

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        //this is used to track the position of the robot, its used in autonomous (if desired)
        //I can pull data out of this Odometry object and use Pose2d's or Pose3d's 
        //found from vision to then move the robot through that motion
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) { //this is where some magic happens
        //the translation 2d object made previously along with rotation is used, I think the issue is the values being passed to the wheels arent right
        SwerveModuleState[] swerveModuleStates = //from the looks of it, this is a ternary operator that passes an rValue (reference or not is unknown) 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates( //to the swerveKinematics object bia the toSwerveModuleStates class
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds( //creates a object that represents the vector of the robot (magnitiude and direction)
                                    translation.getX(),   //Seems like what would serve as a change in X and Y              //this is a static function and it's unknown where these values are stored 
                                    translation.getY(),   //Are actually just speed values. dx/dt, dy/dt essentially
                                    rotation, //dtheta/dt, the change is radians per unit of time
                                    getHeading()// field relative
                                )
                                : new ChassisSpeeds( //not field relative
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed); //normalizes wheel speeds, wheels arent spinning yet
                                                                                                    //this calculation is done before the value is pushed to
                                                                                                    //motors
        //These modules are spinning in the opposite direction usually
        //I'm reversing the direction of the speed they spin in the correct direction
        //For some reason it is only on the right side
        swerveModuleStates[1].speedMetersPerSecond *= -1;
        swerveModuleStates[3].speedMetersPerSecond *= -1;

                                //here is the object being used, mod temporarily represents each swerve module
        for(SwerveModule mod : mSwerveMods){ //a python style for loop that just iterates through 1 by 1
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop); //it passes a temporary object swerveModuleStates which are replaced upon the next input read
        }                                           //module number is defined when the module object is created
    }  
    
    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }
   //returns an array
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        //return gyro.getRotation2d();
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()); 
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    //This returns the pose in meters, I think pathplanner does everything in meters? 
    //that would make sense. What sort of maniac would make it some arbitrary unit that has no
    //relation to a realworld metric system in a competition for highschoolers.
    public Supplier<Pose2d> poseForAuto = () -> { 
     return getPose();
    };

    public Consumer<Pose2d> setPoseForAuto = (Pose2d trajectoryPose) -> { 
        setPose(trajectoryPose);

    };

    public Consumer<ChassisSpeeds> autoDrive = (ChassisSpeeds autoBuilderChassisSpeeds) -> { 
        int otherFella = 1;
        otherFella++;
        //SmartDashboard.putNumber("AutoDrive?" , otherFella);
        SwerveModuleState[] swerveModuleStates = 
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(autoBuilderChassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
                for(SwerveModule mod : mSwerveMods){ //Its passing auto values now and is closed loop to allow use of feedforward
                                                     //we can do feedforward through autobuilder as well I just choose not to.
                    mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
                }

    };

    public Supplier<ChassisSpeeds> robotRelativeChassisSpeeds = () -> {
        int otherFella = 1;
        otherFella++;
        //SmartDashboard.putNumber("ChassisSpeedsGetter??" , otherFella);
        ChassisSpeeds fieldRelativeSpChassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpChassisSpeeds,
                                                    getHeading());
    };


    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        //If need be I can upload the gyro yaw here
    }
}