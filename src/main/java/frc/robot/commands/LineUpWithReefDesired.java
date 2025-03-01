package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class LineUpWithReefDesired extends Command {
    Pose2d initialRobotPose; 
    Pose2d finalRobotPose;
    Pose2d initialTargetPose; 
    Trajectory path;
    ProfiledPIDController thetaController;
    Rotation2d desiredAngle;
    Swerve s_Swerve;
    HolonomicDriveController controller;
    double time;

    static LimelightHelpers.LimelightTarget_Fiducial helperClass = LimelightHelpers.getGlobalLimelightHelper();

    //Creates the Initial instances of location that are required 
    public LineUpWithReefDesired(frc.robot.subsystems.Swerve s_Swerve) { 

        this.s_Swerve = s_Swerve;

        //Create the theta controller 
        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //Create the controller still have to set the kP values 
        HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), //I have yet to set the KPX or KPY values
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                            thetaController);

       addRequirements(s_Swerve);
    }
    /*new ProfiledPIDController(1, 0, 0,
    new TrapezoidProfile.Constraints(6.28, 3.14)) */ //original theta controller
    @Override
    public void initialize() { 
        //milliseconds since 1970 jan1st
        //Difference between the current time and this value will be used to 
        //get the chassisSpeeds from the trajecotry class
        time = System.currentTimeMillis();
        //The yaw would be the rotation angle that I'm going to be 
        //using
        //Looks like the limelight image of the OS will change what values the 
        //pose2d returns since the april tags change positions
        //The angle that will be returned is from the cross hair(center) of the limelight then 
        //positive and negative the directions with typical cartesian systems.
        initialTargetPose = helperClass.getTargetPose_RobotSpace().toPose2d(); //I get how this can return the x,y coordinates but 
        //what about the angle? What is 0 for this angle? 
        //without info like that I'm going to have a hard time 
        //calculating how to move the robot here 
        initialRobotPose = s_Swerve.getPose();
        //This is the theoretical pose that the robot must get to
        //I dont know how to calculate that value yet
        Pose2d somePose = new Pose2d();

        //calculate the actual pose needed for the targetpose from the relative pose given by the 
        //helper class. I'll need to use actual limelights to figure out how the values that it returns 
        //actually work
        //For now it's just going to be some Pose2d to represent the target pose that the robot will have
        //I need to figure out how to transform the TargetPose relative to the robotSpace to actual coordinates the 
        //robot can go to
        //=> 
        double xCoord = ((initialRobotPose.getX() + somePose.getX())/2);
        double yCoord = ((initialRobotPose.getY() + somePose.getY())/2);
        Translation2d interiorPoint = new Translation2d(xCoord, yCoord);
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(interiorPoint);

        //The trajectory configuration is a filled with temporary values
        TrajectoryConfig config = new TrajectoryConfig(5.0, 2.0);
    config.setReversed(true);

        //This is the angle the robot will face for its duration of lining up with the reef
        Rotation2d desiredAngle = initialRobotPose.getRotation();

        //Create the trajectory using cubic splines for simplicity. 
        /*
        generateTrajectory(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config) */
        Trajectory path = TrajectoryGenerator.generateTrajectory(initialRobotPose, interiorWaypoints, somePose, config);

    }

    @Override
    public void execute() { 
        double timeSinceStart = (System.currentTimeMillis() - time)/1000;
        //Sample the speeds at the current time stamp
Trajectory.State goal = path.sample(timeSinceStart);
// This will get the speeds necessary, calculated via the PID from 
//the controller object initialized in the constructor
ChassisSpeeds adjustedSpeeds = controller.calculate(
 s_Swerve.getPose(), goal, desiredAngle);

        //Can implement a quick test to see if the location
        //has been reached. Like if the limelight see's
        //that we're pretty close to the april tag

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { 

    }
}
