package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.RobotContainer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

//DEPRECATED: KEPT FOR LATER ANAL-YSIS


//This would take a Pose2d from the swerveOdometry object thats been defined in the 
//swerve class and take a Pose2d from the camera (Lime light helperrs seems to be capable 
// of returning a Pose2d object) and using a trapezoidal motion profile to get the robot to that 
// position

public class MoveToAprilTag extends Command{
    Pose2d robotPose; //This might be unecessary as the path is made via AutoBuilder 
                      //which finds the robot pose on its own
    Pose2d targetPose;
    //A global instance of the helper class, this will be useful 
    //Might replace with the vison subsystem leter
    LimelightHelpers.LimelightTarget_Fiducial helperClass = LimelightHelpers.getGlobalLimelightHelper();
    Swerve s_Swerve;

    Command pathOnTheFly;
    double endVelocity;
    
    //That path we're passing into this thing will be a reference to a path in the auto that this command will be made in
    //That auto is what will be scheduled and in that auto this path in put into the group.
    public MoveToAprilTag(RobotContainer robotContainer, Command pathOnTheFly, double endVelocity) { 
        s_Swerve = robotContainer.getSwerveObject();
        //fetches the robotpose and targetpose(april tag pose ) 
        robotPose = s_Swerve.getPose();
        targetPose = helperClass.getTargetPose_RobotSpace2D();
        //There is just a static method that returns the targetPose

        this.pathOnTheFly = pathOnTheFly;
        this.endVelocity = endVelocity;
        //reserves the Swerve subsystem 
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
      
    }
    //condititon for the command to end
    @Override
    public boolean isFinished() {
      return true;
    }
  
    

    @Override
    public void execute() { //These values are temporary and should not be used as absolute 
                            //limitations on the robot. Should be found from the constants folder.
        PathConstraints constraints = new PathConstraints(3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
        //I dont know if this should just initialize the reference or have it schedule the reference itself 
        //I think it would be better for it to initialize it and for the auto class to schedule the actual path
        //I need to figure out some way to reliably get the pathOnTheFly to the basicPlannerPath class
        //as this would not work
        pathOnTheFly = AutoBuilder.pathfindToPose(
            targetPose, 
            constraints,
            endVelocity);
    }
    //when isFinished runs true the end method will be called by the scheduler 
    @Override
    public void end(boolean interrupted) {
        
    }


}
