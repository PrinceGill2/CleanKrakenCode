package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

//This would take a Pose2d from the swerveOdometry object thats been defined in the 
//swerve class and take a Pose2d from the camera (Lime light helperrs seems to be capable 
// of returning a Pose2d object) and using a trapezoidal motion profile to get the robot to that 
// position

public class MoveToAprilTag extends Command{
    Pose2d robotPose; 
    Pose2d targetPose;
    

    public MoveToAprilTag() { 

    }
    

    @Override
    public void execute() { 

    }

}
