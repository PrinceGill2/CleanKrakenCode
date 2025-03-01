package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.aprilTagConstants;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LineUpWithReef extends Command {

    Swerve s_Swerve;
    Pose2d initialRobotPose; 
    Pose2d finalRobotPose;
    Pose3d initialTargetPose; 
    Rotation2d desiredAngle;
    LimelightHelpers.PoseEstimate EstimatedPose;

    double omegaRadiansPerSecond;
    double vectorVelocities;
    double vectorXVelocities;
    double vectorYVelocities;
    double stopX = 1;
    double stopY = 1;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    double tx;

    static LimelightHelpers.LimelightTarget_Fiducial helperClass = LimelightHelpers.getGlobalLimelightHelper();

    public LineUpWithReef(frc.robot.subsystems.Swerve s_Swerve) { 

        this.s_Swerve = s_Swerve;

    }
    @Override
    public void initialize() { 
        initialTargetPose = helperClass.getTargetPose_RobotSpace(); 
      
        initialRobotPose = s_Swerve.getPose();

        tx = table.getEntry("tx").getDouble(0.0);
    }
    //as long as the bot is within range of the apriltag like within 10 meters
    //I can just divide the distance by 10 that way the closer I get to the goal the slower it goes
    @Override
    public void execute() { 
        //Dunno if this angle is negative or not
        omegaRadiansPerSecond = initialTargetPose.getRotation().getAngle() * aprilTagConstants.KpTheta;
        //megaRadiansPerSecond = tx * aprilTagConstants.KpTheta * -1;
        //The commented version might be better

        //These values might be negative initially
        if(initialTargetPose.getX() < .10){
            stopX = 0;
        }
        else stopX = 1;
            
        vectorXVelocities = ((initialTargetPose.getX()/10) * aprilTagConstants.KpXVelocity + .05) * stopX ;

        if(initialTargetPose.getY() < .10){
            stopY = 0;
        }
        else stopY = 1;

        vectorYVelocities = ((initialTargetPose.getZ()/10) * aprilTagConstants.KpYVelocity + .05) * stopY ;

        ChassisSpeeds lineUp = new ChassisSpeeds(vectorXVelocities, vectorYVelocities, omegaRadiansPerSecond);

        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(lineUp);

        s_Swerve.setModuleStates(states);


        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { 

    }
}
