package frc.robot;


import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.commands.MoveToAprilTag;
import frc.robot.subsystems.Swerve;

//My auto class 

//This will initialize all of the auto classes defined in the auto folder by providing them instances of itself 
//which they will then use to get the commands they need.
public class Auto {
    Swerve s_Swerve;
    AutoBuilder autoBoy;
    RobotConfig config;
    private final SendableChooser<Command> autoChooser;
    RobotContainer robotContainer;
    static LimelightHelpers.LimelightTarget_Fiducial helperClass = LimelightHelpers.getGlobalLimelightHelper();
    
        public Auto(Swerve s_Swerve, RobotContainer robotContainer) { 
            this.s_Swerve = s_Swerve;
            this.robotContainer = robotContainer;
    
            
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }
    
        // Configure AutoBuilder last
        AutoBuilder.configure(  
          s_Swerve::getPose,
          s_Swerve::setPose,
          s_Swerve.robotRelativeChassisSpeeds, 
          s_Swerve.autoDrive, //You can change this to biconsumer if you want to control feed forward from here
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                s_Swerve
    
        );
                //This is automatically populated with all of the auto in the 
                //pathPlanner
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                //For the end velocity, if need be, come up with a method that can fetch the end velocity if the 
                //command requires it. 
                NamedCommands.registerCommand("MoveToAprilTag", new MoveToAprilTag(robotContainer, getAutonomousCommand(), 0));
              }
    
    
    
        public Command getAutonomousCommand() {
          // An ExampleCommand will run in autonomous
          return autoChooser.getSelected();
        }


        //This will create a path to get to an april tag
        //I will create different settings on different april tags on a case by case basis.
        //This will put the robot always between a certain range of distance from the april tag
        //and give the robot LOS of the April Tag
        public static Command pathOnFly(int endVelocity) {
          //These values are temporary and should not be used as absolute 
                                //limitations on the robot. Should be found from the constants folder.
            PathConstraints constraints = new PathConstraints(3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
            //I dont know if this should just initialize the reference or have it schedule the reference itself 
            //I think it would be better for it to initialize it and for the auto class to schedule the actual path
            //I need to figure out some way to reliably get the pathOnTheFly to the basicPlannerPath class
            //as this would not work
            Command pathOnTheFly = AutoBuilder.pathfindToPose(
              //This pose from the limelights isnt the actual pose required 
              //that will have to be calculated
            LimelightHelpers.getTargetPose3d_RobotSpace(null).toPose2d(),
            constraints,
            endVelocity);
      return pathOnTheFly;
    }

    //Path on the fly with no pathplanner 

    //MAKE METHOD HERE

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    //This will create a sendable chooser populated with all of the autos 
    //I want. VOID TEMPORARILY. returns the sendable chooser to be used in
    //Robot
    public void initializeAllAutos() { 

    }

    //FIND END VELOCITY FOR ANY COMMAND/AUTO VVV
    //public double findVelocity() {}
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}



