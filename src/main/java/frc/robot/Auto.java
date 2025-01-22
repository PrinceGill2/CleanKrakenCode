package frc.robot;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.subsystems.Swerve;

/*AUTO CONSTRUCTOR */
public class Auto {
    Swerve s_Swerve;
    AutoBuilder autoBoy;
    RobotConfig config;
    private final SendableChooser<Command> autoChooser;

    public Auto(Swerve s_Swerve) { 
        this.s_Swerve = s_Swerve;

        
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(  
      s_Swerve.poseForAuto,
      s_Swerve.setPoseForAuto,
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

            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);
    }



    public Command getAutonomousCommand() {
      // An ExampleCommand will run in autonomous
      return autoChooser.getSelected();
    }
}



