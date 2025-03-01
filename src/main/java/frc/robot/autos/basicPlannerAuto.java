package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
//might be pointless
import frc.robot.commands.MoveToAprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto;


public class basicPlannerAuto extends SequentialCommandGroup{
    //This implements on of my first autos 

    //it'll take a pre planned path then use the line up method is get the robot to 
    //the april tag 


    
    public basicPlannerAuto(RobotContainer robotContainer) { 
        
        

        addCommands(
            
        );
    }
}
