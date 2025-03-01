package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;

public class SearchForAprilTag3R extends Command {
    boolean foundTag;
    SearchForAprilTag3R(Swerve s_Swerve) { 


            addRequirements(s_Swerve);
    }

    @Override
    public void initialize() { 

    }

    @Override
    public void execute() { 
        if (!foundTag) { 
            

        }

    }

    @Override
    public boolean isFinished() {
        return foundTag;
    }

    @Override
    public void end(boolean interrupted) { 

    }
}
