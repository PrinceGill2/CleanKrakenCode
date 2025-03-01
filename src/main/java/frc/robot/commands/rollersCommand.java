package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class rollersCommand extends Command {
    Arm arm;
    int direction;

    public rollersCommand(Arm arm, int direction) { 
        this.arm = arm;
        this.direction = direction;
        addRequirements(arm);
    }

    @Override
    public void initialize() { 

    }

    @Override
    public void execute() { 
        arm.rollers(direction);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { 

    }

}
