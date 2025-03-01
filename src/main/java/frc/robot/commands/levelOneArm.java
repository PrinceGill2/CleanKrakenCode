package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

import java.util.regex.MatchResult;

import edu.wpi.first.math.MathUtil;

public class levelOneArm extends Command {

    ArmFeedforward feedforward;
    PIDController PID;


    Arm arm;

    public levelOneArm(Arm arm) { 
        

        addRequirements(arm);
    }

    @Override
    public void initialize() { 

    }

    @Override
    public void execute() { 
        //feedforward object says to calculate the angle setpoint with the 0 at horizontal
        //feedForward takes radians natively while the PID controllers can take whatever units you
        //feel like using
        double voltageArmHinge = arm.armHingePID.calculate(arm.getArmHingeRotat()[0], ArmConstants.levelOneRotat) + arm.armHingeFeedforward.calculate(arm.armHingePID.getSetpoint().position, 0);
        //values passed to this are still calculated via rotations. The rotations are just found by a distance to 
        //rotation conversion via gear math (yay!)
       double voltageExtension = arm.extensionPID.calculate(arm.getExtensionRotat()[0], ArmConstants.levelOneExtension) + arm.extensionFeedforward.calculate(arm.extensionPID.getSetpoint().velocity, 0);

        double voltageHinge = arm.handHingePID.calculate(arm.getHandHinge(), ArmConstants.levelOneHinge) + arm.handHingeFeedforward.calculate(arm.handHingePID.getSetpoint().position, 0);

        //should I clamp the PID and feedforward individually or
        //together
       voltageArmHinge = MathUtil.clamp(voltageArmHinge, -12, 12);
       voltageExtension = MathUtil.clamp(voltageExtension, -12, 12);
       voltageHinge = MathUtil.clamp(voltageHinge, -12, 12);


        arm.setHingePosition(voltageArmHinge);
        arm.extendArm(voltageExtension);
        arm.setHandPosition(voltageHinge);
    }
    //I'll add a way to look at + or - later. Probally a ||
    //boolean expression to account for the difference for both less than or equal to
    //Or just an && statment that checks to see if the Rotat is in range with the 
    //level one rotat
    @Override
    public boolean isFinished() {
        if(arm.getArmHingeRotat()[0] == ArmConstants.levelOneRotat){
            if(arm.getExtensionRotat()[0] == ArmConstants.levelOneExtension) {
                if(arm.getHandHinge() == ArmConstants.levelOneHinge) {
            return true;
                }
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) { 

    }

    
}
