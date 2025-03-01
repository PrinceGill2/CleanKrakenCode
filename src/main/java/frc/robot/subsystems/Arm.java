package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    //The four controllers that control the full arm hinge 
    SparkFlex armHingeLT;
    SparkFlex armHingeLB;
    SparkFlex armHingeRT;
    SparkFlex armHingeRB; 
    //The two motors that control the ability of the arm to extend 
    SparkFlex armExtensionL;
    SparkFlex armExtensionR;
    //The motor that controls the hinge on the hand of the arm 
    SparkFlex handHinge;
    //the two motors that control the intake and shoot function of the arm 
    SparkFlex handShootT;
    SparkFlex handShootB;
    //controllers for the armHinge
   public ProfiledPIDController armHingePID;
   public ArmFeedforward armHingeFeedforward;

    //controllers for the extension
   public ProfiledPIDController extensionPID;
   public SimpleMotorFeedforward extensionFeedforward;

    //controllers for the handHinge
   public ProfiledPIDController handHingePID;
   public ArmFeedforward handHingeFeedforward;

    RelativeEncoder handHingeEncoder;
    RelativeEncoder armHingeEncoderL;
    RelativeEncoder armHingeEncoderR;
    RelativeEncoder extensionEncoderL;
    RelativeEncoder extensionEncoderR;



    //Can use setInverted functionality to subtitute if using negative values dont work
    //The getVoltage method does return negative voltage so I assume its cool
    public Arm() { 
                 //I need to set these ID's
                 //Read the description on setVoltage() before calling it
    armHingePID = new ProfiledPIDController(ArmConstants.kPArm, ArmConstants.kPArm, ArmConstants.kPArm, new TrapezoidProfile.Constraints(ArmConstants.maxArmVelocity, ArmConstants.maxArmAccel));    
    armHingeFeedforward = new ArmFeedforward(ArmConstants.kSArm, ArmConstants.kGArm, ArmConstants.kVArm);

    extensionPID  = new ProfiledPIDController(ArmConstants.kPExtend, ArmConstants.kIExtend, ArmConstants.kDExtend, new TrapezoidProfile.Constraints(ArmConstants.maxExtendVelocity, ArmConstants.maxExtendAccel));
    extensionFeedforward = new SimpleMotorFeedforward(ArmConstants.kSExtend, ArmConstants.kGExtend, ArmConstants.kVExtend);

    handHingePID  = new ProfiledPIDController(ArmConstants.kPHinge, ArmConstants.kIHinge, ArmConstants.kDHinge, new TrapezoidProfile.Constraints(ArmConstants.maxHandVelocity, ArmConstants.maxHandAccel));
    handHingeFeedforward  = new ArmFeedforward(ArmConstants.kSHinge, ArmConstants.kGHinge, ArmConstants.kVHinge);

    armHingeLT = new SparkFlex(ArmConstants.hingeLT, SparkLowLevel.MotorType.kBrushless);
    armHingeLB = new SparkFlex(ArmConstants.hingeLB, SparkLowLevel.MotorType.kBrushless);
    armHingeRT = new SparkFlex(ArmConstants.hingeRT, SparkLowLevel.MotorType.kBrushless);
    armHingeRB = new SparkFlex(ArmConstants.hingeRB, SparkLowLevel.MotorType.kBrushless); 
    
    armExtensionL = new SparkFlex(ArmConstants.extendT, SparkLowLevel.MotorType.kBrushless);
    armExtensionR = new SparkFlex(ArmConstants.extendB, SparkLowLevel.MotorType.kBrushless);
    
    handHinge = new SparkFlex(ArmConstants.hingeHand, SparkLowLevel.MotorType.kBrushless);
    
    handShootT = new SparkFlex(ArmConstants.rollerT, SparkLowLevel.MotorType.kBrushless);
    handShootB = new SparkFlex(ArmConstants.rollerB, SparkLowLevel.MotorType.kBrushless);

    handHingeEncoder = handHinge.getEncoder();

    armHingeEncoderL = armHingeLT.getEncoder();
    armHingeEncoderR = armHingeRT.getEncoder();

    extensionEncoderL = armExtensionL.getEncoder();
    extensionEncoderR = armExtensionR.getEncoder();
    
    SparkBaseConfig hingeLeftT = new SparkFlexConfig();
    SparkBaseConfig hingeLeftB = new SparkFlexConfig();
    SparkBaseConfig hingeRightT = new SparkFlexConfig();
    SparkBaseConfig hingeRightB = new SparkFlexConfig();
    SparkBaseConfig armExtendConfigL = new SparkFlexConfig();
    SparkBaseConfig armExtendConfigR = new SparkFlexConfig();
    SparkBaseConfig handHingeConfig = new SparkFlexConfig();

    armExtendConfigL.idleMode(SparkBaseConfig.IdleMode.kBrake);
    armExtendConfigR.idleMode(SparkBaseConfig.IdleMode.kBrake);

    handHingeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    hingeLeftB.follow(armHingeLT);
    hingeRightB.follow(armHingeRT);

    hingeLeftB.idleMode(SparkBaseConfig.IdleMode.kBrake);
    hingeRightB.idleMode(SparkBaseConfig.IdleMode.kBrake);
    hingeLeftT.idleMode(SparkBaseConfig.IdleMode.kBrake);
    hingeRightT.idleMode(SparkBaseConfig.IdleMode.kBrake);

    armExtendConfigL.inverted(true);

    //Dunno if this has to be inverted or not
    handHingeConfig.inverted(false);

    //Might flip this later
    hingeRightB.inverted(true);
    hingeRightT.inverted(true);

    //Uknown if I want to reset or persist any parameters at his point. I'll look into it
    armExtensionL.configure(armExtendConfigL, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    armExtensionR.configure(armExtendConfigR, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    armHingeLT.configure(hingeLeftT, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    armHingeLB.configure(hingeLeftB, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    armHingeRB.configure(hingeRightT, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    armHingeRT.configure(hingeRightB, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    handHinge.configure(handHingeConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //There are other configurations like ramp rate, current limits,
    //voltage compensation and openloop/closedloop stuff that can be used.


    }

    //Maybe getVelocity on the encoders might be useful too?

    public double getHandHinge() { 
        return handHingeEncoder.getPosition();
    }

    public double[] getExtensionRotat() { 
        double [] EncoderExtendRotations  = {extensionEncoderL.getPosition(), extensionEncoderR.getPosition()};
        return EncoderExtendRotations;
    }

    public double[] getArmHingeRotat() { 
        double [] EncoderHingeRotations  = {armHingeEncoderL.getPosition(), armHingeEncoderR.getPosition()};
        return EncoderHingeRotations;
    }
    //deprecated
    public void hingeArm(int magnitude) { 
        if(armHingeEncoderL.getPosition() > ArmConstants.maxiRotatArm) { 
            magnitude = 0;

        }

        armHingeRT.setVoltage(magnitude);
        armHingeRB.setVoltage(magnitude);

        armHingeLT.setVoltage(magnitude);
        armHingeLB.setVoltage(magnitude);
    }
    //deprecated
    public void moveFirstExtension(int direction) { 
        if(extensionEncoderL.getPosition() > ArmConstants.maxiRotatExtendT) { 
            direction = 0;

        }

        armExtensionL.setVoltage(5 * direction);
    }
    //deprecated
    public void MoveSecondExtension(int direction) { 
        if(extensionEncoderR.getPosition() > ArmConstants.maxiRotatExtendB) { 
            direction = 0;
        }

        armExtensionR.setVoltage(5 * direction);
    }
    //deprecated
    public void hingeHand(double magnitude) { 
        if(extensionEncoderL.getPosition() > ArmConstants.maxiRotatExtendT) { 
            magnitude = 0;

        }

        handHinge.setVoltage(magnitude * .5);
    }
    //I can add something later that auto stops it once the coral is held
    //not deprecated, might adjust I dunno
    public void rollers(int direction ) { 
        handShootB.setVoltage(5 * direction);
        handShootB.setVoltage(5 * direction);
    }

    public void setHingePosition(double voltage) { 
        armHingeLT.setVoltage(voltage);
        armHingeRT.setVoltage(voltage);
    }

    public void extendArm(double voltage) { 
        armExtensionL.setVoltage(voltage);
        armExtensionR.setVoltage(voltage);
    }

    public void setHandPosition(double voltage) {
        handHinge.setVoltage(voltage);

    }

    public boolean withinBounds() {


        return false;
    }


    
  
}
