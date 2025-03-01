package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;


public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), //SwerveDriveKinematic is just the dimension of the robot
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
                                //this will be the top amount the current can hold
        public static final int angleCurrentLimit = 40; //Originally 25
                                //This will be the new lower limit
        public static final int angleCurrentLowerLimit = 25; //Originally 40 When Threshold Was Present
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleSupplyCurrentLimitEnable = true;

                                //this will be the top amount the current can hold
        public static final int driveCurrentLimit = 60; //Originally 35
                                //This will be the new lower limit
        public static final int driveCurrentLowerLimit = 35; //Originally 60 When Threshold Was Present
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveSupplyCurrentLimitEnable = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.0; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        //Make sure to set the offsets in degrees NOT radians

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1; //programmed in a circle
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        //Use the example auto to test what the correct kP values would be.
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1; //Make sure to change these, 
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public final class aprilTagConstants {
        //I'll worry about adding motion profiling, motor controllers, and feedforward later

        //this will be multiplied by the angle taken from the limelights
        //That value will be treated as an angular velocity that the robot will
        //rotate at 
        public static final double KpTheta = 0;

        //Using the distances given by the limelights 
        //This will be multiplied by the distance to find a speed to go 
        //that way it gradually slows down on the approach.
        //priority on the X, I want it lined up with the thing first
        public static final double KpXVelocity = 0;
        public static final double KpYVelocity = 0;

    
    
    }

    public final class ArmConstants{
        public static final int hingeLT = 0;
        public static final int hingeLB = 0;
        public static final int hingeRT = 0;
        public static final int hingeRB = 0;

        public static final int extendT = 0;
        public static final int extendB = 0;

        public static final int rollerT = 0;
        public static final int rollerB = 0;

        public static final int hingeHand = 0;

        //These values are in the unit "rotations" VV

        //the most rotations that the vortex motors at the base can 
        //go before the arm rips itself off of the armhinge
        public static final double  maxiRotatArm = 0;

        //The maximum rotations for the extension mechanism
        //To ensure it doesnt overextend and rip out
        public static final double maxiRotatExtendT = 0;
        public static final double maxiRotatExtendB = 0;

        //The maximum rotations for the hand to ensure it 
        //doesnt rip out
        public static final double maxiRotatHand = 0;

        //PID values for the Arm
        public static final double kPArm = 0;
        public static final double kIArm = 0;
        public static final double kDArm = 0;

        //feedforward constants for the Arm
        public static final double kSArm = 0;
        public static final double kGArm = 0;
        public static final double kVArm = 0;

        //PID values for the Extension
        public static final double kPExtend = 0;
        public static final double kIExtend = 0;
        public static final double kDExtend = 0;

        //feedforward constants for the Extension
        public static final double kSExtend = 0;
        public static final double kGExtend = 0;
        public static final double kVExtend = 0;

        //PID values for the handHinge
        public static final double kPHinge = 0;
        public static final double kIHinge = 0;
        public static final double kDHinge = 0;

        //feedforward constants for the handHinge
        public static final double kSHinge = 0;
        public static final double kGHinge = 0;
        public static final double kVHinge = 0;

        public static final double maxArmVelocity = 4;
        public static final double maxArmAccel = 2;

        public static final double maxExtendVelocity = 0;
        public static final double maxExtendAccel = 0;

        public static final double maxHandVelocity = 0;
        public static final double maxHandAccel = 0;

        //L2 range for the arm
        public static final double levelOneRotat = 0;
        public static final double levelOneExtension = 0;
        public static final double levelOneHinge = 0;

        //L3 range for the arm
        public static final double levelTwoRotat = 0;
        public static final double levelTwoExtension = 0;
        public static final double levelTwoHinge = 0;

        //L4 range for the arm
        public static final double levelThreeRotat = 0;
        public static final double levelThreeExtension = 0;
        public static final double levelThreeHinge = 0;

        //Current limits on the arm, extension and hand goes here
        

    }

}
