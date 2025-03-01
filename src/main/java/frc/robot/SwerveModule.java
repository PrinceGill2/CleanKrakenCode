package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor; //we're running krakens so thats what these are 
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder; //this should be set to the CTRE mag encoders

    CANBus canbus; //This creates a canivore object so the code knows where the motor controllers are

    //this sends out a bit more oomph for any predicted disturbances that could happen.
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    //Kind of like pwm in the sense that it wants frequency with which to send voltage
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0); //recieves a voltage value, unknown at this point how its determined
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0); //recieves a speed 

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0); //the optimizaton happens in this file so this is the unoptimized value

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        //Should create a canbus object to pass onto the TalonFx objects in the SwerveModule 
        //have to figure out the name of the canivore
        CANBus canbus = new CANBus("PLACE HOLDER");
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);  
        //getConfigurator and swerveVancoderConfig come from the same library

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig); //The threshold and the threshold time no longer exist
        resetToAbsolute(); //resets the angle encoders

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig); //this configuration works just fine
        mDriveMotor.getConfigurator().setPosition(0.0);

        //a drive endcoder CAN be added but the chassiss speeds object and odometry object can be used to track the position of the 
        //robot on the field
    }
    //this method is called whenever drive is called, drive is called when the teleoperated command is run and as its the default command its 
    //called all the time
    //Open loop basically means that the motor that sets the speed of the wheel/bot 
    //operates without the known position of the wheel
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); //angle of the wheel is optimized, im guessig its just a new object\
                                                                                   //with a seperate memory address
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations())); //this sets the angle, feed forward makes sure it has the voltage
                                                                                               //to move the right distance
        setSpeed(desiredState, isOpenLoop); //the drive motor has yet to set the speed of the motor
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed; //get a percentage, 0-1
            mDriveMotor.setControl(driveDutyCycle);
            SmartDashboard.putNumber("DriveValue?", desiredState.speedMetersPerSecond);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond); //Heres the different, if its closed loop feed forward is used
                                                                                                       //So the wheel knows where it it when it moved around
            mDriveMotor.setControl(driveVelocity);
        }
    }
            //Called CanCoder
    public Rotation2d getCANcoder(){
    
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations(); //magnetic encoders ARE absolute encoders so this is curious
        mAngleMotor.setPosition(absolutePosition); 
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){ //can be used to find the position of the robot 
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }
}