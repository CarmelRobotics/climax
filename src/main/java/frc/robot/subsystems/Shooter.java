package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RockinTalon;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.LED.STATUS;

public class Shooter extends SubsystemBase {
    private RockinTalon shootmotorone;
    private RockinTalon shootmotortwo;
    private CANSparkMax pivotmotorone;
    //private AHRS navx = new AHRS(I2C.Port.kOnboard);
    public double currentAngle;
   // private double amountMove;
    private double targetAngle;
    private SwerveSubsystem swerve;
    private LED l;
    private ProfiledPIDController pivotController;
    private ArmFeedforward ffController;
    private DigitalInput limitswitch;
    private CANcoder encoder;
    private Vision limelight;
    private BTS bts;
    private Rotation2d pivotGoal = Rotation2d.fromDegrees(45);
    private ShooterState state = ShooterState.STOW;
    public double secondsPerDegree;
    private double pidOutput;
    private double ffOutput;
    public Shooter(SwerveSubsystem s, LED ledSystem){
        limitswitch = new DigitalInput(0);
        encoder = new CANcoder(21);
        bts = new BTS();
        shootmotorone = new RockinTalon(frc.robot.Constants.Shooter.SHOOTER_MOTORONE_CAN);
        shootmotortwo = new RockinTalon(frc.robot.Constants.Shooter.SHOOTER_MOTORTWO_CAN);
        limelight = new Vision();
        pivotmotorone = new CANSparkMax(frc.robot.Constants.Shooter.SHOOTER_PIVOTONE_CAN, MotorType.kBrushless);
        pivotmotorone.setSmartCurrentLimit(frc.robot.Constants.Shooter.PIVOT_CURRENT_LIMIT);
        pivotController = Constants.Shooter.SHOOTER_PID_CONTROLLER;
        ffController = Constants.Shooter.SHOOTER_FF_CONTROLLER;
        swerve = s;
        l = ledSystem;
    }
    @Override
    public void periodic(){
       SmartDashboard.putNumber("Pivot angle", getPivotAngle().getDegrees());
       SmartDashboard.putNumber("Goal angle", pivotGoal.getDegrees());
       SmartDashboard.putString("Current State",state.toString());
       calcAndApplyControllers();
       if(limitswitch.get()){
        pivot(0);
       }
       switch (state) {
        case SPEAKERSHOOT:
            SpeakerShoot();
            break;
        case TRANSFER:
            transfer();
            break;
        case STOW:
            stow();
            break;
        case ERROR:
            error();
            break;
        case AMPAIM:
            AutoShootAmp();
            break;
        case SPEAKERAIM:
            pivotToAngle(getSpeakerAngle(swerve));
            break;
        default:
            break;
       }
       
       
    }
    public static enum ShooterState{
        SPEAKERSHOOT,
        TRANSFER,
        STOW,
        ERROR,
        AMPAIM,
        SPEAKERAIM
    }
    public void setMode(ShooterState state){
        this.state = state;
    }
    public Command setShooterMode(ShooterState state){
        return run(() -> setMode(state));
    }
    public Command shootNote(){
        return run(() -> shoot(1));
    }
    public void calcAndApplyControllers(){
        pidOutput = pivotController.calculate(getPivotAngle().getRadians(), pivotGoal.getRadians());
        State profSetpoint = pivotController.getSetpoint();
        ffOutput = ffController.calculate(profSetpoint.position, profSetpoint.velocity);
        pivotmotorone.setVoltage(pidOutput + ffOutput);
    }
    //used for going under stage
    public void stow(){
        pivotToAngle(15);
        shoot(0);
    }
    public void SpeakerShoot(){
        autoShoot();
    }
    //intake -> shooter
    public void transfer(){
        shoot(1);
        bts.set(1);
    }
    //if something goofy happens
    public void error(){
        //do the LED error signal when LED code ready
        pivot(0);
        shoot(1);
    }  


    public Rotation2d getPivotAngle(){
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue());
    }
    //public double getNavxPitch(){
      //  return navx.getPitch();
    //}
    public void shoot(double speed){
        if (speed != 0) {
            l.setMode(STATUS.SHOOTING);
        } else {
            l.setMode(STATUS.DEFAULT);
        }
        shootmotorone.set(speed);
        shootmotortwo.set(-speed);
    }
    public void pivot(double speed){
        pivotmotorone.set(speed);
    }
    public double getPosition(){
        return pivotmotorone.getEncoder().getPosition();
    }
    public double getDegree(){
        return currentAngle;
    }
    public double getPivotVelocity(){
        return (encoder.getVelocity().getValue()) * (2*Math.PI);
    }
    // public void setAngle(double degree){
    //     currentAngle = degree;
        
    // }
    public void pivotToAngle(double angle){
        pivotGoal = Rotation2d.fromDegrees(angle);
        pivotController.reset(getPivotAngle().getRadians(),getPivotVelocity());
    }
    public boolean isFalling(){
        return ((pivotmotorone.getOutputCurrent() == 0));
    }
    public void splinePivot(){

    }
    
    public void autoShoot(){
        pivotToAngle(getSpeakerAngle(swerve));
        shoot(1);
    }
    public void AutoShootAmp(){
        pivotToAngle(FieldConstants.AMP_ANGLE);
        shoot(1);
    }
    public double getSpeakerAngle(SwerveSubsystem drive){
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
        return Math.atan(Constants.FieldConstants.SPEAKER_HEIGHT / limelight.getPoseX() - Constants.FieldConstants.SPEAKER_X_BLUE);
        } else {
            return Math.atan(Constants.FieldConstants.SPEAKER_HEIGHT / limelight.getPoseX() - Constants.FieldConstants.SPEAKER_X_RED);
        }
    }
}
