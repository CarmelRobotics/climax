package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
//🤫🧏‍♂️
public class ShootMaxxer extends SubsystemBase {
    private TalonFX shootmotorone;
    private TalonFX shootmotortwo;
    private CANSparkMax pivotmotorone;
    private CANSparkMax pivotmotortwo;
    private CANSparkMax pivotmotorthree;
    
    public double currentAngle;
   // private double amountMove;
    private double targetAngle;
    SwerveSubsystem swerve;
    PIDController pivotController;
    public ShootMaxxer(SwerveSubsystem s){

        shootmotorone = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_MOTORONE_CAN);
        shootmotortwo = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_MOTORTWO_CAN);
        //pivotmotor = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_PIVOT_CAN);
        pivotmotorone = new CANSparkMax(frc.robot.Constants.Shooter.SHOOTER_PIVOTONE_CAN, MotorType.kBrushless);
        pivotmotortwo = new CANSparkMax(frc.robot.Constants.Shooter.SHOOTER_PIVOTTWO_CAN, MotorType.kBrushless);
        pivotmotorthree = new CANSparkMax(frc.robot.Constants.Shooter.SHOOTER_PIVOTTHREE_CAN, MotorType.kBrushless);
        pivotmotortwo.follow(pivotmotorone);
        pivotmotorthree.follow(pivotmotorone);
        pivotController = Constants.Shooter.SHOOTER_CONTROLLER;
        pivotmotorone.getPIDController().setP(0.1);
        swerve = s;
        currentAngle = pivotmotorone.getEncoder().getPosition();
        targetAngle = 90;
    }
    @Override
    public void periodic(){
        // currentAngle = pivotmotorone.getEncoder().getPosition() * pivotmotorone.getEncoder().getPositionConversionFactor();
        // System.out.println(currentAngle);
        // System.out.println(targetAngle);
        // pivotmotorone.set(pivotController.calculate(currentAngle, targetAngle));

        // if(currentAngle == targetAngle) pivotmotorone.set(0);
        // if(isFalling()){pivotmotorone.set(Constants.Shooter.FALL_CANCEL_SPEED);}
        RelativeEncoder builtin = pivotmotorone.getEncoder();
        double pos = builtin.getPosition();
        double targetpos = (targetAngle/360.0);
        if (Math.abs(pos-targetpos) > 0.04) {
            if (pos < targetpos) {
                pivotmotorone.set(0.02);
            }else {
                pivotmotorone.set(-0.02);
            } 
        }else {
            pivotmotorone.set(0);
        }
        SmartDashboard.putNumber("Wrist Encoder Value", pos);
        SmartDashboard.putNumber("Wrist Target Value", (targetAngle/360.0));
    }
    
    public void shoot(double speed){
        shootmotorone.set(speed);
        shootmotortwo.set(-speed);
    }
    public void pivot(double speed){
        pivotmotorone.set(speed);
    }
    public double getPosition(){
        return pivotmotorone.getEncoder().getPosition();
    }
    public void pivotToAngle(double angle){
        targetAngle = angle;
    }
    public boolean isFalling(){
        return ((pivotmotorone.getOutputCurrent() == 0));
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
        return Math.atan(Constants.FieldConstants.SPEAKER_HEIGHT / drive.getPosition().getX() - Constants.FieldConstants.SPEAKER_X_BLUE);
        } else {
            return Math.atan(Constants.FieldConstants.SPEAKER_HEIGHT / drive.getPosition().getX() - Constants.FieldConstants.SPEAKER_X_RED);
        }
    }
}
