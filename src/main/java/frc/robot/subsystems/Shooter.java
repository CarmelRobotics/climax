package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private TalonFX shootmotorone;
    private TalonFX shootmotortwo;
    private TalonFX pivotmotor;
    public double currentAngle;
    private double amountMove;
    SwerveSubsystem swerve;
    PIDController pivotController;
    public Shooter(SwerveSubsystem s){
        shootmotorone = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_MOTORONE_CAN);
        shootmotortwo = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_MOTORTWO_CAN);
        pivotmotor = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_PIVOT_CAN);
        pivotmotor.setPosition(0);
        pivotController = Constants.Shooter.PIVOT_CONTROLLER;
        swerve = s;
    }
    @Override
    public void periodic(){
        currentAngle = ((pivotmotor.getPosition().getValue()) % 1);
    }
    public void shoot(double speed){
        shootmotorone.set(speed);
        shootmotortwo.set(-speed);
    }
    public void pivot(double speed){
        pivotmotor.set(speed);
    }
    public void pivotToAngle(double angle){
        pivotmotor.set(pivotController.calculate(currentAngle, angle));

    }
    public void autoAim(){
        pivotToAngle(getSpeakerAngle(swerve));
    }
    public double getSpeakerAngle(SwerveSubsystem drive){
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
        return Math.atan(Constants.FieldConstants.SPEAKER_HEIGHT / drive.getPosition().getX() - Constants.FieldConstants.SPEAKER_X_BLUE);
        } else {
            return Math.atan(Constants.FieldConstants.SPEAKER_HEIGHT / drive.getPosition().getX() - Constants.FieldConstants.SPEAKER_X_RED);
        }
    }
}
