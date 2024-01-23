package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private TalonFX shootmotorone;
    private TalonFX shootmotortwo;
    private TalonFX pivotmotor;
    public double currentAngle;
    private double amountMove;
    PIDController pivotController;
    public Shooter(){
        shootmotorone = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_MOTORONE_CAN);
        shootmotortwo = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_MOTORTWO_CAN);
        pivotmotor = new TalonFX(frc.robot.Constants.Shooter.SHOOTER_PIVOT_CAN);
        pivotmotor.setPosition(90);
        pivotController = Constants.Shooter.PIVOT_CONTROLLER;
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
    
}
