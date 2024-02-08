package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMaxxer extends SubsystemBase{
    private MotorController intakemotorOne;
    private MotorController intakemotorTwo;
    private AnalogInput distanceSensor;
    public IntakeMaxxer(){
        distanceSensor = new AnalogInput(4);
        intakemotorOne = new CANSparkMax(Constants.Intake.INTAKE_CAN_ONE,MotorType.kBrushless );
        intakemotorTwo = new CANSparkMax(Constants.Intake.INTAKE_CAN_TWO, MotorType.kBrushless);
    }
    public void runIntake(double speed){
        intakemotorOne.set(speed);
        intakemotorTwo.set(-speed);
    }
    public double getDist(){
        return AnalogInput.getGlobalSampleRate();
    }
    public boolean hasNote(){
        return (getDist() < 1.5);
        
    }

}
