package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//
public class Intake extends SubsystemBase{
    private MotorController intakemotorOne;
    private IntakeState state = IntakeState.DEFAULT;
    //private MotorController intakemotorTwo;
    private AnalogInput distanceSensor;
    public Intake(){
        distanceSensor = new AnalogInput(4);
        intakemotorOne = new CANSparkMax(Constants.Intake.INTAKE_CAN_ONE,MotorType.kBrushless );
        //intakemotorTwo = new CANSparkMax(Constants.Intake.INTAKE_CAN_TWO, MotorType.kBrushless);
    }
    @Override
    public void periodic(){
        SmartDashboard.putString("Current Intake State", state.toString());
        switch(state){
            case INTAKING:
                runIntake(-0.75);
                break;
            case OUTTAKING:
                runIntake(1);
                break;
            case TRANSFERING:
                runIntake(-1);
            default:
                runIntake(-0);
                break;
            
        }
    }
    public void runIntake(double speed){
        intakemotorOne.set(speed);
        //intakemotorTwo.set(-speed);
    }
    public double getDist(){
        return AnalogInput.getGlobalSampleRate();
    }
    public boolean hasNote(){
        return (getDist() < 1.5);
        
    }
    public Command setIntakeState(IntakeState state){
        return run(() -> setState(state));
    }
    public IntakeState setState(IntakeState state){
        if(this.state != IntakeState.DEFAULT){
            this.state = IntakeState.DEFAULT;
            return this.state;
        }
        this.state = state;
        return this.state;
    }
    public static enum IntakeState
    {
        INTAKING,
        OUTTAKING,
        TRANSFERING,
        DEFAULT
    }

}
