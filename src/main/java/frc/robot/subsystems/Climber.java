package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class Climber extends SubsystemBase {
    private CANSparkMax climbMotor;
    private boolean left;
    public Climber(boolean isLeft){
        left = isLeft;
        if(isLeft)
            climbMotor = new CANSparkMax(Constants.ClimbConstants.CLIMBER_ONE_CAN,MotorType.kBrushless);
        else
            climbMotor = new CANSparkMax(Constants.ClimbConstants.CLIMBER_TWO_CAN, MotorType.kBrushless);
    }
    public void runClimber(double speed){
        climbMotor.set(speed);
    }
    public double getCurrent(){
        if(left){
            return climbMotor.getOutputCurrent();
        }
        return climbMotor.getOutputCurrent();
    }
}
