package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

public class LED_VIBE extends Command {
    // Varyables 
    private LED lights;

    public LED_VIBE(LED ligyboi) {
        // Shooter Object
        lights = ligyboi;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if (lights.getState() != LED.STATUS.VIBE) {
            lights.setMode(LED.STATUS.VIBE); 
        }
    }
    @Override
    public void end(boolean interrupted){
        lights.defaultState();
    }
    @Override
    public boolean isFinished(){return false;}
}
