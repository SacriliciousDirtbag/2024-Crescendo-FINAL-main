package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeController extends Command {
    
    public IntakeController() {
        
    }
    
    @Override
    public void initialize() {
        s_feederSubsystem.goFeederArmState(aState.FLOAT);
    }

    @Override
    public boolean isFinished() {
        return true;
}
