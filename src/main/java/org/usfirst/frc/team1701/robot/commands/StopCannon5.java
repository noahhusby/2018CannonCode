package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class StopCannon5 extends Command {
    public StopCannon5() {

    }
    protected void initialize() {

    }
    protected void execute() {
        Cannon.cannon5.set(false);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
