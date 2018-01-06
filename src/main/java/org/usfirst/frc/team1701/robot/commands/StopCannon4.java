package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class StopCannon4 extends Command {
    public StopCannon4() {

    }
    protected void initialize() {

    }
    protected void execute() {
        Cannon.cannon4.set(false);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
