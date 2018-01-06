package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class StopCannon2 extends Command {
    public StopCannon2() {

    }
    protected void initialize() {

    }
    protected void execute() {
        Cannon.cannon2.set(false);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
