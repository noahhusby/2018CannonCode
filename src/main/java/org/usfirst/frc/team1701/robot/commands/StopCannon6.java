package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class StopCannon6 extends Command {
    public StopCannon6() {

    }
    protected void initialize() {

    }
    protected void execute() {
        Cannon.cannon6.set(false);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
