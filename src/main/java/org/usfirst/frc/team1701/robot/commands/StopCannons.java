package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class StopCannons extends Command {
    public StopCannons() {

    }
    protected void initialize() {

    }
    protected void execute() {
        Cannon.cannon1.set(false);
        Cannon.cannon2.set(false);
        Cannon.cannon3.set(false);
        Cannon.cannon4.set(false);
        Cannon.cannon5.set(false);
        Cannon.cannon6.set(false);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
