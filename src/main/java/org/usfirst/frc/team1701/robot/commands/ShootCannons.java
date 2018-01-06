package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class ShootCannons extends Command {
    public ShootCannons() {

    }
    protected void initialize() {}
    protected void execute() {
        Cannon.cannon1.set(true);
        Cannon.cannon2.set(true);
        Cannon.cannon3.set(true);
        Cannon.cannon4.set(true);
        Cannon.cannon5.set(true);
        Cannon.cannon6.set(true);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
