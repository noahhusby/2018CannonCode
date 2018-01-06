package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class ShootCannon4 extends Command {
    public ShootCannon4() {

    }
    protected void initialize() {}
    protected void execute() {
        Cannon.cannon4.set(true);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
