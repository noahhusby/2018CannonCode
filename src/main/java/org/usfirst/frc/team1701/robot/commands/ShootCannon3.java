package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class ShootCannon3 extends Command {
    public ShootCannon3() {

    }
    protected void initialize() {}
    protected void execute() {
        Cannon.cannon3.set(true);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
