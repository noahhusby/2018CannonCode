package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class ShootCannon2 extends Command {
    public ShootCannon2() {

    }
    protected void initialize() {}
    protected void execute() {
    Cannon.cannon2.set(true);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
