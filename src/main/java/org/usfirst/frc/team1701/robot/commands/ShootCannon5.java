package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.subsystems.Cannon;

public class ShootCannon5 extends Command {
    public ShootCannon5() {

    }
    protected void initialize() {}
    protected void execute() {
        Cannon.cannon5.set(true);
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}