package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.Robot;

public class ShootCannon4 extends Command {
    public ShootCannon4() {
        requires(Robot.i2c);
    }
    protected void initialize() {}
    protected void execute() {
        Robot.i2c.writeToArduino("CN:4");
    }
    protected boolean isFinished() {
        return true;
    }
    protected void end() {}
    protected void interrupted() {}
}
