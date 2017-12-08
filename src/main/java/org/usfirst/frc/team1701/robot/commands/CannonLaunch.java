package org.usfirst.frc.team1701.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1701.robot.Robot;

public class CannonLaunch extends Command {
    protected void initialize()
    {

    }

    protected void execute()
    {
        // Tell arduino to launch cannons
        Robot.i2c.communicate(11);
        Robot.i2c.communicate(12);
        Robot.i2c.communicate(13);
        Robot.i2c.communicate(14);
        Robot.i2c.communicate(15);
        Robot.i2c.communicate(16);
    }
    @Override
    protected boolean isFinished() {
        return false;
    }

}
