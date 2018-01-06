package org.usfirst.frc.team1701.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team1701.robot.RobotMap;

public class Cannon extends Subsystem {

    public static Solenoid cannon1 = RobotMap.cannon1;
    public static Solenoid cannon2 = RobotMap.cannon2;
    public static Solenoid cannon3 = RobotMap.cannon3;
    public static Solenoid cannon4 = RobotMap.cannon4;
    public static Solenoid cannon5 = RobotMap.cannon5;
    public static Solenoid cannon6 = RobotMap.cannon6;

    @Override
    protected void initDefaultCommand() {

    }
}
