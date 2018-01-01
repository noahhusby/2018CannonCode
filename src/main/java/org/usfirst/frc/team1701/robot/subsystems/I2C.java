package org.usfirst.frc.team1701.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team1701.robot.OI;
import org.usfirst.frc.team1701.robot.Robot;
import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team1701.robot.RobotMap;


public class I2C extends Subsystem {

    public void writeToArduino(String writeString)
    {
        char[] CharArray = writeString.toCharArray();
        byte[] WriteData = new byte[CharArray.length];
        for (int i = 0; i < CharArray.length; i++) {
            WriteData[i] = (byte) CharArray[i];
            RobotMap.Wire.transaction(WriteData, WriteData.length, null, 0);
        }

    }

    @Override
    protected void initDefaultCommand() {

    }
}
