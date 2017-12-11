package org.usfirst.frc.team1701.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team1701.robot.RobotMap;
import edu.wpi.first.wpilibj.I2C;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

public class I2CBus extends Subsystem {



    private final I2C i2cCommunication = RobotMap.arduinoCommunication;

    public I2C getI2cCommunication() {
        return i2cCommunication;
    }

    public void sendTo(int device, int value)
    {
        i2cCommunication.write(device, value);
    }


    public void communicate(int message)
    {
        /**
         * Arduino I2C address defined in RobotMap.
         * Refer to https://goo.gl/Pk1HKm for I2C commands.
         */
        i2cCommunication.write(RobotMap.arduinoInterface, message);
    }

    public void sendToDisplay(int toSend, int display)
    {
        for (int i=0; i<4; i++)
        {
            i2cCommunication.write(display, toSend);
        }
    }

    protected void initDefaultCommand() {

    }
}
