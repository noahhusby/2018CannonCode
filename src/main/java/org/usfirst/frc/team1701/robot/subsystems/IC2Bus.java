package org.usfirst.frc.team1701.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team1701.robot.RobotMap;
import edu.wpi.first.wpilibj.I2C;

public class IC2Bus extends Subsystem {

    private final I2C i2cCommunication = RobotMap.arduinoCommunication;

    public I2C getI2cCommunication() {
        return i2cCommunication;
    }

    public void sendToArduino(int message)
    {
        /**
         * Arduino I2C address defined in RobotMap.
         * Refer to https://goo.gl/Pk1HKm for I2C commands.
         */
        i2cCommunication.write(RobotMap.arduinoInterface, message);
    }

    protected void initDefaultCommand() {

    }
}
