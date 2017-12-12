/**
 * subsystems/I2CBus.java
 *
 * Created by Noah Husby on 12/12/2017.
 *
 * Copyright (c) 2017 Team 1701 (Robocubs)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
