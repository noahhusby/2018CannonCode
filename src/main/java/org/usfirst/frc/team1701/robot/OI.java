/**
 * OI.java
 *
 * Created by Noah Husby on 1/7/2018.
 *
 * Copyright (c) 2018 Team 1701 (Robocubs)
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
package org.usfirst.frc.team1701.robot;


import org.usfirst.frc.team1701.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // Create our Joystick and Button controls.
    public JoystickButton gearMode;
    public JoystickButton preciseMode;
    public JoystickButton launchCannons;
    public JoystickButton cannon1;
    public JoystickButton cannon2;
    public JoystickButton cannon3;
    public JoystickButton cannon4;
    public JoystickButton cannon5;
    public JoystickButton cannon6;

    public Joystick drive_FB;
    public Joystick drive_T;
    public Joystick operation;
    public Joystick turret_PN;
    public Joystick turret_R;
    public OI() {
        // Populate the variables above.
        operation = new Joystick(1); // Operation I/O controller to be removed, going to use turret_R as input (New controller not built yet)
        drive_T = new Joystick(0);
        drive_FB = new Joystick(2);
        turret_R = new Joystick(3);
        turret_PN = new Joystick(4);
        preciseMode = new JoystickButton(drive_FB, 1);
        preciseMode.whileHeld(new Preciseify());
        gearMode = new JoystickButton(drive_FB, 3);
        gearMode.whenPressed(new ToggleReverse());
        cannon1 = new JoystickButton(operation, 1);
        cannon1.whenPressed(new ShootCannon1());
        cannon1.whenReleased(new StopCannon1());
        cannon2 = new JoystickButton(operation, 2);
        cannon2.whenPressed(new ShootCannon2());
        cannon2.whenReleased(new StopCannon2());
        cannon3 = new JoystickButton(operation, 3);
        cannon3.whenPressed(new ShootCannon3());
        cannon3.whenReleased(new StopCannon3());
        cannon4 = new JoystickButton(operation, 4);
        cannon4.whenPressed(new ShootCannon4());
        cannon4.whenReleased(new StopCannon4());
        cannon5 = new JoystickButton(operation, 6);
        cannon5.whenPressed(new ShootCannon5());
        cannon5.whenReleased(new StopCannon5());
        cannon6 = new JoystickButton(operation, 7);
        cannon6.whenPressed(new ShootCannon6());
        cannon6.whenReleased(new StopCannon6());
        launchCannons = new JoystickButton(operation, 10);











    }
    // Return drive_FB
    public Joystick getDrive_FB() {
        return drive_FB;
    }
    // Return drive_T
    public Joystick getDrive_T() {
        return drive_T;
    }
    // Return operation
    public Joystick getOperation() {
        return operation;
    }
    // Return turret_PN
    public Joystick getTurret_PN() {
        return turret_PN;

    }
    // Return turret_R
    public Joystick getTurret_R() {
        return turret_R;

    }
}