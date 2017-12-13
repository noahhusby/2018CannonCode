/**
 * OI.java
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
package org.usfirst.frc.team1701.robot;

import org.usfirst.frc.team1701.robot.commands.CannonLaunch;
import org.usfirst.frc.team1701.robot.commands.Preciseify;
import org.usfirst.frc.team1701.robot.commands.ToggleReverse;
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
    public JoystickButton turboMode;
    public JoystickButton launchCannon;
    public Joystick drive_FB;
    public Joystick drive_T;
    public Joystick operation;
    public OI() {
        // Populate the variables above.
        operation = new Joystick(1);
        drive_T = new Joystick(0);
        drive_FB = new Joystick(2);
        preciseMode = new JoystickButton(drive_FB, 1);
        preciseMode.whileHeld(new Preciseify());
        gearMode = new JoystickButton(drive_FB, 3);
        gearMode.whenPressed(new ToggleReverse());
        launchCannon = new JoystickButton(operation, 4); //Change button number to match configuration
        launchCannon.whenPressed(new CannonLaunch());


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
}