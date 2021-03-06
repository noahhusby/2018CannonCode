/**
 * Robot.java
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

import org.usfirst.frc.team1701.robot.subsystems.Cannon;
import org.usfirst.frc.team1701.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * The VM on the RoboRIO is automatically configured to run this class.
 * If you change the name of this class or the package, change it in Gradle
 * and here.
 */
public class Robot extends IterativeRobot {
    Command autonomousCommand;
    // Initialize our various subsystems.
    public static OI oi;
    public static DriveTrain driveTrain;
    public static Cannon cannons;

    /**
     * This function is run when the robot is first started up.
     */
    public void robotInit() {
        RobotMap.init(); // Initialize our RobotMap.
        setupNetworkTable(); // ZeroMQ me up, Scotty!
        // Initialize our subsystems.
        driveTrain = new DriveTrain();

        oi = new OI(); // If you move this... you're gonna have a bad time
    }
    public void setupNetworkTable() {

    }
    /**
     * These functions are called when the robot is disabled.
     */
    public void disabledInit() {}
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }
    /**
     * These functions are called when autonomous mode is started.
     */
    public void autonomousInit() {
        if(autonomousCommand != null) {
            autonomousCommand.start();
        }
    }
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }
    /**
     * These functions are called during teleop.
     */
    public void teleopInit() {
        if(autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
    public void teleopPeriodic() {

        Scheduler.getInstance().run();

    }
    /**
     * This function is called periodically during test mode.
     */
    public void testPeriodic() {
    }
}