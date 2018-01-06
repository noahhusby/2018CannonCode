/**
 * RobotMap.java
 *
 * Created by Noah Husby on 12/30/2017.
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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    // Create all of our initial variables. To be populated.
    public static final int arduinoInterface = 0;
    public static WPI_TalonSRX driveTrainLeft_1;
    public static WPI_TalonSRX driveTrainLeft_2;
    public static WPI_TalonSRX driveTrainRight_1;
    public static WPI_TalonSRX driveTrainRight_2;
    public static WPI_TalonSRX turretPN;
    public static WPI_TalonSRX turretRT;
    public static AHRS navx;
    public static DifferentialDrive driveTrainRM;
    public static DifferentialDrive turretRTDrive;
    public static DifferentialDrive turretPNDrive;
    public static SpeedControllerGroup driveTrainRMleft;
    public static SpeedControllerGroup driveTrainRMright;
    public static Solenoid cannon1;
    public static Solenoid cannon2;
    public static Solenoid cannon3;
    public static Solenoid cannon4;
    public static Solenoid cannon5;
    public static Solenoid cannon6;


    public static void init() {
        // Populate these values.



        driveTrainLeft_1 = new WPI_TalonSRX(1);
        driveTrainLeft_2 = new WPI_TalonSRX(2);
        driveTrainRight_1 = new WPI_TalonSRX(3);
        driveTrainRight_2 = new WPI_TalonSRX(4);
        turretRT = new WPI_TalonSRX(5);
        turretPN = new WPI_TalonSRX(6);
        cannon1 = new Solenoid(0,2);
        cannon2 = new Solenoid(0,3);
        cannon3 = new Solenoid(0,4);
        cannon4 = new Solenoid(0,5);
        cannon5 = new Solenoid(0,6);
        cannon6 = new Solenoid(0, 7);

        driveTrainRMleft = new SpeedControllerGroup(driveTrainLeft_1,driveTrainLeft_2);
        driveTrainRMright = new SpeedControllerGroup(driveTrainRight_1, driveTrainRight_2);

        driveTrainRM = new DifferentialDrive(driveTrainRMleft, driveTrainRMright); //Master Drive Train

        turretRTDrive = new DifferentialDrive(turretRT, turretRT);

        turretPNDrive = new DifferentialDrive(turretPN, turretPN);

    }
}