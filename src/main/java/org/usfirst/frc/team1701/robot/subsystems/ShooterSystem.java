/**
 * subsystems/ShooterSystem.java
 *
 * Created by Nicholas Hubbard on 11/10/2017.
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
import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.RobotMap;
import org.usfirst.frc.team1701.robot.commands.StopHighShooter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ShooterSystem extends Subsystem {
	private final Encoder shootingEncoder = RobotMap.shooterSystemShootingEncoder;
	private final DoubleSolenoid shooterLoader = RobotMap.shooterSystemShooterLoader;
	private final SpeedController mainShoot = RobotMap.shoot1;
	private final SpeedController hopperMotor = RobotMap.hopperMotor;
	private final double PULSES = 360.0;
	private final int TARGET_SPEED = 5500;
	private boolean shooterActive;
	private boolean hopperReverse = true;
	public boolean isShooterActive() {
		return shooterActive;
	}
	public void setShooterAction(boolean shooterActive) {
		this.shooterActive = shooterActive;
	}
	public void setup() {
		shootingEncoder.setDistancePerPulse(1.0);
		shooterActive = false;
	}
	public void fire() {
		shooterActive = true;
		mainShoot.set(1.0);
		double speed = shootingEncoder.getRate();
		speed = (int) (speed * 400 + .05) / 100;
		SmartDashboard.putNumber("Shooter Speed: ", speed);
		if(speed >= TARGET_SPEED) {
			hopperReverse = false;
		}
		if (hopperReverse) {
			hopperMotor.set(1);
		} else {
			hopperMotor.set(-1);
			if (RobotMap.lightsLED2.get() == Relay.Value.kForward) {
				RobotMap.lightsLED2.set(Relay.Value.kOn);
			} else {
				RobotMap.lightsLED2.set(Relay.Value.kReverse);
			}
		}
	}
	public void hopperRun(double speed) {
		if(!shooterActive) {
			hopperMotor.set(-1 * speed);
		}
	}
	public void hopperStop() {
		hopperMotor.set(0.0);
	}
	public void stop() {
		mainShoot.set(0);
		hopperMotor.set(0);
		shooterActive = false;
		hopperReverse = true;
		if (RobotMap.lightsLED2.get() == Relay.Value.kForward || RobotMap.lightsLED2.get() == Relay.Value.kOn) {
			RobotMap.lightsLED2.set(Relay.Value.kForward);
		} else {
			RobotMap.lightsLED2.set(Relay.Value.kOff);
		}
	}
	public void initDefaultCommand() {
		setDefaultCommand(new StopHighShooter());
	}
}