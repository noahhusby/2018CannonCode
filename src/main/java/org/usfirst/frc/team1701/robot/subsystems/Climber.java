/**
 * subsystems/Climber.java
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
import org.usfirst.frc.team1701.robot.RobotMap;
import org.usfirst.frc.team1701.robot.commands.StopClimb;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;
public class Climber extends Subsystem {
	CANTalon climberMotor1 = (CANTalon) RobotMap.climberMotor1;
	CANTalon climberMotor2 = (CANTalon) RobotMap.climberMotor2;
	public static double CLIMB_POWER = .70;
	public static int CLIMB_DIRECTION = -1;
	public void resetEncoder() {
		climberMotor1.setEncPosition(0);
	}
	public int getEncVelocity() {
		return climberMotor1.getEncVelocity();
	}
	public int getEncPosition() {
		return climberMotor1.getEncPosition();
	}
	public double getCurrent() {
		return climberMotor1.getOutputCurrent();
	}
	public void turnOn() {
		climberMotor1.set(CLIMB_DIRECTION * CLIMB_POWER);
		climberMotor2.set(CLIMB_DIRECTION * CLIMB_POWER);
	}
	public void turnOff() {
		climberMotor1.set(0);
		climberMotor2.set(0);
	}
	public void initDefaultCommand() {
		setDefaultCommand(new StopClimb());
	}
}