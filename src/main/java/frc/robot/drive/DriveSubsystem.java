// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(DriveConstants.MOTOR_ID, MotorType.kBrushless);

    private NetworkTableEntry targetEntry;
    private NetworkTableEntry tolaranceEntry;
    private NetworkTableEntry speedEntry;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        var tab = Shuffleboard.getTab("Debug");

        tab.addNumber("Current Ticks", this::getCurrent);
        tab.addNumber("Current Degrees", () -> getCurrent() % 1 * 360);

        targetEntry = tab.add("Target Degrees", 0).getEntry();
        tab.addNumber("Target Ticks", this::getTarget);

        tolaranceEntry = tab.add("Tolarance Degrees", 0).getEntry();
        speedEntry = tab.add("Speed", 0).getEntry();

        motor.getEncoder().setPosition(0);
    }

    public double getTarget() {
        return targetEntry.getDouble(0) / 360 % 1;
    }

    public double getCurrent() {
        return motor.getEncoder().getPosition() % 1;
    }

    public double getDifference() {
        return getTarget() - getCurrent();
    }

    public double getTolorance() {
        return tolaranceEntry.getDouble(0) / 360 % 1;
    }

    public double getSpeed() {
        return speedEntry.getDouble(0);
    }

    public void startMotor(final double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
        motor.set(0);
    }
}
