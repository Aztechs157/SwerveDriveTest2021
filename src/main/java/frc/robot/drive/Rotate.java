// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Rotate extends CommandBase {
    private DriveSubsystem drive;

    /** Creates a new Rotate. */
    public Rotate(final DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
        Shuffleboard.getTab("Debug").addNumber("Difference", () -> getDifference() * 360);
    }

    public double getDifference() {
        // TARGET: 90, 355, 97
        // CURRENT: 0, 0, 355
        // DELTA: 90, -5 (355), 102 (-258)
        var difference = drive.getTarget() - drive.getCurrent();

        if (difference > 0.5) {
            return difference - 1;
        }

        if (difference < -0.5) {
            return difference + 1;
        }

        return difference;
    }

    @Override
    public void initialize() {
        if (getDifference() > 0) {
            drive.startMotorPositive();
        } else {
            drive.startMotorNegative();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getDifference()) < drive.getTolorance();
    }
}
