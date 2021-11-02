// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Rotate extends CommandBase {
    private DriveSubsystem drive;

    /** Creates a new Rotate. */
    public Rotate(final DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.startMotor(Math.copySign(drive.getSpeed(), drive.getDifference()));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getDifference()) < drive.getTolorance();
    }
}
