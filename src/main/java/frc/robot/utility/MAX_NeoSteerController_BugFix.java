// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.team1891.common.drivetrains.swervemodules.MAX_NeoSteerController;
import frc.team1891.common.hardware.WPI_CANSparkMax;

/** In the sim, MAX_NeoSteerController seems to output random measurements for the rotation of the module.  This fixes that by always returning 0 in the sim. */
public class MAX_NeoSteerController_BugFix extends MAX_NeoSteerController {


    // TODO: getPosition instead of getAbsolutePosition

    public MAX_NeoSteerController_BugFix(int turningCANId, double chassisAngularOffset, double kP, double kI, double kD, double kFF) {
        super(turningCANId, chassisAngularOffset, kP, kI, kD, kFF);
    }

    public MAX_NeoSteerController_BugFix(WPI_CANSparkMax turningNeo, double chassisAngularOffset, double kP, double kI, double kD, double kFF) {
        super(turningNeo, chassisAngularOffset, kP, kI, kD, kFF);
    }

    @Override
    public Rotation2d getRotation2d() {
        if (Robot.isReal()) {
            return super.getRotation2d();
        }
        return new Rotation2d();
    }

    @Override
    public double getRadians() {
        if (Robot.isReal()) {
            return super.getRadians();
        }
        return 0;
    }

    @Override
    public double getDegrees() {
        if (Robot.isReal()) {
            return super.getDegrees();
        }
        return 0;
    }
}
