package org.firstinspires.ftc.teamcode.command.tests.z_system_specific_tests.RoadRunner.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner;

/*
 * This is a simple routine to test turning capabilities.
 */

@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        Drive_Mecanum_RoadRunner drive = new Drive_Mecanum_RoadRunner(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
