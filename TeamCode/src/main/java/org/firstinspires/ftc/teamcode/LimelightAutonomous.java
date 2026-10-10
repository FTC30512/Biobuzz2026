package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class LimelightAutonomous extends LinearOpMode {

    private boolean targetRedCell;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Initialized. Target: " + (targetRedCell ? "RedCell (Pipeline 2)" : "BlueCell (Pipeline 1)"));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // ==========================================
            // STEP 1: Run code using Pipeline 0
            // ==========================================
            limelight.pipelineSwitch(0);
            telemetry.addData("Status", "Running Pipeline 0 (Initial Detection)");
            telemetry.update();

            // Drive forward slightly or run initial routine
            setDrivePower(0.3, 0.3, 0.3, 0.3);
            long startTime = System.currentTimeMillis();

            while (opModeIsActive() && (System.currentTimeMillis() - startTime < 1500)) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    // Pipeline 0 data handling if needed
                }
                sleep(20);
            }

            setDrivePower(0, 0, 0, 0);
            sleep(500);

            // ==========================================
            // STEP 2: Switch to Pipeline 2 (RedCell) or Pipeline 1 (BlueCell) & Align
            // ==========================================
            int targetPipeline = targetRedCell ? 2 : 1;
            limelight.pipelineSwitch(targetPipeline);

            telemetry.addData("Status", "Switched to Pipeline " + targetPipeline + " (" + (targetRedCell ? "RedCell" : "BlueCell") + ")");
            telemetry.update();

            sleep(300); // Allow camera to stabilize

            boolean targetFound = false;

            // Search for target and align000000000000000000000000000000000000000000000000000000000000000..

            while (opModeIsActive() && !targetFound) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

                LLResult llResult = limelight.getLatestResult();

                if (llResult != null && llResult.isValid()) {
                    double tx = llResult.getTx();

                    telemetry.addData("Status", "Target Detected on Pipeline " + targetPipeline);
                    telemetry.addData("Tx Offset", tx);
                    telemetry.update();

                    if (Math.abs(tx) < 1.0) {
                        setDrivePower(0, 0, 0, 0);
                        targetFound = true; // Exits alignment loop
                    } else {
                        double kp = 0.02;
                        double power = Math.min(Math.max(tx * kp, -0.4), 0.4);
                        setDrivePower(-power, -power, power, power);
                    }
                } else {
                    telemetry.addData("Status", "Searching for Target on Pipeline " + targetPipeline + "...");
                    telemetry.update();
                    setDrivePower(-0.2, -0.2, 0.2, 0.2); // Scan slowly
                }
                sleep(20);
            }

            // ==========================================
            // STEP 3: Hold Position (No Shooting)
            // ==========================================
            setDrivePower(0, 0, 0, 0);

            telemetry.addData("Status", "Target Aligned! Holding Position.");
            telemetry.update();

            // Keep the OpMode running safely until autonomous ends
            while (opModeIsActive()) {
                sleep(50);
            }
        }
    }



    private void initHardware() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

    }
}