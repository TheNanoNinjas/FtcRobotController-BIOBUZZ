package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utilities.RobotHardware;

@Autonomous(
        name = "Basic Limelight Align",
        group = "Test"
)
public class BasicLimelightAlign extends OpMode {

    // =========================================================
    // ROBOT
    // =========================================================

    private RobotHardware robot = new RobotHardware();

    // =========================================================
    // TARGET
    // =========================================================

    private static final int TARGET_ID = 36;

    // =========================================================
    // ALIGNMENT
    // =========================================================

    /*
     * How close tx needs to be to zero.
     *
     * tx = 0 means the AprilTag is centered
     * in the Limelight's horizontal view.
     */
    private static final double TX_TOLERANCE = 1.0;

    /*
     * Proportional steering gain.
     */
    private static final double ALIGN_KP = 0.015;

    /*
     * Maximum turning power.
     */
    private static final double MAX_TURN_POWER = 0.30;

    /*
     * Minimum turning power.
     */
    private static final double MIN_TURN_POWER = 0.08;

    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {

        robot.init(hardwareMap);

        robot.startLimelight();

        robot.stopDrive();

        telemetry.addLine(
                "Basic Limelight Align"
        );

        telemetry.addData(
                "Target ID",
                TARGET_ID
        );

        telemetry.addLine(
                "Waiting for start..."
        );

        telemetry.update();
    }

    // =========================================================
    // START
    // =========================================================

    @Override
    public void start() {

        robot.stopDrive();
    }

    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {

        LLResult result =
                robot.limelight.getLatestResult();

        // =====================================================
        // NO RESULT
        // =====================================================

        if (result == null) {

            robot.stopDrive();

            telemetry.addLine(
                    "No Limelight result"
            );

            telemetry.update();

            return;
        }

        // =====================================================
        // FIND TAG 36
        // =====================================================

        LLResultTypes.FiducialResult target =
                findTargetTag(
                        result,
                        TARGET_ID
                );

        // =====================================================
        // TAG NOT FOUND
        // =====================================================

        if (target == null) {

            robot.stopDrive();

            telemetry.addLine(
                    "Searching for AprilTag 36..."
            );

            telemetry.update();

            return;
        }

        // =====================================================
        // TAG FOUND
        // =====================================================

        double tx =
                target.getTargetXDegrees();

        double ty =
                target.getTargetYDegrees();

        double ta =
                target.getTargetArea();

        telemetry.addLine(
                "AprilTag 36 DETECTED"
        );

        telemetry.addData(
                "Tag ID",
                TARGET_ID
        );

        telemetry.addData(
                "TX",
                "%.2f",
                tx
        );

        telemetry.addData(
                "TY",
                "%.2f",
                ty
        );

        telemetry.addData(
                "Area",
                "%.2f",
                ta
        );

        // =====================================================
        // ALREADY ALIGNED
        // =====================================================

        if (Math.abs(tx)
                <= TX_TOLERANCE) {

            robot.stopDrive();

            telemetry.addLine(
                    "ALIGNED TO HIVE"
            );

            telemetry.update();

            return;
        }

        // =====================================================
        // CALCULATE TURN
        // =====================================================

        double turnPower =
                tx * ALIGN_KP;

        // Limit maximum power
        turnPower =
                clip(
                        turnPower,
                        -MAX_TURN_POWER,
                        MAX_TURN_POWER
                );

        // Minimum useful power
        if (Math.abs(turnPower)
                < MIN_TURN_POWER) {

            turnPower =
                    Math.copySign(
                            MIN_TURN_POWER,
                            turnPower
                    );
        }

        // =====================================================
        // TURN
        // =====================================================

        robot.setDrivePower(
                turnPower,
                -turnPower,
                turnPower,
                -turnPower
        );

        telemetry.addData(
                "Turn Power",
                "%.3f",
                turnPower
        );

        telemetry.update();
    }

    // =========================================================
    // FIND TARGET TAG
    // =========================================================

    private LLResultTypes.FiducialResult findTargetTag(
            LLResult result,
            int wantedID
    ) {

        if (!result.isValid()) {

            return null;
        }

        for (
                LLResultTypes.FiducialResult fiducial
                : result.getFiducialResults()
        ) {

            if (
                    fiducial.getFiducialId()
                            == wantedID
            ) {

                return fiducial;
            }
        }

        return null;
    }

    // =========================================================
    // CLIP
    // =========================================================

    private double clip(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(
                        max,
                        value
                )
        );
    }

    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {

        robot.stopDrive();

        robot.stopLimelight();
    }
}