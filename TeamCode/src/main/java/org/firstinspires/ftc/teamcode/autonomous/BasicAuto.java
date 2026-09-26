```java
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.RobotHardware;

import java.util.List;

@Autonomous(name = "BasicAuto", group = "OpMode")
public class BasicAuto extends OpMode {

    // =========================================================
    // ROBOT HARDWARE
    // =========================================================

    private RobotHardware robot = new RobotHardware();

    // =========================================================
    // LIMELIGHT
    // =========================================================

    private Limelight3A limelight;

    // AprilTag we want to track
    private static final int TARGET_TAG_ID = 36;

    // Your AprilTag pipeline
    private static final int LIMELIGHT_PIPELINE = 8;

    // =========================================================
    // LIMELIGHT ALIGNMENT SETTINGS
    // =========================================================

    // How close the tag needs to be to the center of the camera
    private static final double TX_TOLERANCE = 1.5;

    // Maximum power used for Limelight correction
    private static final double MAX_ALIGNMENT_POWER = 0.25;

    // Minimum correction power
    private static final double MIN_ALIGNMENT_POWER = 0.08;

    // =========================================================
    // AUTONOMOUS STATES
    // =========================================================

    private enum AutoState {

        DRIVE_FORWARD_1,

        WAIT_1,

        TURN_LEFT_1,

        DRIVE_BACKWARD,

        SHOOT,

        WAIT_AFTER_SHOOT,

        TURN_LEFT_2,

        WAIT_2,

        INTAKE_DRIVE,

        STOP
    }

    private AutoState currentState = AutoState.DRIVE_FORWARD_1;

    // =========================================================
    // STATE VARIABLES
    // =========================================================

    private long stateStartTime;

    private double targetX;
    private double targetY;
    private double targetHeading;

    // =========================================================
    // LIMELIGHT VARIABLES
    // =========================================================

    private boolean targetVisible = false;

    private double tagTx = 0.0;
    private double tagTy = 0.0;

    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {

        // Initialize robot hardware
        robot.init(hardwareMap);

        // Initialize Limelight
        limelight = hardwareMap.get(
                Limelight3A.class,
                "Limelight"
        );

        // Select AprilTag pipeline
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        // Start Limelight
        limelight.start();

        robot.logHardwareStatus(telemetry);

        telemetry.addLine("");
        telemetry.addLine("Pinpoint initialized.");
        telemetry.addLine("Limelight initialized.");
        telemetry.addData("AprilTag Target", TARGET_TAG_ID);
        telemetry.addData("Limelight Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addLine("Ready to start.");
        telemetry.update();
    }

    // =========================================================
    // START
    // =========================================================

    @Override
    public void start() {

        currentState = AutoState.DRIVE_FORWARD_1;

        stateStartTime = System.currentTimeMillis();

        prepareDriveForward(500);

        telemetry.addLine("Autonomous Started");
        telemetry.update();
    }

    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {

        // Update odometry
        robot.updateOdo();

        // Update Limelight
        updateLimelight();

        // =====================================================
        // AUTONOMOUS STATE MACHINE
        // =====================================================

        switch (currentState) {

            case DRIVE_FORWARD_1:
                DRIVE_FORWARD_1();
                break;

            case WAIT_1:
                WAIT_1();
                break;

            case TURN_LEFT_1:
                TURN_LEFT_1();
                break;

            case DRIVE_BACKWARD:
                DRIVE_BACKWARD();
                break;

            case SHOOT:
                SHOOT();
                break;

            case WAIT_AFTER_SHOOT:
                WAIT_AFTER_SHOOT();
                break;

            case TURN_LEFT_2:
                TURN_LEFT_2();
                break;

            case WAIT_2:
                WAIT_2();
                break;

            case INTAKE_DRIVE:
                INTAKE_DRIVE();
                break;

            case STOP:
                STOP();
                break;
        }

        updateTelemetry();
    }

    // =========================================================
    // DRIVE FORWARD 1
    // =========================================================

    private void DRIVE_FORWARD_1() {

        if (driveToPosition(0.5)) {

            robot.stopDrive();

            currentState = AutoState.WAIT_1;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // WAIT 1
    // =========================================================

    private void WAIT_1() {

        robot.stopDrive();

        if (elapsedTime(1000)) {

            prepareTurnLeft(90);

            currentState = AutoState.TURN_LEFT_1;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // TURN LEFT 1
    // =========================================================

    private void TURN_LEFT_1() {

        if (turnToHeading(0.2)) {

            robot.stopDrive();

            prepareDriveBackward(500);

            currentState = AutoState.DRIVE_BACKWARD;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // DRIVE BACKWARD
    // =========================================================

    private void DRIVE_BACKWARD() {

        /*
         * Continue using odometry to get to the shooting area.
         *
         * Once the target position is reached, the robot
         * transitions into the SHOOT state.
         */

        if (driveToPosition(0.2)) {

            robot.stopDrive();

            currentState = AutoState.SHOOT;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // SHOOT
    // =========================================================

    private void SHOOT() {

        /*
         * Keep the shooter running for 6 seconds.
         */
        shootMotors();

        /*
         * If AprilTag 36 is visible, use the Limelight
         * to correct the robot's horizontal alignment.
         */
        if (targetVisible) {

            alignToAprilTag();
        }

        /*
         * Six-second shooting period.
         */
        if (elapsedTime(6000)) {

            stopShootMotors();

            robot.stopDrive();

            currentState = AutoState.WAIT_AFTER_SHOOT;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // WAIT AFTER SHOOT
    // =========================================================

    private void WAIT_AFTER_SHOOT() {

        robot.stopDrive();

        if (elapsedTime(1000)) {

            prepareTurnLeft(90);

            currentState = AutoState.TURN_LEFT_2;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // TURN LEFT 2
    // =========================================================

    private void TURN_LEFT_2() {

        if (turnToHeading(0.2)) {

            robot.stopDrive();

            currentState = AutoState.WAIT_2;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // WAIT 2
    // =========================================================

    private void WAIT_2() {

        robot.stopDrive();

        if (elapsedTime(1000)) {

            prepareDriveForward(250);

            startIntake();

            currentState = AutoState.INTAKE_DRIVE;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // INTAKE DRIVE
    // =========================================================

    private void INTAKE_DRIVE() {

        startIntake();

        if (driveToPosition(0.5)) {

            stopIntake();

            robot.stopDrive();

            currentState = AutoState.STOP;

            stateStartTime =
                    System.currentTimeMillis();
        }
    }

    // =========================================================
    // STOP
    // =========================================================

    private void STOP() {

        robot.stopAllMotors();

        if (limelight != null) {
            limelight.stop();
        }

        telemetry.addLine("AUTONOMOUS COMPLETE");
        telemetry.update();
    }

    // =========================================================
    // LIMELIGHT UPDATE
    // =========================================================

    private void updateLimelight() {

        targetVisible = false;

        LLResult result =
                limelight.getLatestResult();

        if (result == null) {
            return;
        }

        if (!result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int id =
                    fiducial.getFiducialId();

            /*
             * Only use AprilTag ID 36.
             */
            if (id == TARGET_TAG_ID) {

                targetVisible = true;

                tagTx =
                        fiducial.getTargetXDegrees();

                tagTy =
                        fiducial.getTargetYDegrees();

                break;
            }
        }
    }

    // =========================================================
    // APRILTAG ALIGNMENT
    // =========================================================

    private void alignToAprilTag() {

        /*
         * If the tag is already centered,
         * don't move the robot.
         */
        if (Math.abs(tagTx) <= TX_TOLERANCE) {

            robot.stopDrive();

            return;
        }

        /*
         * Convert TX into a proportional correction.
         *
         * Positive TX:
         * Tag is to the right.
         *
         * Negative TX:
         * Tag is to the left.
         */

        double correction =
                tagTx * 0.025;

        /*
         * Limit maximum correction.
         */

        correction =
                Math.max(
                        -MAX_ALIGNMENT_POWER,
                        Math.min(
                                MAX_ALIGNMENT_POWER,
                                correction
                        )
                );

        /*
         * Make sure very small corrections
         * still move the robot.
         */

        if (Math.abs(correction)
                < MIN_ALIGNMENT_POWER) {

            correction =
                    Math.copySign(
                            MIN_ALIGNMENT_POWER,
                            correction
                    );
        }

        /*
         * Strafe toward the AprilTag.
         *
         * Because your left motors are reversed
         * in RobotHardware, these powers are
         * intentionally written for your existing
         * motor configuration.
         */

        if (tagTx > 0) {

            // Tag is right -> strafe right

            robot.setDrivePower(
                    correction,
                    -correction,
                    -correction,
                    correction
            );

        } else {

            // Tag is left -> strafe left

            robot.setDrivePower(
                    -correction,
                    correction,
                    correction,
                    -correction
            );
        }
    }

    // =========================================================
    // PREPARE FORWARD MOVEMENT
    // =========================================================

    private void prepareDriveForward(double distanceMm) {

        robot.updateOdo();

        double startX =
                robot.getOdoPositionX(
                        DistanceUnit.MM
                );

        double startY =
                robot.getOdoPositionY(
                        DistanceUnit.MM
                );

        double heading =
                robot.getOdoHeading(
                        AngleUnit.RADIANS
                );

        targetX =
                startX
                        + distanceMm
                        * Math.cos(heading);

        targetY =
                startY
                        + distanceMm
                        * Math.sin(heading);
    }

    // =========================================================
    // PREPARE BACKWARD MOVEMENT
    // =========================================================

    private void prepareDriveBackward(double distanceMm) {

        robot.updateOdo();

        double startX =
                robot.getOdoPositionX(
                        DistanceUnit.MM
                );

        double startY =
                robot.getOdoPositionY(
                        DistanceUnit.MM
                );

        double heading =
                robot.getOdoHeading(
                        AngleUnit.RADIANS
                );

        targetX =
                startX
                        - distanceMm
                        * Math.cos(heading);

        targetY =
                startY
                        - distanceMm
                        * Math.sin(heading);
    }

    // =========================================================
    // DRIVE TO TARGET
    // =========================================================

    private boolean driveToPosition(double power) {

        double currentX =
                robot.getOdoPositionX(
                        DistanceUnit.MM
                );

        double currentY =
                robot.getOdoPositionY(
                        DistanceUnit.MM
                );

        double remainingDistance =
                Math.hypot(
                        targetX - currentX,
                        targetY - currentY
                );

        telemetry.addData(
                "Target Distance",
                "%.1f mm",
                remainingDistance
        );

        /*
         * Position reached.
         */

        if (remainingDistance <= 10) {

            robot.stopDrive();

            return true;
        }

        /*
         * Safety timeout.
         */

        if (System.currentTimeMillis()
                - stateStartTime > 10000) {

            robot.stopDrive();

            telemetry.addLine(
                    "DRIVE TIMEOUT"
            );

            return true;
        }

        /*
         * Forward movement.
         */

        robot.setDrivePower(
                power,
                power,
                power,
                power
        );

        return false;
    }

    // =========================================================
    // PREPARE TURN LEFT
    // =========================================================

    private void prepareTurnLeft(double degrees) {

        robot.updateOdo();

        double startHeading =
                robot.getOdoHeading(
                        AngleUnit.DEGREES
                );

        targetHeading =
                normalizeDegrees(
                        startHeading + degrees
                );
    }

    // =========================================================
    // TURN TO TARGET HEADING
    // =========================================================

    private boolean turnToHeading(double power) {

        double currentHeading =
                robot.getOdoHeading(
                        AngleUnit.DEGREES
                );

        double error =
                angleDifference(
                        targetHeading,
                        currentHeading
                );

        telemetry.addData(
                "Target Heading",
                "%.1f°",
                targetHeading
        );

        telemetry.addData(
                "Current Heading",
                "%.1f°",
                currentHeading
        );

        telemetry.addData(
                "Heading Error",
                "%.1f°",
                error
        );

        /*
         * Heading reached.
         */

        if (Math.abs(error) <= 2) {

            robot.stopDrive();

            return true;
        }

        /*
         * Safety timeout.
         */

        if (System.currentTimeMillis()
                - stateStartTime > 5000) {

            robot.stopDrive();

            telemetry.addLine(
                    "TURN TIMEOUT"
            );

            return true;
        }

        /*
         * Turn left.
         */

        if (error > 0) {

            robot.setDrivePower(
                    -power,
                    power,
                    -power,
                    power
            );

        } else {

            /*
             * Turn right.
             */

            robot.setDrivePower(
                    power,
                    -power,
                    power,
                    -power
            );
        }

        return false;
    }

    // =========================================================
    // SHOOTER
    // =========================================================

    private void shootMotors() {

        robot.shooter.setPower(1.0);
    }

    private void stopShootMotors() {

        robot.shooter.setPower(0.0);
    }

    // =========================================================
    // INTAKE
    // =========================================================

    private void startIntake() {

        robot.intakeMotor.setPower(1.0);
    }

    private void stopIntake() {

        robot.intakeMotor.setPower(0.0);
    }

    // =========================================================
    // TIMER
    // =========================================================

    private boolean elapsedTime(long milliseconds) {

        return System.currentTimeMillis()
                - stateStartTime
                >= milliseconds;
    }

    // =========================================================
    // TELEMETRY
    // =========================================================

    private void updateTelemetry() {

        telemetry.addData(
                "State",
                currentState
        );

        telemetry.addData(
                "X",
                "%.2f mm",
                robot.getOdoPositionX(
                        DistanceUnit.MM
                )
        );

        telemetry.addData(
                "Y",
                "%.2f mm",
                robot.getOdoPositionY(
                        DistanceUnit.MM
                )
        );

        telemetry.addData(
                "Heading",
                "%.2f°",
                robot.getOdoHeading(
                        AngleUnit.DEGREES
                )
        );

        telemetry.addData(
                "AprilTag Target",
                TARGET_TAG_ID
        );

        telemetry.addData(
                "AprilTag Visible",
                targetVisible
        );

        if (targetVisible) {

            telemetry.addData(
                    "Tag TX",
                    "%.2f°",
                    tagTx
            );

            telemetry.addData(
                    "Tag TY",
                    "%.2f°",
                    tagTy
            );
        }

        telemetry.update();
    }

    // =========================================================
    // ANGLE DIFFERENCE
    // =========================================================

    private double angleDifference(
            double target,
            double current
    ) {

        double difference =
                target - current;

        while (difference > 180) {

            difference -= 360;
        }

        while (difference < -180) {

            difference += 360;
        }

        return difference;
    }

    // =========================================================
    // NORMALIZE ANGLE
    // =========================================================

    private double normalizeDegrees(
            double angle
    ) {

        while (angle >= 360) {

            angle -= 360;
        }

        while (angle < 0) {

            angle += 360;
        }

        return angle;
    }
}
