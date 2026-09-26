package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Utilities.RobotHardware;

@Autonomous(name = "BasicAuto", group = "OpMode")
public class BasicAuto extends OpMode {

    // =========================================================
    // ROBOT HARDWARE
    // =========================================================

    private RobotHardware robot = new RobotHardware();

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
    // INIT
    // =========================================================

    @Override
    public void init() {

        // Initialize robot hardware
        robot.init(hardwareMap);

        // Show hardware status
        robot.logHardwareStatus(telemetry);

        telemetry.addLine("");
        telemetry.addLine("Pinpoint initialized.");
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

        robot.updateOdo();

        switch (currentState) {

            // =====================================================
            // DRIVE FORWARD 500mm
            // =====================================================

            case DRIVE_FORWARD_1:

                if (driveToPosition(0.5)) {

                    robot.stopDrive();

                    currentState = AutoState.WAIT_1;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // WAIT 1 SECOND
            // =====================================================

            case WAIT_1:

                robot.stopDrive();

                if (elapsedTime(1000)) {

                    prepareTurnLeft(90);

                    currentState = AutoState.TURN_LEFT_1;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // TURN LEFT 90°
            // =====================================================

            case TURN_LEFT_1:

                if (turnToHeading(0.2)) {

                    robot.stopDrive();

                    prepareDriveBackward(500);

                    currentState = AutoState.DRIVE_BACKWARD;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // DRIVE BACKWARD 500mm
            // =====================================================

            case DRIVE_BACKWARD:

                if (driveToPosition(0.2)) {

                    robot.stopDrive();

                    shootMotors();

                    currentState = AutoState.SHOOT;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // SHOOT FOR 6 SECONDS
            // =====================================================

            case SHOOT:

                shootMotors();

                if (elapsedTime(6000)) {

                    stopShootMotors();

                    currentState = AutoState.WAIT_AFTER_SHOOT;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // WAIT 1 SECOND AFTER SHOOTING
            // =====================================================

            case WAIT_AFTER_SHOOT:

                robot.stopDrive();

                if (elapsedTime(1000)) {

                    prepareTurnLeft(90);

                    currentState = AutoState.TURN_LEFT_2;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // TURN LEFT ANOTHER 90°
            // =====================================================

            case TURN_LEFT_2:

                if (turnToHeading(0.2)) {

                    robot.stopDrive();

                    currentState = AutoState.WAIT_2;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // WAIT 1 SECOND
            // =====================================================

            case WAIT_2:

                robot.stopDrive();

                if (elapsedTime(1000)) {

                    prepareDriveForward(250);

                    startIntake();

                    currentState = AutoState.INTAKE_DRIVE;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // INTAKE + DRIVE FORWARD 250mm
            // =====================================================

            case INTAKE_DRIVE:

                startIntake();

                if (driveToPosition(0.5)) {

                    stopIntake();
                    robot.stopDrive();

                    currentState = AutoState.STOP;
                    stateStartTime = System.currentTimeMillis();
                }

                break;

            // =====================================================
            // STOP
            // =====================================================

            case STOP:

                robot.stopAllMotors();

                telemetry.addLine("AUTONOMOUS COMPLETE");
                telemetry.update();

                break;
        }

        updateTelemetry();
    }

    // =========================================================
    // PREPARE FORWARD MOVEMENT
    // =========================================================

    private void prepareDriveForward(double distanceMm) {

        robot.updateOdo();

        double startX =
                robot.getOdoPositionX(DistanceUnit.MM);

        double startY =
                robot.getOdoPositionY(DistanceUnit.MM);

        double heading =
                robot.getOdoHeading(AngleUnit.RADIANS);

        targetX =
                startX +
                distanceMm * Math.cos(heading);

        targetY =
                startY +
                distanceMm * Math.sin(heading);
    }

    // =========================================================
    // PREPARE BACKWARD MOVEMENT
    // =========================================================

    private void prepareDriveBackward(double distanceMm) {

        robot.updateOdo();

        double startX =
                robot.getOdoPositionX(DistanceUnit.MM);

        double startY =
                robot.getOdoPositionY(DistanceUnit.MM);

        double heading =
                robot.getOdoHeading(AngleUnit.RADIANS);

        targetX =
                startX -
                distanceMm * Math.cos(heading);

        targetY =
                startY -
                distanceMm * Math.sin(heading);
    }

    // =========================================================
    // DRIVE TO TARGET
    // =========================================================

    private boolean driveToPosition(double power) {

        double currentX =
                robot.getOdoPositionX(DistanceUnit.MM);

        double currentY =
                robot.getOdoPositionY(DistanceUnit.MM);

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

        if (remainingDistance <= 10) {

            robot.stopDrive();

            return true;
        }

        // Safety timeout
        if (System.currentTimeMillis() - stateStartTime > 10000) {

            robot.stopDrive();

            telemetry.addLine("DRIVE TIMEOUT");

            return true;
        }

        // Drive forward
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

        // Target reached
        if (Math.abs(error) <= 2) {

            robot.stopDrive();

            return true;
        }

        // Safety timeout
        if (System.currentTimeMillis() - stateStartTime > 5000) {

            robot.stopDrive();

            telemetry.addLine("TURN TIMEOUT");

            return true;
        }

        // Turn left
        if (error > 0) {

            robot.setDrivePower(
                    -power,
                    power,
                    -power,
                    power
            );

        } else {

            // Turn right if error is negative
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

        return System.currentTimeMillis() - stateStartTime
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

        telemetry.update();
    }

    // =========================================================
    // ANGLE DIFFERENCE
    // =========================================================

    private double angleDifference(
            double target,
            double current
    ) {

        double difference = target - current;

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
