package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Utilities.RobotHardware;

@Autonomous(name = "BasicAuto", group = "Linear Opmode")
public class BasicAuto extends LinearOpMode {

    // =========================================================
    // ROBOT HARDWARE
    // =========================================================

    private RobotHardware robot = new RobotHardware();

    // =========================================================
    // RUN OPMODE
    // =========================================================

    @Override
    public void runOpMode() {

        // Initialize ALL robot hardware
        robot.init(hardwareMap);

        // Show hardware status
        robot.logHardwareStatus(telemetry);

        telemetry.addLine("");
        telemetry.addLine("Pinpoint initialized.");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        // =====================================================
        // AUTONOMOUS
        // =====================================================

        driveForward(
                500,
                0.5
        );

        sleep(1000);

        turnLeft(
                90,
                0.2
        );

        driveBackward(
                500,
                0.2
        );

        // Shoot
        shootMotors();

        sleep(2000);

        stopShootMotors();

        sleep(1000);

        turnLeft(
                90,
                0.2
        );

        sleep(1000);

        // Intake
        startIntake();

        driveForward(
                250,
                0.5
        );

        stopIntake();

        robot.stopAllMotors();
    }

    // =========================================================
    // DRIVE FORWARD
    // =========================================================

    private void driveForward(
            double distanceMm,
            double power
    ) {

        robot.updateOdo();

        double startX =
                robot.getOdoPositionX(DistanceUnit.MM);

        double startY =
                robot.getOdoPositionY(DistanceUnit.MM);

        double startHeading =
                robot.getOdoHeading(AngleUnit.RADIANS);

        double targetX =
                startX +
                        distanceMm * Math.cos(startHeading);

        double targetY =
                startY +
                        distanceMm * Math.sin(startHeading);

        robot.setDrivePower(
                power,
                power,
                power,
                power
        );

        long startTime =
                System.currentTimeMillis();

        while (opModeIsActive()) {

            robot.updateOdo();

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

            updateTelemetry();

            telemetry.addData(
                    "Target Distance",
                    "%.1f mm",
                    distanceMm
            );

            telemetry.addData(
                    "Remaining",
                    "%.1f mm",
                    remainingDistance
            );

            telemetry.update();

            if (remainingDistance <= 10) {
                break;
            }

            // Safety timeout
            if (System.currentTimeMillis() - startTime > 10000) {
                telemetry.addLine("DRIVE TIMEOUT");
                telemetry.update();
                break;
            }
        }

        robot.stopDrive();
    }

    // =========================================================
    // DRIVE BACKWARD
    // =========================================================

    private void driveBackward(
            double distanceMm,
            double power
    ) {

        robot.updateOdo();

        double startX =
                robot.getOdoPositionX(DistanceUnit.MM);

        double startY =
                robot.getOdoPositionY(DistanceUnit.MM);

        double startHeading =
                robot.getOdoHeading(AngleUnit.RADIANS);

        double targetX =
                startX -
                        distanceMm * Math.cos(startHeading);

        double targetY =
                startY -
                        distanceMm * Math.sin(startHeading);

        robot.setDrivePower(
                -power,
                -power,
                -power,
                -power
        );

        long startTime =
                System.currentTimeMillis();

        while (opModeIsActive()) {

            robot.updateOdo();

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

            updateTelemetry();

            telemetry.addData(
                    "Target Distance",
                    "%.1f mm",
                    distanceMm
            );

            telemetry.addData(
                    "Remaining",
                    "%.1f mm",
                    remainingDistance
            );

            telemetry.update();

            if (remainingDistance <= 10) {
                break;
            }

            if (System.currentTimeMillis() - startTime > 10000) {
                telemetry.addLine("DRIVE TIMEOUT");
                telemetry.update();
                break;
            }
        }

        robot.stopDrive();
    }

    // =========================================================
    // TURN LEFT
    // =========================================================

    private void turnLeft(
            double degrees,
            double power
    ) {

        double startHeading =
                robot.getOdoHeading(
                        AngleUnit.DEGREES
                );

        double targetHeading =
                normalizeDegrees(
                        startHeading + degrees
                );

        robot.setDrivePower(
                -power,
                power,
                -power,
                power
        );

        long startTime =
                System.currentTimeMillis();

        while (opModeIsActive()) {

            double currentHeading =
                    robot.getOdoHeading(
                            AngleUnit.DEGREES
                    );

            double error =
                    angleDifference(
                            targetHeading,
                            currentHeading
                    );

            updateTelemetry();

            telemetry.addData(
                    "Target Heading",
                    "%.1f°",
                    targetHeading
            );

            telemetry.addData(
                    "Heading Error",
                    "%.1f°",
                    error
            );

            telemetry.update();

            if (Math.abs(error) <= 2) {
                break;
            }

            if (System.currentTimeMillis() - startTime > 5000) {
                telemetry.addLine("TURN TIMEOUT");
                telemetry.update();
                break;
            }
        }

        robot.stopDrive();
    }

    // =========================================================
    // TURN RIGHT
    // =========================================================

    private void turnRight(
            double degrees,
            double power
    ) {

        double startHeading =
                robot.getOdoHeading(
                        AngleUnit.DEGREES
                );

        double targetHeading =
                normalizeDegrees(
                        startHeading - degrees
                );

        robot.setDrivePower(
                power,
                -power,
                power,
                -power
        );

        long startTime =
                System.currentTimeMillis();

        while (opModeIsActive()) {

            double currentHeading =
                    robot.getOdoHeading(
                            AngleUnit.DEGREES
                    );

            double error =
                    angleDifference(
                            targetHeading,
                            currentHeading
                    );

            updateTelemetry();

            telemetry.addData(
                    "Target Heading",
                    "%.1f°",
                    targetHeading
            );

            telemetry.addData(
                    "Heading Error",
                    "%.1f°",
                    error
            );

            telemetry.update();

            if (Math.abs(error) <= 2) {
                break;
            }

            if (System.currentTimeMillis() - startTime > 5000) {
                telemetry.addLine("TURN TIMEOUT");
                telemetry.update();
                break;
            }
        }

        robot.stopDrive();
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
    // TELEMETRY
    // =========================================================

    private void updateTelemetry() {

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