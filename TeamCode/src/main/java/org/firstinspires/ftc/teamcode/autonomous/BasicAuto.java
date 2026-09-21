package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BasicAuto", group = "Linear Opmode")
public class BasicAuto extends LinearOpMode {

    // Drive motors
    private DcMotor fr_motor;
    private DcMotor fl_motor;
    private DcMotor br_motor;
    private DcMotor bl_motor;

    // Mechanisms
    private DcMotor shooter;
    private DcMotor intake;
    private DcMotor transferMotor;

    // Pinpoint Odometry
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // =========================
        // HARDWARE
        // =========================

        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");

        // Pinpoint
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);

        // =========================
        // PINPOINT SETUP
        // =========================

        pinpoint.setOffsets(0,0,DistanceUnit.MM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        // =========================
        // INITIAL TELEMETRY
        // =========================

        telemetry.addLine("Pinpoint initialized.");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // =========================
            // AUTONOMOUS
            // =========================

            driveForward(0.5, 2500);

            sleep(1000);

            turnLeft(0.2, 1000);

            driveBackward(0.2, 2500);

            shootMotors();

            sleep(2000);

            stopShootMotors();

            sleep(1000);

            turnLeft(0.2, 1000);

            sleep(1000);

            startIntake();

            driveForward(0.5, 1000);

            stopIntake();
        }
    }

    // =========================================================
    // DRIVE FORWARD
    // =========================================================

    private void driveForward(double power, long timeMs) {

        fr_motor.setPower(power);
        fl_motor.setPower(power);
        br_motor.setPower(power);
        bl_motor.setPower(power);

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - startTime < timeMs) {

            updateOdometry();

            telemetry.update();
        }

        stopMotors();
    }

    // =========================================================
    // DRIVE BACKWARD
    // =========================================================

    private void driveBackward(double power, long timeMs) {

        fr_motor.setPower(-power);
        fl_motor.setPower(-power);
        br_motor.setPower(-power);
        bl_motor.setPower(-power);

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - startTime < timeMs) {

            updateOdometry();

            telemetry.update();
        }

        stopMotors();
    }

    // =========================================================
    // TURN RIGHT
    // =========================================================

    private void turnRight(double power, long timeMs) {

        fr_motor.setPower(-power);
        br_motor.setPower(-power);

        fl_motor.setPower(power);
        bl_motor.setPower(power);

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - startTime < timeMs) {

            updateOdometry();

            telemetry.update();
        }

        stopMotors();
    }

    // =========================================================
    // TURN LEFT
    // =========================================================

    private void turnLeft(double power, long timeMs) {

        fr_motor.setPower(power);
        br_motor.setPower(power);

        fl_motor.setPower(-power);
        bl_motor.setPower(-power);

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - startTime < timeMs) {

            updateOdometry();

            telemetry.update();
        }

        stopMotors();
    }

    // =========================================================
    // UPDATE ODOMETRY
    // =========================================================

    private void updateOdometry() {

        pinpoint.update();

        telemetry.addData(
                "X",
                "%.2f",
                pinpoint.getPosX(DistanceUnit.MM)
        );

        telemetry.addData(
                "Y",
                "%.2f",
                pinpoint.getPosY(DistanceUnit.MM)
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                pinpoint.getHeading(DistanceUnit.MM)
        );
    }

    // =========================================================
    // STOP DRIVE MOTORS
    // =========================================================

    private void stopMotors() {

        fr_motor.setPower(0);
        fl_motor.setPower(0);
        br_motor.setPower(0);
        bl_motor.setPower(0);
    }

    // =========================================================
    // SHOOTER
    // =========================================================

    private void shootMotors() {

        shooter.setPower(1.0);
    }

    private void stopShootMotors() {

        shooter.setPower(0.0);
    }

    // =========================================================
    // INTAKE
    // =========================================================

    private void startIntake() {

        intake.setPower(1.0);
        transferMotor.setPower(1.0);
    }

    private void stopIntake() {

        intake.setPower(0.0);
        transferMotor.setPower(0.0);
    }
}