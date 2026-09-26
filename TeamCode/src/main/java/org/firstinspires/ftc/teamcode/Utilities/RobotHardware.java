```java
package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotHardware {

    // =========================================================
    // DRIVE MOTORS
    // =========================================================

    public DcMotor fl_motor;
    public DcMotor fr_motor;
    public DcMotor bl_motor;
    public DcMotor br_motor;

    // =========================================================
    // MECHANISM MOTORS
    // =========================================================

    public DcMotorEx shooter;
    public DcMotor intakeMotor;

    // =========================================================
    // SERVO MECHANISMS
    // =========================================================

    public Servo Stage1Servo;
    public Servo Stage2Servo;
    public Servo Stage3Servo;
    public Servo Stage4Servo;

    // =========================================================
    // SENSORS
    // =========================================================

    private GoBildaPinpointDriver odo;

    public Rev2mDistanceSensor distance_sensor;

    public IMU imu;

    // =========================================================
    // LIMELIGHT
    // =========================================================

    public Limelight3A limelight;

    // =========================================================
    // INITIALIZE HARDWARE
    // =========================================================

    public void init(HardwareMap hardwareMap) {

        // =====================================================
        // DRIVE MOTORS
        // =====================================================

        fl_motor =
                hardwareMap.get(
                        DcMotor.class,
                        "fl_motor"
                );

        fr_motor =
                hardwareMap.get(
                        DcMotor.class,
                        "fr_motor"
                );

        bl_motor =
                hardwareMap.get(
                        DcMotor.class,
                        "bl_motor"
                );

        br_motor =
                hardwareMap.get(
                        DcMotor.class,
                        "br_motor"
                );

        // Left motors reversed
        fl_motor.setDirection(
                DcMotor.Direction.REVERSE
        );

        bl_motor.setDirection(
                DcMotor.Direction.REVERSE
        );

        // =====================================================
        // MECHANISM MOTORS
        // =====================================================

        shooter =
                hardwareMap.get(
                        DcMotorEx.class,
                        "leftShooter"
                );

        intakeMotor =
                hardwareMap.get(
                        DcMotor.class,
                        "intakeMotor"
                );

        // Shooter direction
        shooter.setDirection(
                DcMotor.Direction.REVERSE
        );

        // =====================================================
        // SERVOS
        // =====================================================

        Stage1Servo =
                hardwareMap.get(
                        Servo.class,
                        "Stage1Servo"
                );

        Stage2Servo =
                hardwareMap.get(
                        Servo.class,
                        "Stage2Servo"
                );

        Stage3Servo =
                hardwareMap.get(
                        Servo.class,
                        "Stage3Servo"
                );

        Stage4Servo =
                hardwareMap.get(
                        Servo.class,
                        "Stage4Servo"
                );

        // =====================================================
        // IMU
        // =====================================================

        imu =
                hardwareMap.get(
                        IMU.class,
                        "imu"
                );

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(
                new IMU.Parameters(
                        orientationOnRobot
                )
        );

        // =====================================================
        // DISTANCE SENSOR
        // =====================================================

        distance_sensor =
                hardwareMap.get(
                        Rev2mDistanceSensor.class,
                        "distance_sensor"
                );

        // =====================================================
        // PINPOINT ODOMETRY
        // =====================================================

        odo =
                hardwareMap.get(
                        GoBildaPinpointDriver.class,
                        "odo"
                );

        odo.setOffsets(
                -88,
                0.0,
                DistanceUnit.MM
        );

        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        odo.resetPosAndIMU();

        odo.setPosition(
                new Pose2D(
                        DistanceUnit.MM,
                        0,
                        0,
                        AngleUnit.DEGREES,
                        0
                )
        );

        // =====================================================
        // LIMELIGHT
        // =====================================================

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "Limelight"
                );
    }

    // =========================================================
    // ODOMETRY
    // =========================================================

    public void updateOdo() {

        odo.update();
    }

    public double getOdoPositionY(
            DistanceUnit unit
    ) {

        odo.update();

        Pose2D position =
                odo.getPosition();

        return position.getY(unit);
    }

    public double getOdoPositionX(
            DistanceUnit unit
    ) {

        odo.update();

        Pose2D position =
                odo.getPosition();

        return position.getX(unit);
    }

    public double getOdoHeading(
            AngleUnit angleUnit
    ) {

        odo.update();

        return odo
                .getPosition()
                .getHeading(angleUnit);
    }

    public Pose2D getOdoPosition() {

        odo.update();

        return odo.getPosition();
    }

    // =========================================================
    // DRIVE
    // =========================================================

    public void setDrivePower(
            double fl,
            double fr,
            double bl,
            double br
    ) {

        fl_motor.setPower(fl);
        fr_motor.setPower(fr);
        bl_motor.setPower(bl);
        br_motor.setPower(br);
    }

    public void stopDrive() {

        setDrivePower(
                0,
                0,
                0,
                0
        );
    }

    // =========================================================
    // STOP ALL MOTORS
    // =========================================================

    public void stopAllMotors() {

        stopDrive();

        shooter.setPower(0);

        intakeMotor.setPower(0);
    }

    // =========================================================
    // IMU
    // =========================================================

    public YawPitchRollAngles getOrientation() {

        return imu.getRobotYawPitchRollAngles();
    }

    // =========================================================
    // RESET ODOMETRY
    // =========================================================

    public void resetOdo() {

        odo.resetPosAndIMU();

        odo.setPosition(
                new Pose2D(
                        DistanceUnit.MM,
                        0,
                        0,
                        AngleUnit.DEGREES,
                        0
                )
        );
    }

    // =========================================================
    // HARDWARE STATUS
    // =========================================================

    public void logHardwareStatus(
            Telemetry telemetry
    ) {

        telemetry.addData(
                "FL Motor",
                fl_motor != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "FR Motor",
                fr_motor != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "BL Motor",
                bl_motor != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "BR Motor",
                br_motor != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Stage1Servo",
                Stage1Servo != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Stage2Servo",
                Stage2Servo != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Stage3Servo",
                Stage3Servo != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Stage4Servo",
                Stage4Servo != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Shooter",
                shooter != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Intake Motor",
                intakeMotor != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Odometry Wheels",
                odo != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Distance Sensor",
                distance_sensor != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addData(
                "Limelight",
                limelight != null
                        ? "OK"
                        : "FAIL"
        );

        telemetry.addLine(
                "Hardware Initialized"
        );
    }

    // =========================================================
    // PORT MAPPING
    // =========================================================

    public void displayPortMapping(
            Telemetry telemetry
    ) {

        telemetry.addLine(
                "=== PORT MAPPING ==="
        );

        if (fl_motor != null) {

            telemetry.addData(
                    "FL Motor",
                    "Port " +
                            fl_motor.getPortNumber()
            );
        }

        if (fr_motor != null) {

            telemetry.addData(
                    "FR Motor",
                    "Port " +
                            fr_motor.getPortNumber()
            );
        }

        if (bl_motor != null) {

            telemetry.addData(
                    "BL Motor",
                    "Port " +
                            bl_motor.getPortNumber()
            );
        }

        if (br_motor != null) {

            telemetry.addData(
                    "BR Motor",
                    "Port " +
                            br_motor.getPortNumber()
            );
        }

        if (Stage1Servo != null) {

            telemetry.addData(
                    "Stage1Servo",
                    "Port " +
                            Stage1Servo.getPortNumber()
            );
        }

        if (Stage2Servo != null) {

            telemetry.addData(
                    "Stage2Servo",
                    "Port " +
                            Stage2Servo.getPortNumber()
            );
        }

        if (Stage3Servo != null) {

            telemetry.addData(
                    "Stage3Servo",
                    "Port " +
                            Stage3Servo.getPortNumber()
            );
        }

        if (Stage4Servo != null) {

            telemetry.addData(
                    "Stage4Servo",
                    "Port " +
                            Stage4Servo.getPortNumber()
            );
        }

        if (shooter != null) {

            telemetry.addData(
                    "Left Shooter",
                    "Port " +
                            shooter.getPortNumber()
            );
        }

        if (intakeMotor != null) {

            telemetry.addData(
                    "Intake Motor",
                    "Port " +
                            intakeMotor.getPortNumber()
            );
        }

        if (limelight != null) {

            telemetry.addData(
                    "Limelight",
                    "Connected"
            );
        }

        telemetry.addLine(
                "==================="
        );
    }
}
