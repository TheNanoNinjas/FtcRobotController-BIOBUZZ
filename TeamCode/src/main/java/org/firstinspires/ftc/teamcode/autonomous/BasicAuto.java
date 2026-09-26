package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilities.RobotHardware;

@Autonomous(
        name = "BasicAuto",
        group = "Competition"
)
public class BasicAuto extends OpMode {

    // =========================================================
    // ROBOT
    // =========================================================

    private RobotHardware robot = new RobotHardware();

    // =========================================================
    // STAGES
    // =========================================================

    private enum AutoStage {

        DRIVE_FORWARD_1,
        TURN_LEFT_1,
        DRIVE_BACKWARD_1,

        ALIGN_TO_HIVE,

        SHOOT,

        TURN_RIGHT_1,

        START_INTAKE,

        DRIVE_BACKWARD_2,

        STOP
    }

    private AutoStage currentStage = AutoStage.DRIVE_FORWARD_1;

    // =========================================================
    // TIMERS
    // =========================================================

    private ElapsedTime stageTimer = new ElapsedTime();

    // =========================================================
    // DISTANCES
    // =========================================================

    private static final double FORWARD_DISTANCE_1 = 60.0;

    private static final double BACKWARD_DISTANCE_1 = 36.0;

    private static final double BACKWARD_DISTANCE_2 = 60.0;

    // =========================================================
    // TURN ANGLES
    // =========================================================

    private static final double TURN_LEFT_ANGLE = 90.0;

    private static final double TURN_RIGHT_ANGLE = 90.0;

    // =========================================================
    // SHOOTING
    // =========================================================

    private static final double SHOOT_TIME = 6.0;

    private static final double SHOOTER_POWER = 1.0;

    // =========================================================
    // INTAKE
    // =========================================================

    private static final double INTAKE_POWER = 1.0;

    // =========================================================
    // DRIVE POWER
    // =========================================================

    private static final double MAX_DRIVE_POWER = 0.65;

    private static final double MIN_DRIVE_POWER = 0.15;

    // =========================================================
    // TURN POWER
    // =========================================================

    private static final double MAX_TURN_POWER = 0.50;

    private static final double MIN_TURN_POWER = 0.12;

    // =========================================================
    // ODOMETRY TOLERANCES
    // =========================================================

    private static final double DRIVE_TOLERANCE = 0.75;

    private static final double TURN_TOLERANCE = 1.5;

    // =========================================================
    // LIMELIGHT
    // =========================================================

    /*
     * Pipeline 8 is assumed to be your hive/AprilTag pipeline.
     */
    private static final int LIMELIGHT_PIPELINE = 8;

    /*
     * How close tx needs to be to consider the robot aligned.
     */
    private static final double LIMELIGHT_TX_TOLERANCE = 1.0;

    /*
     * Maximum rotation power while aligning.
     */
    private static final double LIMELIGHT_MAX_POWER = 0.25;

    /*
     * Proportional gain for Limelight alignment.
     *
     * If the robot turns the WRONG direction, change this
     * from -0.015 to +0.015.
     */
    private static final double LIMELIGHT_KP = -0.015;

    /*
     * Don't let the robot wait forever for the Limelight.
     */
    private static final double LIMELIGHT_TIMEOUT = 3.0;

    // =========================================================
    // SHOOTER SERVO POSITIONS
    // =========================================================

    /*
     * CHANGE THESE TO YOUR ACTUAL SERVO POSITIONS.
     */
    private static final double STAGE1_IDLE = 0.0;
    private static final double STAGE1_FIRE = 1.0;

    private static final double STAGE2_IDLE = 0.0;
    private static final double STAGE2_FIRE = 1.0;

    private static final double STAGE3_IDLE = 0.0;
    private static final double STAGE3_FIRE = 1.0;

    private static final double STAGE4_IDLE = 0.0;
    private static final double STAGE4_FIRE = 1.0;

    // =========================================================
    // ODOMETRY STARTING VALUES
    // =========================================================

    private double startX;
    private double startY;

    private double turnTarget;

    // =========================================================
    // INITIALIZE
    // =========================================================

    @Override
    public void init() {

        robot.init(hardwareMap);

        // Start Limelight.
        robot.limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        robot.limelight.start();

        // Put shooter off.
        robot.shooter.setPower(0);

        // Put intake off.
        robot.intakeMotor.setPower(0);

        // Put servos into their idle positions.
        setShooterServosIdle();

        // Make sure odometry is updated.
        robot.updateOdo();

        telemetry.addLine("BasicAuto Initialized");
        telemetry.addLine("Odometry: READY");
        telemetry.addLine("Limelight: READY");
        telemetry.addLine("Press PLAY to start.");
        telemetry.update();
    }

    // =========================================================
    // INIT LOOP
    // =========================================================

    @Override
    public void init_loop() {

        robot.updateOdo();

        Pose2D pose = robot.getOdoPosition();

        telemetry.addData(
                "Odo X",
                "%.2f in",
                pose.getX(DistanceUnit.INCH)
        );

        telemetry.addData(
                "Odo Y",
                "%.2f in",
                pose.getY(DistanceUnit.INCH)
        );

        telemetry.addData(
                "Odo Heading",
                "%.2f°",
                pose.getHeading(AngleUnit.DEGREES)
        );

        telemetry.update();
    }

    // =========================================================
    // START
    // =========================================================

    @Override
    public void start() {

        robot.resetOdo();

        robot.updateOdo();

        Pose2D startingPose = robot.getOdoPosition();

        startX = startingPose.getX(DistanceUnit.INCH);
        startY = startingPose.getY(DistanceUnit.INCH);

        stageTimer.reset();

        currentStage = AutoStage.DRIVE_FORWARD_1;
    }

    // =========================================================
    // MAIN LOOP
    // =========================================================

    @Override
    public void loop() {

        robot.updateOdo();

        switch (currentStage) {

            // =================================================
            // STAGE 1
            // DRIVE FORWARD 60 INCHES
            // =================================================

            case DRIVE_FORWARD_1:

                DRIVE_FORWARD_1();

                break;

            // =================================================
            // STAGE 2
            // TURN LEFT 90 DEGREES
            // =================================================

            case TURN_LEFT_1:

                TURN_LEFT_1();

                break;

            // =================================================
            // STAGE 3
            // MOVE BACKWARD 36 INCHES
            // =================================================

            case DRIVE_BACKWARD_1:

                DRIVE_BACKWARD_1();

                break;

            // =================================================
            // STAGE 4
            // ALIGN WITH HIVE
            // =================================================

            case ALIGN_TO_HIVE:

                ALIGN_TO_HIVE();

                break;

            // =================================================
            // STAGE 5
            // SHOOT
            // =================================================

            case SHOOT:

                SHOOT();

                break;

            // =================================================
            // STAGE 6
            // TURN RIGHT
            // =================================================

            case TURN_RIGHT_1:

                TURN_RIGHT_1();

                break;

            // =================================================
            // STAGE 7
            // START INTAKE
            // =================================================

            case START_INTAKE:

                START_INTAKE();

                break;

            // =================================================
            // STAGE 8
            // BACK UP 60 INCHES
            // =================================================

            case DRIVE_BACKWARD_2:

                DRIVE_BACKWARD_2();

                break;

            // =================================================
            // FINISHED
            // =================================================

            case STOP:

                STOP();

                break;
        }

        telemetry.addData(
                "Stage",
                currentStage
        );

        telemetry.addData(
                "Odo X",
                "%.2f",
                robot.getOdoPositionX(DistanceUnit.INCH)
        );

        telemetry.addData(
                "Odo Y",
                "%.2f",
                robot.getOdoPositionY(DistanceUnit.INCH)
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                robot.getOdoHeading(AngleUnit.DEGREES)
        );

        telemetry.update();
    }

    // =========================================================
    // STAGE METHODS
    // =========================================================

    /*
     * 60 INCHES FORWARD
     */
    private void DRIVE_FORWARD_1() {

        if (driveDistance(
                FORWARD_DISTANCE_1,
                true
        )) {

            robot.stopDrive();

            startX = robot.getOdoPositionX(
                    DistanceUnit.INCH
            );

            startY = robot.getOdoPositionY(
                    DistanceUnit.INCH
            );

            nextStage(
                    AutoStage.TURN_LEFT_1
            );
        }
    }

    /*
     * TURN LEFT 90 DEGREES
     */
    private void TURN_LEFT_1() {

        if (stageTimer.seconds() == 0) {

            double currentHeading =
                    robot.getOdoHeading(
                            AngleUnit.DEGREES
                    );

            turnTarget =
                    normalizeAngle(
                            currentHeading
                                    + TURN_LEFT_ANGLE
                    );
        }

        if (turnToHeading(turnTarget)) {

            robot.stopDrive();

            startX = robot.getOdoPositionX(
                    DistanceUnit.INCH
            );

            startY = robot.getOdoPositionY(
                    DistanceUnit.INCH
            );

            nextStage(
                    AutoStage.DRIVE_BACKWARD_1
            );
        }
    }

    /*
     * 36 INCHES BACKWARD
     */
    private void DRIVE_BACKWARD_1() {

        if (driveDistance(
                BACKWARD_DISTANCE_1,
                false
        )) {

            robot.stopDrive();

            nextStage(
                    AutoStage.ALIGN_TO_HIVE
            );
        }
    }

    /*
     * ALIGN TO HIVE USING LIMELIGHT
     */
    private void ALIGN_TO_HIVE() {

        LLResult result =
                robot.limelight.getLatestResult();

        /*
         * If no valid target is detected,
         * wait until one appears.
         */
        if (result == null || !result.isValid()) {

            robot.stopDrive();

            telemetry.addLine(
                    "Waiting for hive target..."
            );

            /*
             * Safety timeout.
             * If Limelight cannot see the target,
             * continue instead of getting stuck forever.
             */
            if (stageTimer.seconds()
                    >= LIMELIGHT_TIMEOUT) {

                robot.stopDrive();

                nextStage(
                        AutoStage.SHOOT
                );
            }

            return;
        }

        double tx = result.getTx();

        telemetry.addData(
                "Hive TX",
                "%.2f",
                tx
        );

        /*
         * Target is centered.
         */
        if (Math.abs(tx)
                <= LIMELIGHT_TX_TOLERANCE) {

            robot.stopDrive();

            nextStage(
                    AutoStage.SHOOT
            );

            return;
        }

        /*
         * Proportional rotation.
         */
        double turnPower =
                tx * LIMELIGHT_KP;

        turnPower =
                clip(
                        turnPower,
                        -LIMELIGHT_MAX_POWER,
                        LIMELIGHT_MAX_POWER
                );

        /*
         * Prevent very small motor commands
         * from being ineffective.
         */
        if (Math.abs(turnPower)
                < MIN_TURN_POWER) {

            turnPower =
                    Math.copySign(
                            MIN_TURN_POWER,
                            turnPower
                    );
        }

        robot.setDrivePower(
                turnPower,
                -turnPower,
                turnPower,
                -turnPower
        );
    }

    /*
     * SHOOT FOR 6 SECONDS
     */
    private void SHOOT() {

        robot.stopDrive();

        robot.shooter.setPower(
                SHOOTER_POWER
        );

        runShootingSequence();

        if (stageTimer.seconds()
                >= SHOOT_TIME) {

            robot.shooter.setPower(0);

            setShooterServosIdle();

            nextStage(
                    AutoStage.TURN_RIGHT_1
            );
        }
    }

    /*
     * TURN RIGHT 90 DEGREES
     */
    private void TURN_RIGHT_1() {

        if (stageTimer.seconds() == 0) {

            double currentHeading =
                    robot.getOdoHeading(
                            AngleUnit.DEGREES
                    );

            turnTarget =
                    normalizeAngle(
                            currentHeading
                                    - TURN_RIGHT_ANGLE
                    );
        }

        if (turnToHeading(turnTarget)) {

            robot.stopDrive();

            nextStage(
                    AutoStage.START_INTAKE
            );
        }
    }

    /*
     * START INTAKE
     */
    private void START_INTAKE() {

        robot.intakeMotor.setPower(
                INTAKE_POWER
        );

        nextStage(
                AutoStage.DRIVE_BACKWARD_2
        );
    }

    /*
     * BACKWARD 60 INCHES
     */
    private void DRIVE_BACKWARD_2() {

        /*
         * Keep intake running while backing up.
         */
        robot.intakeMotor.setPower(
                INTAKE_POWER
        );

        if (driveDistance(
                BACKWARD_DISTANCE_2,
                false
        )) {

            robot.stopDrive();

            robot.intakeMotor.setPower(0);

            nextStage(
                    AutoStage.STOP
            );
        }
    }

    /*
     * FINAL STOP
     */
    private void STOP() {

        robot.stopAllMotors();

        telemetry.addLine(
                "AUTONOMOUS COMPLETE"
        );
    }

    // =========================================================
    // DRIVE DISTANCE USING ODOMETRY
    // =========================================================

    /*
     * Uses the Pinpoint odometry position instead of
     * timing the motors.
     *
     * forward = true:
     *     robot travels forward
     *
     * forward = false:
     *     robot travels backward
     */
    private boolean driveDistance(
            double targetDistance,
            boolean forward
    ) {

        double currentX =
                robot.getOdoPositionX(
                        DistanceUnit.INCH
                );

        double currentY =
                robot.getOdoPositionY(
                        DistanceUnit.INCH
                );

        double currentHeading =
                robot.getOdoHeading(
                        AngleUnit.RADIANS
                );

        double deltaX =
                currentX - startX;

        double deltaY =
                currentY - startY;

        /*
         * Project odometry movement onto the robot's
         * original forward direction.
         */
        double distanceTraveled =
                deltaX * Math.cos(currentHeading)
                        + deltaY * Math.sin(currentHeading);

        /*
         * If we're driving backward, reverse
         * the measured displacement.
         */
        if (!forward) {

            distanceTraveled =
                    -distanceTraveled;
        }

        double error =
                targetDistance
                        - distanceTraveled;

        telemetry.addData(
                "Target Distance",
                "%.2f",
                targetDistance
        );

        telemetry.addData(
                "Distance Traveled",
                "%.2f",
                distanceTraveled
        );

        telemetry.addData(
                "Distance Error",
                "%.2f",
                error
        );

        /*
         * Reached target.
         */
        if (Math.abs(error)
                <= DRIVE_TOLERANCE) {

            robot.stopDrive();

            return true;
        }

        /*
         * Proportional speed control.
         *
         * Far away = faster
         * Close = slower
         */
        double power =
                Math.abs(error)
                        * 0.025;

        power =
                clip(
                        power,
                        MIN_DRIVE_POWER,
                        MAX_DRIVE_POWER
                );

        /*
         * Slow down very close to the target.
         */
        if (Math.abs(error) < 6.0) {

            power =
                    Math.min(
                            power,
                            0.25
                    );
        }

        double direction =
                forward ? 1.0 : -1.0;

        robot.setDrivePower(
                direction * power,
                direction * power,
                direction * power,
                direction * power
        );

        return false;
    }

    // =========================================================
    // TURN USING ODOMETRY HEADING
    // =========================================================

    private boolean turnToHeading(
            double targetHeading
    ) {

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
                "Turn Target",
                "%.2f",
                targetHeading
        );

        telemetry.addData(
                "Current Heading",
                "%.2f",
                currentHeading
        );

        telemetry.addData(
                "Turn Error",
                "%.2f",
                error
        );

        /*
         * Target reached.
         */
        if (Math.abs(error)
                <= TURN_TOLERANCE) {

            robot.stopDrive();

            return true;
        }

        /*
         * Proportional turn speed.
         */
        double power =
                Math.abs(error)
                        * 0.012;

        power =
                clip(
                        power,
                        MIN_TURN_POWER,
                        MAX_TURN_POWER
                );

        double direction =
                Math.signum(error);

        double turnPower =
                direction * power;

        /*
         * Mecanum rotation:
         *
         * Left side  = one direction
         * Right side = opposite direction
         */
        robot.setDrivePower(
                turnPower,
                -turnPower,
                turnPower,
                -turnPower
        );

        return false;
    }

    // =========================================================
    // SHOOTING SEQUENCE
    // =========================================================

    private void runShootingSequence() {

        /*
         * Currently all four stages are activated
         * throughout the shooting period.
         *
         * If your mechanism requires sequential
         * movement, this can be changed.
         */

        robot.Stage1Servo.setPosition(
                STAGE1_FIRE
        );

        robot.Stage2Servo.setPosition(
                STAGE2_FIRE
        );

        robot.Stage3Servo.setPosition(
                STAGE3_FIRE
        );

        robot.Stage4Servo.setPosition(
                STAGE4_FIRE
        );
    }

    // =========================================================
    // RESET SHOOTER SERVOS
    // =========================================================

    private void setShooterServosIdle() {

        robot.Stage1Servo.setPosition(
                STAGE1_IDLE
        );

        robot.Stage2Servo.setPosition(
                STAGE2_IDLE
        );

        robot.Stage3Servo.setPosition(
                STAGE3_IDLE
        );

        robot.Stage4Servo.setPosition(
                STAGE4_IDLE
        );
    }

    // =========================================================
    // CHANGE STAGE
    // =========================================================

    private void nextStage(
            AutoStage nextStage
    ) {

        currentStage = nextStage;

        stageTimer.reset();

        robot.stopDrive();
    }

    // =========================================================
    // ANGLE NORMALIZATION
    // =========================================================

    private double normalizeAngle(
            double angle
    ) {

        while (angle > 180.0) {

            angle -= 360.0;
        }

        while (angle < -180.0) {

            angle += 360.0;
        }

        return angle;
    }

    // =========================================================
    // ANGLE DIFFERENCE
    // =========================================================

    private double angleDifference(
            double target,
            double current
    ) {

        return normalizeAngle(
                target - current
        );
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

        robot.stopAllMotors();

        robot.limelight.stop();
    }
}
