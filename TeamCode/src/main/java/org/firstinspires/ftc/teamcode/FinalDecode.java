package org.firstinspires.ftc.teamcode;

// Required FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "FinalDecode", group = "TeleOp")
public class FinalDecode extends LinearOpMode {

    // Drive (Control Hub)
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Expansion Hub
    private DcMotor intake;          // Tetrix or similar
    private DcMotorEx shooter;       // GoBilda 6000RPM (5202) with encoder
    private DcMotorEx carousel;      // Gearboxed motor with encoder

    // Servo
    private Servo pusher;            // angular servo used as pusher

    // Shooter tracking
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean targetMet = false;

    // Carousel angle (degrees)
    private double carouselAngleDeg = 0.0;

    // Constants
    private static final double SHOOTER_PPR = 28.0;           // pulses per motor rev (5202)
    private static final double CAROUSEL_PPR = 2786.2;        // output pulses per rotation
    private static final double SERVO_FULL_RANGE_DEG = 300.0; // servo full sweep

    private static final double LOW_SPEED_SCALE = 0.30;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware map
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        pusher = hardwareMap.get(Servo.class, "pusher");

        // Directions
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo init
        setServoAngle(pusher, 0.0);

        telemetry.addLine("Init complete. Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----- DRIVE -----
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            double lf = ly + lx + rx;
            double rf = ly - lx - rx;
            double lb = ly - lx + rx;
            double rb = ly + lx - rx;

            double max = Math.max(1.0,
                    Math.max(Math.abs(lf),
                            Math.max(Math.abs(rf),
                                    Math.max(Math.abs(lb), Math.abs(rb)))));

            lf /= max; rf /= max; lb /= max; rb /= max;

            double speedLimit = gamepad1.right_trigger > 0.05 ? LOW_SPEED_SCALE : 1.0;

            leftFront.setPower(lf * speedLimit);
            rightFront.setPower(rf * speedLimit);
            leftBack.setPower(lb * speedLimit);
            rightBack.setPower(rb * speedLimit);

            // ----- INTAKE -----
            double intakePower = 0.0;
            if (gamepad1.left_bumper) intakePower = -1.0;
            else if (gamepad1.left_trigger > 0.05) intakePower = gamepad1.left_trigger;
            if (gamepad1.a) intakePower *= 0.30;
            intake.setPower(intakePower);

            // ----- SHOOTER (gamepad2) -----
            if (gamepad2.x) shoot(1000.0);
            else if (gamepad2.a) shoot(2500.0);
            else if (gamepad2.b) shoot(5000.0);

            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                double percent = Range.clip(-gamepad2.right_stick_y, -1.0, 1.0);
                setShooterTargetRPM(percent * 5000.0);
            }

            updateShooterRPM();

            // ----- CAROUSEL -----
            double carouselPower = 0.0;
            if (gamepad2.right_trigger > 0.05) carouselPower = gamepad2.right_trigger;
            else if (gamepad2.left_trigger > 0.05) carouselPower = -gamepad2.left_trigger;

            carousel.setVelocity(carouselPower * 50.0);

            if (gamepad2.dpad_up) moveCarouselByDegrees(60.0);
            if (gamepad2.dpad_down) moveCarouselByDegrees(-60.0);
            if (gamepad2.dpad_right) moveCarouselByDegrees(120.0);
            if (gamepad2.dpad_left) moveCarouselByDegrees(-120.0);

            // TELEMETRY
            telemetry.clearAll();
            telemetry.addData("Shooter RPM", currentRPM);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Target Met", targetMet);
            telemetry.addData("Carousel Encoder", carousel.getCurrentPosition());
            telemetry.update();
        }
    }

    // ======================
    // Helper Methods
    // ======================

    private void setServoAngle(Servo s, double angleDeg) {
        double pos = Range.clip(angleDeg / SERVO_FULL_RANGE_DEG, 0.0, 1.0);
        s.setPosition(pos);
    }

    private void shoot(double desiredRPM) {
        setShooterTargetRPM(desiredRPM);
        updateShooterRPM();

        while (opModeIsActive() && !targetMet) {
            updateShooterRPM();
            idle();
        }

        sleep(300);

        setServoAngle(pusher, 80.0);
        sleep(200);
        setServoAngle(pusher, 0.0);
        sleep(400);

        setShooterTargetRPM(0.0);
        updateShooterRPM();
    }

    private void setShooterTargetRPM(double desiredRPM) {
        targetRPM = desiredRPM;
        double ticksPerSec = desiredRPM * SHOOTER_PPR / 60.0;
        shooter.setVelocity(ticksPerSec);
        updateShooterRPM();
    }

    private void updateShooterRPM() {
        double ticksPerSec = shooter.getVelocity();
        currentRPM = ticksPerSec * 60.0 / SHOOTER_PPR;
        targetMet = currentRPM >= targetRPM - 1.0;
    }

    private void moveCarouselByDegrees(double deltaDeg) {
        double newAngle = normalizeAngle(carouselAngleDeg + deltaDeg);
        double targetRotations = newAngle / 360.0;
        double targetTicks = targetRotations * CAROUSEL_PPR;
        int targetTicksInt = (int) Math.round(targetTicks);

        carousel.setTargetPosition(targetTicksInt);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setPower(0.5);

        while (opModeIsActive() && carousel.isBusy()) {
            idle();
        }

        carousel.setPower(0.0);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        carouselAngleDeg = newAngle;
        sleep(200);
    }

    private double normalizeAngle(double a) {
        double v = a % 360.0;
        if (v < 0) v += 360.0;
        return v;
    }
}


