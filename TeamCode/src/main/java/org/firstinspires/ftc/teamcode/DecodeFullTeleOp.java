package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DecodeFullTeleOp", group = "Competition")
public class DecodeFullTeleOp extends LinearOpMode {

    private DecodeChassis chassis;
    private DecodeIntake intake;
    private DecodeShooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        // Chassis motors
        DcMotor lf = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rf = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor lb = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rightBack");

        // Other motors
        DcMotorEx carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        Servo pusherServo = hardwareMap.get(Servo.class, "pusher");

        chassis = new DecodeChassis(lf, rf, lb, rb);
        intake = new DecodeIntake(intakeMotor);
        shooter = new DecodeShooter(shooterMotor, carousel, pusherServo);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // -------------------
            // Drive (gamepad1)
            // -------------------
            chassis.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
                    gamepad1.right_trigger > 0.05);

            // -------------------
            // Intake (gamepad1)
            // -------------------
            intake.run(gamepad1.left_bumper, gamepad1.left_trigger > 0.05, gamepad1.left_trigger,
                    gamepad1.a);

            // -------------------
            // Shooter (gamepad2)
            // -------------------
            if (gamepad2.x) shooter.setShooterRPM(1000);
            if (gamepad2.a) shooter.setShooterRPM(2500);
            if (gamepad2.b) shooter.setShooterRPM(5000);
            if (gamepad2.y) shooter.setShooterRPM(0);
            shooter.updateCurrentRPM();

            // -------------------
            // Pusher (gamepad2)
            // -------------------
            if (gamepad2.right_bumper) shooter.pushServo();

            // -------------------
            // Carousel (gamepad2)
            // -------------------
            if (gamepad2.left_bumper && gamepad2.dpad_right) shooter.rotateCarousel(60);
            else if (gamepad2.left_bumper && gamepad2.dpad_left) shooter.rotateCarousel(-60);
            else if (gamepad2.dpad_right) shooter.rotateCarousel(120);
            else if (gamepad2.dpad_left) shooter.rotateCarousel(-120);

            // -------------------
            // Telemetry
            // -------------------
            telemetry.clearAll();
            telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
            telemetry.addData("Target RPM", shooter.getTargetRPM());
            telemetry.addData("Target Met", shooter.isTargetMet());
            telemetry.update();
        }
    }
}

