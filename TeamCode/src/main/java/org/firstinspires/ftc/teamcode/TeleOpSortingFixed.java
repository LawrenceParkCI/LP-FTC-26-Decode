package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpSortingFixed", group = "TeleOp")
class TeleOpSortingFixed extends OpMode {

    // Drive motors
    DcMotor frontLeft, backLeft, frontRight, backRight;

    // Intake
    DcMotor intake;

    // Shooter motors
    DcMotorEx shooterLeft, shooterRight;

    // Servos
    Servo indexerServo;
    CRServo continuousServo;

    // Color sensor
    ColorSensor colorSensor;

    // Timer
    ElapsedTime runtime = new ElapsedTime();

    // Ball storage
    List<String> storedBalls = new ArrayList<>();
    final int MAX_STORED = 3;

    // Indexer servo positions
    final double INDEXER_REST = 0.5;
    final double INDEXER_UP = 0.9;
    final double INDEXER_DOWN = 0.1;

    boolean indexerBusy = false;
    double indexerStartTime = 0;

    // Shooter target
    double targetRPM = 0;

    // Low speed mode
    boolean lowSpeedMode = false;

    @Override
    public void init() {
        // Drive
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Shooter
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        // Servos
        indexerServo = hardwareMap.get(Servo.class, "indexerServo");
        continuousServo = hardwareMap.get(CRServo.class, "continuousServo");

        // Color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        indexerServo.setPosition(INDEXER_REST);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // ----- Driving -----
        double ly = -gamepad1.left_stick_y; // forward/back
        double lx = gamepad1.left_stick_x;  // strafe
        double rx = gamepad1.right_stick_x; // rotate

        // low speed if RT pressed
        lowSpeedMode = gamepad1.right_trigger > 0.1;
        double maxPower = lowSpeedMode ? 0.3 : 1.0;

        double frontLeftPower = Range.clip(ly + lx + rx, -1, 1) * maxPower;
        double frontRightPower = Range.clip(ly - lx - rx, -1, 1) * maxPower;
        double backLeftPower = Range.clip(ly - lx + rx, -1, 1) * maxPower;
        double backRightPower = Range.clip(ly + lx - rx, -1, 1) * maxPower;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // ----- Intake -----
        if (gamepad2.right_bumper) {
            intake.setPower(1);
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        // ----- Color Detection (simplified) -----
        if (gamepad2.right_bumper) {
            String color = detectColor();
            if (storedBalls.size() < MAX_STORED && !color.equals("none")) {
                if (storedBalls.isEmpty() || !storedBalls.get(storedBalls.size() - 1).equals(color)) {
                    storedBalls.add(color);
                }
            }
        }

        // ----- Shooter Control -----
        if (gamepad2.x) targetRPM = 1000;
        if (gamepad2.a) targetRPM = 2500;
        if (gamepad2.b) targetRPM = 5000;
        if (gamepad2.y) targetRPM = 0;

        if (targetRPM == 0) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        } else {
            double power = Range.clip(targetRPM / 5000.0, 0, 1);
            shooterLeft.setPower(power);
            shooterRight.setPower(power);
        }

        // ----- Firing Servo -----
        if (!indexerBusy && gamepad2.right_trigger > 0.2) {
            indexerBusy = true;
            indexerStartTime = runtime.milliseconds();
            indexerServo.setPosition(INDEXER_UP);
        }

        if (indexerBusy) {
            if (runtime.milliseconds() > indexerStartTime + 300) {
                indexerServo.setPosition(INDEXER_REST);
                indexerBusy = false;
            }
        }

        // ----- Telemetry -----
        telemetry.addData("Low Speed Mode", lowSpeedMode);
        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Stored Balls", storedBalls.toString());
        telemetry.addData("Detected Color", detectColor());
        telemetry.update();
    }

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        if (r > g && r > b && r > 100) return "red";
        if (b > r && b > g && b > 100) return "blue";
        if (g > r && g > b && g > 100) return "green";
        return "none";
    }
}
