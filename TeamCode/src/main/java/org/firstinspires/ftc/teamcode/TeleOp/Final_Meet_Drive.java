package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="Final_Meet_Drive", group="Iterative Opmode")
public class Final_Meet_Drive extends OpMode
{
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    // wheels lol

    private DcMotor arm;
    private DcMotor arm2;
    private DcMotor ducky;
    private DcMotor intake;
    // attachment motors

    private TouchSensor touch;

    ElapsedTime timer;
    public enum ArmState {
        BOTTOM,
        LOWER,
        MIDDLE,
        UPPER,
        RESET
    };
    ArmState levels;

    boolean moving;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        ducky = hardwareMap.get(DcMotor.class, "ducky");
        intake = hardwareMap.get(DcMotor.class, "intake");

        touch = hardwareMap.get(TouchSensor.class, "touch");

        moving =    false;
        //  touch.setMode(DigitalChannel.Mode.INPUT);
        // touch sensor that reads environment

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        /*
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        */


        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        levels = ArmState.BOTTOM;
        timer = new ElapsedTime();
        timer.reset();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void loop() {
        // Assigning & Data
        double lefty1 = -(gamepad1.left_stick_y);
        double leftx1 = gamepad1.left_stick_x;
        double rightx1 = -gamepad1.right_stick_x;
        double righty1 = (gamepad1.right_stick_y);
        double lefty2 = -(gamepad2.left_stick_y);
        double leftx2 = gamepad2.left_stick_x;
        double rightx2 = gamepad2.right_stick_x;
        double righty2 = (gamepad2.right_stick_y);
        boolean buttonUp = gamepad1.dpad_up;
        boolean buttonDown = gamepad1.dpad_down;
        boolean buttonLeft = gamepad1.dpad_left;
        boolean buttonRight = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean a1 = gamepad1.a;
        boolean b1 = gamepad1.b;
        boolean x1 = gamepad1.x;
        boolean y1 = gamepad1.y;
        boolean rt = gamepad1.right_stick_button;

        boolean buttonUp2 = gamepad2.dpad_up;
        boolean buttonDown2 = gamepad2.dpad_down;
        boolean buttonLeft2 = gamepad2.dpad_left;
        boolean b2 = gamepad2.b;
        boolean a2 = gamepad2.a;
        boolean y2 = gamepad2.y;
        boolean x2 = gamepad2.x;

        int redFront;
        int redMid;
        int greenFront;
        int greenMid;
        int blueFront;
        int blueMid;
        // for color sensors XD

        telemetry.addData("lefty1", lefty1);
        telemetry.addData("leftx1", leftx1);
        telemetry.addData("rightx1", rightx1);
        telemetry.addData("lefty2", lefty2);
        telemetry.addData("leftx2", leftx2);
        telemetry.addData("rightx2", rightx2);
        telemetry.addData("buttonUp", buttonUp);
        telemetry.addData("buttonDown", buttonDown);
        telemetry.addData("buttonRight", buttonRight);
        telemetry.addData("buttonLeft", buttonLeft);
        telemetry.addData("lb", lb);
        telemetry.addData("rb", rb);
        telemetry.addData("a", a2);
        telemetry.addData("b", b2);
        telemetry.addData("x", x2);
        telemetry.addData("y", y2);
        telemetry.addData("rt", rt);
        telemetry.addData("Touch Detected?", touch.isPressed());

        // Finite State Machine - Levels
        final int lower = 300;
        final int middle = 600;
        final int upper = 1000;
        final double speed = .6;


        switch (levels) {
            case BOTTOM:
                if (a2) {
                    timer.reset();
                    levels = ArmState.LOWER;
                }
                if (b2) {
                    timer.reset();
                    levels = ArmState.MIDDLE;
                }
                if (y2) {
                    timer.reset();
                    levels = ArmState.UPPER;
                }
                if (x2) {
                    levels = ArmState.UPPER;
                }
                break;
            case LOWER:
                if (timer.milliseconds() < lower) {
                    arm.setPower(speed);
                    arm2.setPower(speed);
                    moving = true;
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                    moving = false;
                }
                if (x2) {
                    levels = ArmState.RESET;
                }
                break;
            case MIDDLE:
                if (timer.milliseconds() < middle) {
                    arm.setPower(speed);
                    arm2.setPower(speed);
                    moving = true;
                } else {
                    arm.setPower(0);
                    arm2.setPower(0);
                    moving = false;
                }
                if (x2) {
                    levels = ArmState.RESET;
                }
                break;
            case UPPER:
                if (timer.milliseconds() < upper) {
                    arm.setPower(speed);
                    arm2.setPower(speed);
                    moving = true;
                } else if (Math.abs(lefty2) <= .1 && !buttonDown2 && !buttonUp2){
                    arm.setPower(0);
                    arm2.setPower(0);
                    moving = false;
                }
                if (x2) {
                    levels = ArmState.RESET;
                }
                break;
            case RESET:
                if (!touch.isPressed()) {
                    arm.setPower(-speed*.5);
                    arm2.setPower(-speed*.5);
                } else {
                    levels = ArmState.BOTTOM;
                }
                if (buttonLeft2) levels = ArmState.BOTTOM;
                break;
            default:
                levels = ArmState.BOTTOM;
        }



        telemetry.addData("level", levels);
        telemetry.update();

        double pow;
        if (a1) pow = 1; // turbo mode
        else pow = .4;
        double c = Math.hypot(leftx1, lefty1);
        double perct = pow * c;
        if (c <= .1) perct = 0;
        //
        double theta;

        if (leftx1 <= 0 && lefty1 >= 0) {
            theta = Math.atan(Math.abs(leftx1) / Math.abs(lefty1));
            theta += (Math.PI / 2);
        } else if (leftx1 < 0 && lefty1 <= 0) {
            theta = Math.atan(Math.abs(lefty1) / Math.abs(leftx1));
            theta += (Math.PI);
        } else if (leftx1 >= 0 && lefty1 < 0) {
            theta = Math.atan(Math.abs(leftx1) / Math.abs(lefty1));
            theta += (3 * Math.PI / 2);
        } else {
            theta = Math.atan(Math.abs(lefty1) / Math.abs(leftx1));
        }

        double dir = 1;
        if (theta >= Math.PI) {
            theta -= Math.PI;
            dir = -1;
        }
        //if (leftx1 <= 0 && lefty1 >= 0 || leftx1 >= 0 && lefty1 <= 0){
        //   theta += (Math.PI/2);
        //}

        telemetry.addData("pow", pow);
        telemetry.addData("dir", dir);
        telemetry.addData("c", c);
        telemetry.addData("theta", theta);

        double fr = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (fr > 1) fr = 1;
        if (fr < -1) fr = -1;
        fr = (perct * fr);
        if (leftx1 == 0 && lefty1 == 0) fr = 0;

        double bl = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (bl > 1) bl = 1;
        if (bl < -1) bl = -1;
        bl = (perct * bl);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) bl = 0;

        double fl = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (fl > 1) fl = 1;
        if (fl < -1) fl = -1;
        fl = (perct * fl);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) fl = 0;

        double br = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (br > 1) br = 1;
        if (br < -1) br = -1;
        br = (perct * br);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) br = 0;

        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("br", br);

        telemetry.addData("rlf", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rrf", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbl", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbr", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));

        rightx1 *= .6;
        if (!buttonDown && !buttonLeft && !buttonRight && !buttonUp && !lb && !rb) {
            leftFront.setPower(fl + rightx1);
            leftBack.setPower(bl + rightx1);
            rightFront.setPower(fr - rightx1);
            rightBack.setPower(br - rightx1);
        }

        pow = .6;

        if (Math.abs(lefty2) >= .1) {
            if (lefty2 < 0 && !touch.isPressed()) {
                arm.setPower(lefty2 * pow);
                arm2.setPower(lefty2 * pow);
            }
            if (lefty2 > 0) {
                arm.setPower(lefty2 * pow);
                arm2.setPower(lefty2 * pow);
            }
        } else if (buttonDown2 && !touch.isPressed()) {
            arm.setPower(-pow / 2);
            arm2.setPower(-pow / 2);
        } else if (buttonUp2) {
            arm.setPower(pow / 2);
            arm2.setPower(pow / 2);
        } else if (!moving) {
            arm.setPower(0);
            arm2.setPower(0);
        }

        if (touch.isPressed() && arm.getPower() < 0 && arm2.getPower() < 0) {
            arm.setPower(0);
            arm2.setPower(0);
        }

        if (Math.abs(righty2) >= .1) {
            intake.setPower(-righty2);
        } else {
            intake.setPower(0);
        }


        pow = .4;

        // Below: precision (slower) movement
        pow *= 0.5;
        double pow2 = pow*.9;
        if (buttonUp) {
            // slowly moves forwards
            leftFront.setPower(pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(pow);
        } else if (buttonDown) {
            // slowly moves backwards
            leftFront.setPower(-pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(-pow);
        } else if (buttonRight) {
            // slowly moves right
            leftFront.setPower(pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(pow);
        } else if (buttonLeft) {
            // slowly moves left
            leftFront.setPower(-pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(-pow);
        } else if (rb) {
            // slowly moves clockwise
            leftFront.setPower(pow2);
            leftBack.setPower(pow2);
            rightFront.setPower(-pow2);
            rightBack.setPower(-pow2);
        } else if (lb) {
            // slowly moves counter-clockwise
            leftFront.setPower(-pow2);
            leftBack.setPower(-pow2);
            rightFront.setPower(pow2);
            rightBack.setPower(pow2);
        } else if (Math.abs(rightx1) <= .1 && Math.abs(leftx1) <= .1 && Math.abs(lefty1) <= .1) {
            // stops movement
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

        pow *= .6;


        ducky.setPower(rightx2*.65);
        if (rightx2 == 1) ducky.setPower(.75);
        if (rightx2 == -1) ducky.setPower(-.75);

        // Ensures Data Updates
        telemetry.update();
    }
    @Override
    public void stop() {
    }

}