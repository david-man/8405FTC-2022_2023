package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    public Drive(DcMotor getFrontLeft,
                 DcMotor getBackLeft,
                 DcMotor getFrontRight,
                 DcMotor getBackRight) {
        motorFrontLeft = getFrontLeft;
        motorBackLeft = getBackLeft;
        motorFrontRight = getFrontRight;
        motorBackRight = getBackRight;
    }

    public void move_to_hendry(double x, double y) {
        while (true) {
            int move_to_count = (int) move_to_count + 1;
            double phi = heading * Math.pi / 180;
            double IMU_error = -1 * (heading - new_heading);
            double y_error = new_y - y;
            double x_error = -1 * (new_x - x);

            double power = power_PD.get_value(y_error * Math.cos(phi) + x_error * Math.sin(phi));
            double strafe = strafe_PD.get_value(x_error * Math.cos(phi) - y_error * Math.sin(phi));
            double turn = turn_PD.get_value(IMU_error) * turn_coefficient;
            mecanum(power, strafe, turn, 127);
        }
	delay(5);
    }

    public void moveToKedaar(double x, double y) {
        while(true) {
            move_to_count = (int)move_to_count + 1;
            double phi = heading * Math.pi / 180;

            double imuError = -(heading - new_heading);
            double yError = new_y - y;
            double xError = (new_x - x) * -1;

            double power = power_PD.get_value(yError * std::cos(phi) + xError * std::sin(phi));
            double strafe = strafe_PD.get_value(xError * std::cos(phi) - yError * std::sin(phi));
            double turn = turn_PD.get_value(imuError) * turn_coefficient;

            mecanum(power, strafe, turn, 127);

            delay(5);
        }
    }

    public void mecanum() {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // denominator is largest motor power
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }
}
