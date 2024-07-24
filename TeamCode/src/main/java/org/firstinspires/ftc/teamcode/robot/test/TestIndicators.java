package org.firstinspires.ftc.teamcode.robot.test;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.gamepadEx1;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.keyPressed;
import static org.firstinspires.ftc.teamcode.util.LEDIndicator.State.AMBER;
import static org.firstinspires.ftc.teamcode.util.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.util.LEDIndicator.State.OFF;
import static org.firstinspires.ftc.teamcode.util.LEDIndicator.State.RED;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.BulkReader;
import org.firstinspires.ftc.teamcode.util.LEDIndicator;


@TeleOp(name = "Test Indicators", group = "Mechanism Test")
public final class TestIndicators extends LinearOpMode {

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        BulkReader bulkReader = new BulkReader(hardwareMap);
        LEDIndicator[] indicators = {
                new LEDIndicator(hardwareMap, "led left green", "led left red"),
                new LEDIndicator(hardwareMap, "led right green", "led right red")
        };

        waitForStart();

        while (opModeIsActive()) {
            bulkReader.bulkRead();

            gamepadEx1.readButtons();

            if (keyPressed(1, B)) {
                for (LEDIndicator indicator : indicators) indicator.setState(RED);
            }
            if (keyPressed(1, A)) {
                for (LEDIndicator indicator : indicators) indicator.setState(GREEN);
            }
            if (keyPressed(1, X)) {
                for (LEDIndicator indicator : indicators) indicator.setState(OFF);
            }
            if (keyPressed(1, Y)) {
                for (LEDIndicator indicator : indicators) indicator.setState(AMBER);
            }
        }
    }
}
