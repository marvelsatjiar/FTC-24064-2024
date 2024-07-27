package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Robot;

@Config
@Autonomous(name = "testBuildQueueAuton", group = "24064 Main", preselectTeleOp = "MainTeleOp")
public class testBuildQueue extends LinearOpMode {

    static Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        Action traj = robot.drivetrain.actionBuilder(MainAuton.startRed).lineToY(MainAuton.startRed.position.y + 2).build();

        waitForStart();

        Actions.runBlocking(traj);
    }
}
