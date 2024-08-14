package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.LinkedList;
import java.util.Queue;

// Inspired by QC 21229
// An alternative to Actions.runBlocking() that doesn't block. Main use-case is for tele-op.
// Call run() in every loop and add actions as needed.
public class ActionScheduler {
    final Queue<Action> actions = new LinkedList<>();
    final FtcDashboard dash = FtcDashboard.getInstance();
    final Canvas canvas = new Canvas();

    public void addAction(Action action) {
        actions.add(action);
    }

    // Won't generate previews
    public void run() {
        if (actions.peek() != null) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            boolean running = actions.peek().run(packet);
            dash.sendTelemetryPacket(packet);

            if (!running) {
                actions.remove();
            }
        }
    }
}