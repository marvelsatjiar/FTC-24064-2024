package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.LinkedList;
import java.util.Queue;

// QC 21229
public class AutoActionScheduler {
    final Queue<Action> actions = new LinkedList<>();
    final FtcDashboard dashboard = FtcDashboard.getInstance();
    final Canvas canvas = new Canvas();
    final Runnable async; // Anything that needs to be run asynchronously (like PID loops or whatnot)

    public AutoActionScheduler(Runnable async) {
        this.async = async;
    }

    public void addAction(Action action) {
        actions.add(action);
    }

    public void run() {
        while (actions.peek() != null && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            async.run();

            boolean running = actions.peek().run(packet);
            dashboard.sendTelemetryPacket(packet);

            if (!running) {
                actions.remove();
                // Shows any action previews if needed
                if (actions.peek() != null) {
                    actions.peek().preview(canvas);
                }
            }
        }
    }
}