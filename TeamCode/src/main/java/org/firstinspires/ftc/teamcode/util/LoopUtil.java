package org.firstinspires.ftc.teamcode.util;

public class LoopUtil {
    private static long lastTime = 0;

    /**
     * @return The time between each function call
     */
    public static long getLoopTime()
    {
        long currentTime = System.nanoTime();
        long delta = lastTime - currentTime;
        lastTime = currentTime;
        return delta;
    }
}
