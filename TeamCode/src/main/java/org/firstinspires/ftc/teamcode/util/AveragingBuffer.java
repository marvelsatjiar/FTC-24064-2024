package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;

import java.util.concurrent.ArrayBlockingQueue;

public class AveragingBuffer {
    private final int size;
    private final EvictingBlockingQueue<Double> buffer;

    public AveragingBuffer(int size) {
        this.size = size;
        buffer = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(size));
    }

    public double put(double value) {
        try {
            buffer.put(value);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return average();
    }

    public double average() {
        double sum = 0;
        for (double value : buffer) {
            sum += value;
        }
        return sum / size;
    }
}