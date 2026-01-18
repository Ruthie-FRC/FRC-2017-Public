package com.team254.lib.util;

import java.util.LinkedList;

/**
 * Implements a simple circular buffer.
 * Modernized with improved naming and structure.
 */
public class CircularBuffer {
    private final int windowSize;
    private final LinkedList<Double> samples;
    private double sum;

    public CircularBuffer(int windowSize) {
        this.windowSize = windowSize;
        this.samples = new LinkedList<>();
        this.sum = 0.0;
    }

    public void clear() {
        samples.clear();
        sum = 0.0;
    }

    public double getAverage() {
        if (samples.isEmpty()) {
            return 0.0;
        }
        return sum / samples.size();
    }

    public void recomputeAverage() {
        // Reset any accumulation drift.
        sum = 0.0;
        if (samples.isEmpty()) {
            return;
        }
        for (var val : samples) {
            sum += val;
        }
        sum /= windowSize;
    }

    public void addValue(double val) {
        samples.addLast(val);
        sum += val;
        if (samples.size() > windowSize) {
            sum -= samples.removeFirst();
        }
    }

    public int getNumValues() {
        return samples.size();
    }

    public boolean isFull() {
        return windowSize == samples.size();
    }
}
