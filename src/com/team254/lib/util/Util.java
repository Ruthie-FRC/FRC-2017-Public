package com.team254.lib.util;

import java.util.List;

/**
 * Contains basic utility functions that are used often.
 * Modernized with improved structure.
 */
public final class Util {
    
    /** Prevent this class from being instantiated. */
    private Util() {
        throw new AssertionError("Utility class should not be instantiated");
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static String joinStrings(String delim, List<?> strings) {
        var sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        for (var valueIn : list) {
            if (!epsilonEquals(valueIn, value, epsilon)) {
                return false;
            }
        }
        return true;
    }
}
