package com.team254.lib.util;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.concurrent.ConcurrentLinkedDeque;

/**
 * Writes data to a CSV file using reflection.
 * Modernized with improved naming and structure.
 */
public class ReflectingCSVWriter<T> {
    private final ConcurrentLinkedDeque<String> linesToWrite = new ConcurrentLinkedDeque<>();
    private PrintWriter output = null;
    private final Field[] fields;

    public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
        fields = typeClass.getFields();
        try {
            output = new PrintWriter(fileName);
        } catch (FileNotFoundException e) {
            System.err.println("Could not create CSV file: " + fileName);
            e.printStackTrace();
        }
        
        // Write field names
        var line = new StringBuilder();
        for (var field : fields) {
            if (line.length() != 0) {
                line.append(", ");
            }
            line.append(field.getName());
        }
        writeLine(line.toString());
    }

    public void add(T value) {
        var line = new StringBuilder();
        for (var field : fields) {
            if (line.length() != 0) {
                line.append(", ");
            }
            try {
                line.append(field.get(value).toString());
            } catch (IllegalArgumentException | IllegalAccessException e) {
                System.err.println("Error accessing field: " + field.getName());
                e.printStackTrace();
            }
        }
        linesToWrite.add(line.toString());
    }

    protected synchronized void writeLine(String line) {
        if (output != null) {
            output.println(line);
        }
    }

    /**
     * Call this periodically from any thread to write to disk.
     */
    public void write() {
        while (true) {
            var val = linesToWrite.pollFirst();
            if (val == null) {
                break;
            }
            writeLine(val);
        }
    }

    public synchronized void flush() {
        if (output != null) {
            write();
            output.flush();
        }
    }
}
