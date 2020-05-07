package net.sf.openrocket.simulation.extension.impl;

import java.util.*;
import java.util.concurrent.locks.ReentrantLock;

public class ValueFunction {
    private static final long MAX_TABLE_SIZE = 10 * 1000 * 1000;  // 10 milion
    public static final ReentrantLock mainLock = new ReentrantLock();
    public transient ReentrantLock[] locks = null;

    public boolean isTable;
    private float[] valueFunctionTable = null;
    private HashMap<Integer, Float> valueFunctionMap = null;

    public static boolean isReasonableTableSize(int size) {
        return (size >= 0) && (size <= MAX_TABLE_SIZE);
    }

    public ValueFunction(float[] valueFunctionTable) {
        isTable = true;
        this.valueFunctionTable = valueFunctionTable;
        if (valueFunctionTable == null) {
            locks = null;
            return;
        }
        locks = new ReentrantLock[valueFunctionTable.length];
        for (int i = 0; i < valueFunctionTable.length; i++) {
            locks[i] = new ReentrantLock();
        }
        checkTableValues();
    }

    private void checkTableValues() {
        for (int i = 0; i < valueFunctionTable.length; i++) {
            if (Float.isNaN(valueFunctionTable[i])) {
                System.out.println("NAN IN TABLE!");
            }
        }
    }

    public ValueFunction(HashMap<Integer, Float> valueFunctionMap) {
        isTable = false;
        this.valueFunctionMap = valueFunctionMap;
    }

    public float[] getValueFunctionTable() {
        if (isTable) return valueFunctionTable;
        else return null;
    }

    public HashMap<Integer, Float> getValueFunctionMap() {
        if (!isTable) return valueFunctionMap;
        else return null;
    }

    public float get(int index) {
        if (isTable) {
            return valueFunctionTable[index];
        } else {
            if (!valueFunctionMap.containsKey(index)) return 0.0f;
            return valueFunctionMap.get(index);
        }
    }

    public void put(int index, float value) {
        if (isTable) {
            valueFunctionTable[index] = value;
        } else {
            valueFunctionMap.put(index, value);
        }
    }

    public void lock(int index) {
        if (locks != null) {
            locks[index].lock();
        }
    }

    public void unlock(int index) {
        if (locks != null) {
            locks[index].unlock();
        }
    }

    public void lockMainLock() {
        mainLock.lock();
    }

    public void unlockMainLock() {
        mainLock.unlock();
    }
}
