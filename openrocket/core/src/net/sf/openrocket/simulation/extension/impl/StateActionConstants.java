package net.sf.openrocket.simulation.extension.impl;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.HashSet;

public class StateActionConstants {
    public static final double _1deg = (float)Math.PI / 180;
    public static final double _30deg = (float)Math.PI / 6;
    public static final double _60deg = (float)Math.PI / 3;
    public static final double _45deg = (float)Math.PI / 4;
    public static final double _2deg = (float)Math.PI / 90;
    public static final double _20deg = (float)Math.PI / 9;
    public static final double _0_5deg = (float)Math.PI / 360;
    public static final double _15deg = (float)Math.PI / 12;
    public static final double _5deg = (float)Math.PI / 36;
    public static final double _180deg = (float)Math.PI;
    public static final double _90deg = (float)Math.PI / 2;
    public static final double _720deg = (float)Math.PI * 4;
    public static final double _4deg = (float)Math.PI / 90;

    // altitude, positionX, positionY & velocity
    public static final double MIN_ALTITUDE = 0.0f;
    public static final double MAX_ALTITUDE = 36.0f;
    public static final double ALTITUDE_PRECISION = 1.0f;

    public static final double MIN_VELOCITY = -15.0f;
    public static final double MAX_VELOCITY = 5.0f;
    public static final double VELOCITY_PRECISION = 1.0f;

    // positionX, positionY
    public static final double MIN_POSITIONX = -8.0f;
    public static final double MIN_POSITIONY = MIN_POSITIONX;
    public static final double MAX_POSITIONX = 8.0f;
    public static final double MAX_POSITIONY = MAX_POSITIONX;
    public static final double POSITIONX_PRECISION = 100.0f;
    public static final double POSITIONY_PRECISION = 100.0f;

    // thurst
    public static final double MIN_THRUST = 0.0f;
    public static final double MAX_THRUST = 1.0f;
    public static final double THRUST_PRECISION = 0.25f;  // TODO: RESTORE TO 0.25!!!
    public static final double MAX_THRUST_INCREMENT_PER_TIMESTEP = 1.0f;  // TODO: RESTORE TO 1.0!!!

    // angleX
    public static final double MIN_ANGLEX = -_720deg;
    public static final double MAX_ANGLEX =_720deg;
    public static final double ANGLEX_PRECISION = _45deg;  // TODO: RESTORE TO _45deg!!!
    // angleZ
    public static final double MIN_ANGLEZ = 0.0f; // 0deg
    public static final double MAX_ANGLEZ = _30deg; // 30deg
    public static final double ANGLEZ_PRECISION = _2deg;  // TODO: RESTORE TO _2deg!!!

    // gimble angles
    public static final double MIN_GIMBLEY = -_720deg;
    public static final double MAX_GIMBLEY = _720deg;
    public static final double MIN_GIMBLEZ = 0.0f;
    public static final double MAX_GIMBLEZ = _5deg;  // TODO: RESTORE TO _15deg!!!
    public static final double MAX_HALF_CIRCLE = _180deg;

    public static final double GIMBLEY_PRECISION = _45deg;  // TODO: RESTORE TO 45!!!
    //public static float MAX_GIMBLEY_INCREMENT = _45deg;
    public static final double MAX_GIMBLEY_INCREMENT = _180deg;
    public static final double GIMBLEZ_PRECISION = _1deg; // TODO: RESTORE TO 1!!!
    //public static float MAX_GIMBLE_Z_INCREMENT_PER_TIMESTEP = _2deg;
    public static final double MAX_GIMBLEZ_INCREMENT = MAX_GIMBLEZ;

    public static final double TIME_PRECISION = 1.0f;
    public static final double MIN_TIME = 0.0f;
    public static final double MAX_TIME = 7.0f;

    private static volatile StateActionConstants instance;
    private static volatile HashMap<String, Double> constants;
    private static volatile HashMap<String, Double> precisionConstants;
    private static volatile HashMap<String, Field> stateActionTupleFields;


    private StateActionConstants(){
        constants = getAllDynamicFieldsDouble(this);
        precisionConstants = getAllDynamicFieldsDouble(this, "_PRECISION");
        HashMap<String, Field> a = getAllDynamicFieldsField(new StateActionTuple(null, null));
        HashMap<String, Field> b = getAllDynamicFieldsField(new StateActionTuple.State(null));
        HashMap<String, Field> c = getAllDynamicFieldsField(new StateActionTuple.Action(null));
        a.putAll(b);
        a.putAll(c);
        stateActionTupleFields = a;
    }

    public static StateActionConstants getInstance() {
        if (instance == null) { // first time lock
            synchronized (StateActionConstants.class) {
                if (instance == null) {  // second time lock
                    instance = new StateActionConstants();
                }
            }
        }
        return instance;
    }

    public static double getConstant(String field){
        getInstance();
        return constants.get(field);
    }

    public static double getPrecisionConstant(String field){
        getInstance();
        return precisionConstants.get(field);
    }

    public static Field getField(String field) {
        getInstance();
        return stateActionTupleFields.get(field);
    }

    public static HashMap<String, Double> getAllDynamicFieldsDouble(Object object){
        if (object == null) return null;
        HashMap<String, Double> dynamicFields = new HashMap<>();
        Class<?> c = object.getClass();
        for(Field field: c.getFields()){
            String fieldName = field.getName();
            try {
                dynamicFields.put(fieldName, (double) field.get(object));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        return dynamicFields;
    }

    public static HashMap<String, Double> getAllDynamicFieldsDouble(Object object, String ignoreStringText){
        if (object == null) return null;
        HashMap<String, Double> dynamicFields = new HashMap<>();
        Class<?> c = object.getClass();
        for(Field field: c.getFields()){
            String fieldName = field.getName();
            if (ignoreStringText.isEmpty()) {
                try { dynamicFields.put(fieldName, (double) field.get(object)); } catch (IllegalAccessException e) {  e.printStackTrace(); }
            } else {
                fieldName = fieldName.replace(ignoreStringText, "").toLowerCase();
                try { dynamicFields.put(fieldName, (double) field.get(object)); } catch (IllegalAccessException e) {  e.printStackTrace(); }
                int len = fieldName.length();
                fieldName = fieldName.substring(0, len - 1) + fieldName.substring(len - 1, len).toUpperCase();
                try { dynamicFields.put(fieldName, (double) field.get(object)); } catch (IllegalAccessException e) {  e.printStackTrace(); }
            }
        }
        return dynamicFields;
    }

    public static HashMap<String, Field> getAllDynamicFieldsField(Object object){
        if (object == null) return null;
        HashMap<String, Field> dynamicFields = new HashMap<>();
        Class<?> c = object.getClass();
        for (Field f: c.getDeclaredFields()) {
            f.setAccessible(true);
            dynamicFields.put(f.getName(), f);
        }
        for (Field f: c.getFields()) {
            f.setAccessible(true);
            dynamicFields.put(f.getName(), f);
        }
        return dynamicFields;
    }
}

