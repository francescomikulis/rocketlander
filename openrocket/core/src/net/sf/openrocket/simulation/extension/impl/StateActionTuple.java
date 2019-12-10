package net.sf.openrocket.simulation.extension.impl;

import java.io.Serializable;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;


import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import static net.sf.openrocket.simulation.extension.impl.StateActionConstants.*;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

public class StateActionTuple implements Serializable {
    public static StateActionTuple baseClass = new StateActionTuple(null, null);
    public State state;
    public Action action;
    public StateActionTuple(State state, Action action) {
        this.state = state;
        this.action = action;
    }

    @Override
    public String toString() {
        String stateString = (state != null) ? state.toString() : "";
        String actionString = (action != null) ? action.toString() : "";
        return ("(" + stateString + ", " + actionString + ")");
    }

    @Override
    public int hashCode() {
        return state.hashCode() + action.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        StateActionTuple other = (StateActionTuple) obj;
        return this.action.equals(other.action) && this.state.equals(other.state);
    }

    public static class StateActionClass implements Serializable {
        public HashMap<String, Integer> valueMap = new HashMap<>();

        protected static int group_by_precision(double value, double precision) {
            return (int) Math.floor((double)value / (double)precision);
        }

        /**
         * e.g. symmetric values around 0 [-10, 10]
         *
         * Without symmetric grouping:
         *  (-10, -5) --> -2
         *  ( -5, -0) --> -1
         *  ( 0,   5) --> 0
         *  ( 5,  10) --> 1
         *
         * With symmetric grouping [if (result < 0) result += 1;]
         *  (-10, -5) --> -2 + 1 = -1
         *  ( -5, -0) --> -1 + 1 = 0
         *  ( 0,   5) --> 0
         *  ( 5,  10) --> 1
         */
        protected static int group_by_precision_symmetric(double value, double precision) {
            int result = (int) Math.floor((double)value / (double)precision);
            if (result < 0)
                result += 1;
            return result;
        }

        protected static int group_angle_by_precision(double angle, double precision) {
            if (angle == 2 * Math.PI) angle -= 0.00001;  // fall into right sector
            double shiftedAngle = ((angle + (4 * Math.PI)) % (2 * Math.PI));
            return group_by_precision(shiftedAngle, precision);
        }





        public int get(String field) {
            Object result = valueMap.get(field);
            if (result == null) return 0;
            return (int)result;
        }

        public static int get(StateActionClass object, String field) {
            return object.get(field);
        }

        public double getDouble(String field) {
            return get(field) * getPrecisionConstant(field);
        }

        public static double getDouble(StateActionClass object, String field) {
            return object.get(field) * getPrecisionConstant(field);
        }

        public StateActionClass set(String field, int value) {
            valueMap.put(field, value);
            return this;
        }

        public static StateActionClass set(StateActionClass object, String field, int value){
            object.valueMap.put(field, value);
            return object;
        }

        public StateActionClass setDouble(String field, double value) {
            int realValue = 0;
            if (field.contains("gimble") || field.contains("angle"))
                realValue = group_angle_by_precision(value, getPrecisionConstant(field));
            else if (field.contains("position"))
                realValue = group_by_precision_symmetric(value, getPrecisionConstant(field));
            else
                realValue = group_by_precision(value, getPrecisionConstant(field));
            valueMap.put(field, realValue);
            return this;
        }
    }

    // Required data structures.

    public static class State extends StateActionClass {
        public State(SimulationStatus status) {
            if (status == null) return;
            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);
            setDouble("altitude", status.getRocketPosition().z);
            setDouble("positionX", status.getRocketPosition().x);
            setDouble("positionY", status.getRocketPosition().y);
            setDouble("velocity", status.getRocketVelocity().z);
            //setVelocity(Math.signum(status.getRocketVelocity().z) * status.getRocketVelocity().length());

            setDouble("angleX", Math.acos(rocketDirection.x) * Math.signum(rocketDirection.y));
            setDouble("angleZ", Math.acos(rocketDirection.z));
            setDouble("time", status.getSimulationTime());
        }


        @Override
        public int hashCode() {
            return get("velocity") * 1000000 + get("altitude") * 100000 + get("thrust") * 10000 + get("angleX") * 100 + get("angleZ");
        }
        /*

        private int special_area_angle_hashcode() {
            if (this.angleZ == group_angle_by_precision(Math.PI / 2, StateActionConstants.ANGLEZ_PRECISION)) {
                this.angleX = 0;  // adapt for this special case.  May be needed when comparing equals code.
                return this.angleZ;
            }
            return this.angleX * this.angleZ;
        }
         */

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            State other = (State) obj;
            boolean equivalent = true;
            for (String stateField: stateDefinition) {
                equivalent = equivalent && this.get(stateField) == other.get(stateField);
            }
            return equivalent;
        }

        @Override
        public String toString() { return stringifyObject(this); }
    }

    public static class Action extends StateActionClass {
        public Action(Object nullObj){
            if (nullObj != null) System.out.println("FAILURE!");
        }
        public Action(double thrust, double gimbleY, double gimbleZ) {
            setDouble("thrust", thrust);
            setDouble("gimbleY", gimbleY);
            setDouble("gimbleZ", gimbleZ);
        }

        @Override
        public int hashCode() {
            return get("thrust") * 10000 + get("gimbleY") * 100 + get("gimbleZ");
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            Action other = (Action) obj;
            boolean equivalent = true;
            for (String stateField: actionDefinition) {
                equivalent = equivalent && this.get(stateField) == other.get(stateField);
            }
            return equivalent;
        }

        @Override
        public String toString() { return stringifyObject(this); }
    }


    public static Quaternion getConjugateQuaternion(Quaternion quaternion) {
        return new Quaternion(quaternion.getW(), -quaternion.getX(), -quaternion.getY(), -quaternion.getZ());
    }

    public static Coordinate convertRocketStatusQuaternionToDirection(SimulationStatus status) {
        return status.getRocketOrientationQuaternion().rotateZ();
    }

    public static String stringifyObject(StateActionClass object) {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(object.getClass().getSimpleName());
        stringBuilder.append("(");
        for (Map.Entry<String, Integer> entry: object.valueMap.entrySet()) {
            stringBuilder.append(entry.getKey());
            stringBuilder.append(": ");
            stringBuilder.append(entry.getValue());
            stringBuilder.append(", ");
        }
        stringBuilder.replace(stringBuilder.length() - 2, stringBuilder.length(), "");
        stringBuilder.append(")");
        return stringBuilder.toString();
        /*

        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(object.getClass().getSimpleName());
        stringBuilder.append("(");
        for(Field field : object.getClass().getFields()){
            try {
                String fieldName = field.getName();
                int value = (int)field.get(object);
                stringBuilder.append(fieldName);
                stringBuilder.append(": ");
                stringBuilder.append(value);
                stringBuilder.append(", ");
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        stringBuilder.replace(stringBuilder.length() - 2, stringBuilder.length(), "");
        stringBuilder.append(")");
        return stringBuilder.toString();
         */
    }

    public static Object getDynamicField(Object object, String stringField){
        return interactDynamicField(false, object, stringField, 0.0);
    }

    public static Object setDynamicField(Object object, String stringField, Object value){
        return interactDynamicField(true, object, stringField, value);
    }

    public static Object interactDynamicField(boolean needsSet, Object object, String stringField, Object value){
        if (object == null) return null;
        Class<?> c = object.getClass();
        Field field = getField(stringField);
        if (field != null) {
            try{
                if (!needsSet) return field.get(object);
                else field.set(object, value);

                return object;
            } catch (IllegalAccessException e) {
                System.out.println(e.getMessage());
                e.printStackTrace();
            }
        }
        try {
            throw new NoSuchFieldException("EXCEPTION!!");
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        }
        return object;

    }
}

