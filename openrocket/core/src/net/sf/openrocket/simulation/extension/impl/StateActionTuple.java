package net.sf.openrocket.simulation.extension.impl;

import java.io.Serializable;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;


import java.lang.reflect.Field;

import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

public class StateActionTuple implements Serializable {
    public static StateActionTuple baseClass = new StateActionTuple(null, null);

    public static double _1deg = (float)Math.PI / 180;
    public static double _30deg = (float)Math.PI / 6;
    public static double _60deg = (float)Math.PI / 3;
    public static double _45deg = (float)Math.PI / 4;
    public static double _2deg = (float)Math.PI / 90;
    public static double _20deg = (float)Math.PI / 9;
    public static double _0_5deg = (float)Math.PI / 360;
    public static double _15deg = (float)Math.PI / 12;
    public static double _5deg = (float)Math.PI / 36;
    public static double _180deg = (float)Math.PI;
    public static double _90deg = (float)Math.PI / 2;
    public static double _720deg = (float)Math.PI * 4;
    public static double _4deg = (float)Math.PI / 90;

    // altitude, positionX, positionY & velocity
    public static double MIN_ALTITUDE = 0.0f;
    public static double MAX_ALTITUDE = 36.0f;
    public static double ALTITUDE_PRECISION = 1.0f;

    public static double MIN_VELOCITY = -15.0f;
    public static double MAX_VELOCITY = 5.0f;
    public static double VELOCITY_PRECISION = 1.0f;

    // positionX, positionY
    public static double MIN_POSITIONX = -8.0f;
    public static double MIN_POSITIONY = MIN_POSITIONX;
    public static double MAX_POSITIONX = 8.0f;
    public static double MAX_POSITIONY = MAX_POSITIONX;
    public static double POSITIONX_PRECISION = 100.0f;
    public static double POSITIONY_PRECISION = 100.0f;

    // thurst
    public static double MIN_THRUST = 0.0f;
    public static double MAX_THRUST = 1.0f;
    public static double THRUST_PRECISION = 0.25f;  // TODO: RESTORE TO 0.25!!!
    public static double MAX_THRUST_INCREMENT_PER_TIMESTEP = 1.0f;  // TODO: RESTORE TO 1.0!!!

    // angleX
    public static double MIN_ANGLEX = -_720deg;
    public static double MAX_ANGLEX =_720deg;
    public static double ANGLEX_PRECISION = _45deg;  // TODO: RESTORE TO _45deg!!!
    // angleZ
    public static double MIN_ANGLEZ = 0.0f; // 0deg
    public static double MAX_ANGLEZ = _30deg; // 30deg
    public static double ANGLEZ_PRECISION = _2deg;  // TODO: RESTORE TO _2deg!!!

    // gimble angles
    public static double MIN_GIMBLEY = -_720deg;
    public static double MAX_GIMBLEY = _720deg;
    public static double MIN_GIMBLEZ = 0.0f;
    public static double MAX_GIMBLEZ = _5deg;  // TODO: RESTORE TO _15deg!!!
    public static double MAX_HALF_CIRCLE = _180deg;

    public static double GIMBLEY_PRECISION = _45deg;  // TODO: RESTORE TO 45!!!
    //public static float MAX_GIMBLEY_INCREMENT = _45deg;
    public static double MAX_GIMBLEY_INCREMENT = _180deg;
    public static double GIMBLEZ_PRECISION = _1deg; // TODO: RESTORE TO 1!!!
    //public static float MAX_GIMBLE_Z_INCREMENT_PER_TIMESTEP = _2deg;
    public static double MAX_GIMBLEZ_INCREMENT = MAX_GIMBLEZ;

    public static double TIME_PRECISION = 1.0f;
    public static double MIN_TIME = 0.0f;
    public static double MAX_TIME = 7.0f;

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
        public int thrust = 0;
        public int gimbleY = 0;
        public int gimbleZ = 0;

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






        public int getInt(String field) {
            return (int)getDynamicField(this, field);
        }

        public Object get(String field) {
            return get(this, field);
        }

        public static Object get(StateActionClass object, String field) {
            return getDynamicField(object, field);
        }

        public double getDouble(String field) {
            return getDouble(this, field);
        }

        public static double getDouble(StateActionClass object, String field) {
            String precisionField = field.toUpperCase() + "_PRECISION";
            return object.getInt(field) * getConstant(precisionField);
        }

        public StateActionClass set(String field, Object value) {
            return (StateActionClass)setDynamicField(this, field, value);
        }

        public StateActionClass setDouble(String field, double value) {
            return setDouble(this, field, value);
        }

        public static StateActionClass setDouble(StateActionClass object, String field, double value) {
            int realValue = 0;
            String precisionField = field.toUpperCase() + "_PRECISION";
            if (precisionField.contains("GIMBLE") || precisionField.contains("ANGLE"))
                realValue = group_angle_by_precision(value, getConstant(precisionField));
            else if (precisionField.contains("POSITION"))
                realValue = group_by_precision_symmetric(value, getConstant(precisionField));
            else
                realValue = group_by_precision(value, getConstant(precisionField));
            return object.set(field, realValue);
        }
    }

    // Required data structures.

    public static class State extends StateActionClass {
        public int altitude = 0;
        public int positionX = 0;
        public int positionY = 0;
        public int velocity = 0;
        public int angleX = 0;
        public int angleZ = 0;
        public int time = 0;

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
            return velocity * 1000000 + altitude * 100000 + thrust * 10000 + angleX * 100 + angleZ;
        }

        private int special_area_angle_hashcode() {
            if (this.angleZ == group_angle_by_precision(Math.PI / 2, ANGLEZ_PRECISION)) {
                this.angleX = 0;  // adapt for this special case.  May be needed when comparing equals code.
                return this.angleZ;
            }
            return this.angleX * this.angleZ;
        }

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
        public Action(double thrust, double gimbleY, double gimbleZ) {
            setDouble("thrust", thrust);
            setDouble("gimbleY", gimbleY);
            setDouble("gimbleZ", gimbleZ);
        }

        @Override
        public int hashCode() {
            return thrust * 10000 + gimbleY * 100 + gimbleZ;
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

    public static String stringifyObject(Object object) {
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
    }

    public static Object getDynamicField(Object object, String field){
        if (object == null) return null;
        Object result = null;
        Class<?> c = object.getClass();
        try {
            Field f = c.getDeclaredField(field);
            f.setAccessible(true);
            result = (Object) f.get(object);
        } catch (Exception e) {
            try {
                Field f = c.getField(field);
                f.setAccessible(true);
                result = (Object) f.get(object);
            } catch (NoSuchFieldException | IllegalAccessException ex) {
                System.out.println(ex.getMessage());
                ex.printStackTrace();
            }
        }
        return result;
    }

    public static double getConstant(String field){
        return (double)getDynamicField(baseClass, field);
    }

    public static Object setDynamicField(Object object, String field, Object value){
        if (object == null) return null;
        Class<?> c = object.getClass();
        try {
            Field f = c.getDeclaredField(field);
            f.setAccessible(true);
            f.set(object, value);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            try {
                Field f = c.getField(field);
                f.setAccessible(true);
                f.set(object, value);
            } catch (NoSuchFieldException | IllegalAccessException ex) {
                System.out.println(ex.getMessage());
                ex.printStackTrace();
            }
        }
        return object;
    }

}

