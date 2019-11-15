package net.sf.openrocket.simulation.extension.impl;

import com.sun.javaws.exceptions.InvalidArgumentException;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.RLModel.*;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;

import java.io.Serializable;
import java.lang.reflect.Field;

public class StateActionTuple implements Serializable {
    public static double MIN_THRUST = 0.0;
    public static double MAX_THRUST = 1.0;
    public static double MIN_THRUST_INCREMENT_PER_TIMESTEP = 0.25;
    public static double MAX_THRUST_INCREMENT_PER_TIMESTEP = 1.0;
    public static double MIN_ANGLE_INCREMENT_PER_TIMESTEP = Math.PI / 36;
    public static double MAX_ANGLE_INCREMENT_PER_TIMESTEP = Math.PI / 18;

    public State state;
    public Action action;
    public StateActionTuple(State state, Action action) {
        this.state = state;
        this.action = action;
    }

    @Override
    public String toString() {
        return ("(" + state.toString() + ", " + action.toString() + ")");
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
        int thrust = 0;
        int gimbleY = 0;
        int gimbleZ = 0;

        protected static int group_by_precision(double value, double precision) {
            return (int) Math.round((double)value / (double)precision);
        }

        // Thrust

        public double getThrustDouble() { return ((double)this.thrust * MIN_THRUST_INCREMENT_PER_TIMESTEP); }

        public void setThrust(double thrust) {
            if ((thrust < 0.0) || (thrust > 1.0)) {
                throw new IllegalArgumentException("Invalid Thrust");
            }
            this.thrust = group_by_precision(thrust, MIN_THRUST_INCREMENT_PER_TIMESTEP);
        }

        // GimbleY

        protected void setGimbleY(double gimbleY) {
            this.gimbleY = group_by_precision(gimbleY, MIN_ANGLE_INCREMENT_PER_TIMESTEP);
        }

        public double getGimbleYDouble() {
            // Radians
            return gimbleY * MIN_ANGLE_INCREMENT_PER_TIMESTEP;
        }

        // GimbleZ

        protected void setGimbleZ(double gimbleZ) { this.gimbleZ = group_by_precision(gimbleZ, MIN_ANGLE_INCREMENT_PER_TIMESTEP); }

        public double getGimbleZDouble() {
            // Radians
            return gimbleZ * MIN_ANGLE_INCREMENT_PER_TIMESTEP;
        }
    }

    // Required data structures.

    public static class State extends StateActionClass {
        int altitude = 0;
        int velocity = 0;
        int angleX = 0;
        int angleZ = 0;

        public State(SimulationStatus status) {
            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);
            setAltitude(status.getRocketPosition().z);
            setVelocity(status.getRocketVelocity().z);
            setAngleX(Math.acos(rocketDirection.x) * Math.signum(rocketDirection.y));
            setAngleZ(Math.acos(rocketDirection.z));
        }

        private void setAltitude(double altitude) {
            this.altitude = group_by_precision(altitude, 1);
        }

        private void setVelocity(double velocity) {
            this.velocity = group_by_precision(velocity, 1);
        }

        private void setAngleX(double angle) {
            this.angleX = group_by_precision(angle, MIN_ANGLE_INCREMENT_PER_TIMESTEP);
        }

        private void setAngleZ(double angle) {
            this.angleZ = group_by_precision(angle, MIN_ANGLE_INCREMENT_PER_TIMESTEP);
        }

        @Override
        public int hashCode() {
            return velocity + altitude * (thrust + 1);
        }

        private int special_area_angle_hashcode() {
            if (this.angleZ == group_by_precision(Math.PI / 2, MIN_ANGLE_INCREMENT_PER_TIMESTEP)) {
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
            return altitude == other.altitude && velocity == other.velocity &&
                    angleX == other.angleX && angleZ == other.angleZ &&
                    gimbleY == other.gimbleY && gimbleZ == other.gimbleZ;
        }

        @Override
        public String toString() { return stringifyObject(this); }
    }

    public static class Action extends StateActionClass {
        public Action(double thrust, double gimbleY, double gimbleZ) {
            setThrust(thrust);
            gimbleY = (gimbleY % (2 * Math.PI));
            gimbleZ = (gimbleZ % (2 * Math.PI));
            setGimbleY(gimbleY);
            setGimbleZ(gimbleZ);
        }

        @Override
        public int hashCode() {
            return thrust + gimbleY + gimbleZ;
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
            return thrust == other.thrust && gimbleY == other.gimbleY && gimbleZ == other.gimbleZ;
        }

        @Override
        public String toString() { return stringifyObject(this); }
    }






    public static Quaternion getConjugateQuaternion(Quaternion quaternion) {
        return new Quaternion(quaternion.getW(), -quaternion.getX(), -quaternion.getY(), -quaternion.getZ());
    }

    private static Coordinate convertRocketStatusQuaternionToDirection(SimulationStatus status) {
        Quaternion q = status.getRocketOrientationQuaternion();
        Quaternion p = new Quaternion(0, 0, 0, 1);  // z direction - un-rotated
        Quaternion q_conjugate = getConjugateQuaternion(q);
        Quaternion p_result = q.multiplyRight(p).multiplyRight(q_conjugate);
        // NOTE: This code has been verified extensively.
        return new Coordinate(p_result.getX(), p_result.getY(), p_result.getZ());
    }

    public static String stringifyObject(Object object) {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(object.getClass().getSimpleName());
        stringBuilder.append("(");
        for(Field field : object.getClass().getDeclaredFields()){
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
}

