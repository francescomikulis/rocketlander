package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.RLModel.*;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;

import java.io.Serializable;
import java.lang.reflect.Field;

public class StateActionTuple implements Serializable {
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

    // Required data structures.

    private static double angle_precision = Math.PI / 360;
    private static double gimble_precision = Math.PI / 360;

    public static class State implements Serializable {
        double altitude = 0.0;
        double velocity = 0.0;
        double angleX = 0.0;
        double angleZ = 0.0;
        //double angularVelocity = 0.0;
        double gimbleY = 0.0;
        double gimbleZ = 0.0;

        public State(SimulationStatus status) {
            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);
            double xDir = rocketDirection.x;
            double yDir = rocketDirection.y;
            double zDir = rocketDirection.z;
            setAltitude(status.getRocketPosition().z);
            setVelocity(status.getRocketVelocity().z);
            setAngleX(Math.acos(xDir) * Math.signum(yDir));
            setAngleZ(Math.acos(zDir));

            /*
            setAngularVelocity(Math.sqrt(
                Math.pow(status.getRocketRotationVelocity().x, 2)
                + Math.pow(status.getRocketRotationVelocity().y, 2)
                + Math.pow(status.getRocketRotationVelocity().z, 2))
            );
            */
        }

        private void setAltitude(double altitude) {
            this.altitude = group_by_precision(altitude, 0.1);
        }

        private void setVelocity(double velocity) {
            this.velocity = group_by_precision(velocity, 0.1);
        }

        private void setAngleX(double angle) {
            this.angleX = group_by_precision(angle, angle_precision);
        }

        private void setAngleZ(double angle) {
            this.angleZ = group_by_precision(angle, angle_precision);
        }

        public void setGimbleYWithoutRounding(double roundedGimbleY) { this.gimbleY = roundedGimbleY; }

        public void setGimbleZWithoutRounding(double roundedGimbleZ) { this.gimbleZ = roundedGimbleZ; }

        public double getGimbleInRadians(double angle) { return angle * gimble_precision; }

        // private void setAngularVelocity(double angularVelocity) { this.angularVelocity = group_by_precision(angularVelocity, 0.1); }

        @Override
        public String toString() { return stringifyObject(this); }

        private int getDoubleHashCode(Double val) {
            return Double.valueOf(val).hashCode();
        }

        @Override
        public int hashCode() {
            return getDoubleHashCode(altitude) + getDoubleHashCode(velocity) +
                    getDoubleHashCode(special_area_angle_hashcode()) +
                    getDoubleHashCode(gimbleY) + getDoubleHashCode(gimbleZ);
        }

        private Double special_area_angle_hashcode() {
            if (this.angleZ == group_by_precision(Math.PI / 2, angle_precision)) {
                this.angleX = 0.0;  // adapt for this special case.  May be needed when comparing equals code.
                return (double) getDoubleHashCode(this.angleZ);
            }

            return (double) getDoubleHashCode(this.angleX) + getDoubleHashCode(this.angleZ);
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
    }

    public static class Action implements Serializable {
        double thrust = 0.0;
        double gimbleY = 0.0;
        double gimbleZ = 0.0;

        public Action(double thrust, double gimbleY, double gimbleZ) {
            setThrust(thrust);
            setGimbleY(gimbleY);
            setGimbleZ(gimbleZ);
        }

        private void setThrust(double thrust) {
            this.thrust = thrust;
        }

        private void setGimbleY(double gimbleY) {
            this.gimbleY = group_by_precision(gimbleY, gimble_precision);
        }

        private void setGimbleZ(double gimbleZ) { this.gimbleZ = group_by_precision(gimbleZ, gimble_precision); }

        @Override
        public String toString() { return stringifyObject(this); }

        @Override
        public int hashCode() {
            return Double.valueOf(thrust).hashCode() + Double.valueOf(gimbleY).hashCode() + Double.valueOf(gimbleZ).hashCode();
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
    }

    private static int group_by_precision (double value, double precision) {
        return (int) Math.round(value / precision);
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
                double value = (double)field.get(object);
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

