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
    private static int thrust_precision = 25;

    public static class State implements Serializable {
        int altitude = 0;
        int velocity = 0;
        int angleX = 0;
        int angleZ = 0;
        //double angularVelocity = 0.0;
        int gimbleY = 0;
        int gimbleZ = 0;
        int thrust = 0;

        public State(SimulationStatus status) {
            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);
            double xDir = rocketDirection.x;
            double yDir = rocketDirection.y;
            double zDir = rocketDirection.z;
            setAltitude(status.getRocketPosition().z);
            setVelocity(status.getRocketVelocity().z);
            setAngleX(Math.acos(xDir) * Math.signum(yDir));
            setAngleZ(Math.acos(zDir));
            setAngleX(0);
            setAngleZ(0);

            /*
            setAngularVelocity(Math.sqrt(
                Math.pow(status.getRocketRotationVelocity().x, 2)
                + Math.pow(status.getRocketRotationVelocity().y, 2)
                + Math.pow(status.getRocketRotationVelocity().z, 2))
            );
            */
        }

        private void setAltitude(double altitude) {
            this.altitude = group_by_precision(altitude, 1);
        }

        private void setVelocity(double velocity) {
            this.velocity = group_by_precision(velocity, 1);
        }

        private void setAngleX(double angle) {
            this.angleX = group_by_precision(angle, angle_precision);
        }

        private void setAngleZ(double angle) {
            this.angleZ = group_by_precision(angle, angle_precision);
        }

        public double getGimbleInRadians(double angle) { return angle * gimble_precision; }

        public void setThrust(double thrust) {
            assert (thrust >= 0.0);
            assert (thrust <= 1.0);
            this.thrust = group_by_precision(thrust, thrust_precision);
        }

        // private void setAngularVelocity(double angularVelocity) { this.angularVelocity = group_by_precision(angularVelocity, 0.1); }

        @Override
        public String toString() { return stringifyObject(this); }

        @Override
        public int hashCode() {
            return velocity + altitude * (thrust + 1);
        }

        private int special_area_angle_hashcode() {
            if (this.angleZ == group_by_precision(Math.PI / 2, angle_precision)) {
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
    }

    public static class Action implements Serializable {
        int thrust = 0;
        int gimbleY = 0;
        int gimbleZ = 0;

        public Action(double thrust, double gimbleY, double gimbleZ) {
            setThrust(thrust);
            gimbleY = gimbleY % (2 * Math.PI) - Math.PI;
            gimbleZ = gimbleZ % (2 * Math.PI) - Math.PI;
            setGimbleY(gimbleY);
            setGimbleZ(gimbleZ);
            setGimbleY(0.0);
            setGimbleZ(0.0);
        }

        private void setThrust(double thrust) {
            assert (thrust >= 0.0);
            assert (thrust <= 1.0);
            this.thrust = group_by_precision(thrust, thrust_precision);
        }

        private void setGimbleY(double gimbleY) {
            this.gimbleY = group_by_precision(gimbleY, gimble_precision);
        }

        public double getGimbleYInRadians() {
            return gimbleY * gimble_precision;
        }

        private void setGimbleZ(double gimbleZ) { this.gimbleZ = group_by_precision(gimbleZ, gimble_precision); }

        public double getGimbleZInRadians() {
            return gimbleZ * gimble_precision;
        }

        public double getThrustDouble() {
            return ((double)this.thrust) / 100.0;
        }

        @Override
        public String toString() { return stringifyObject(this); }

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

