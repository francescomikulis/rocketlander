package net.sf.openrocket.simulation.extension.impl;

import java.io.Serializable;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.RLModel.*;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;


import java.lang.reflect.Field;

public class StateActionTuple implements Serializable {
    public static float _1deg = (float)Math.PI / 180;
    public static float _30deg = (float)Math.PI / 6;
    public static float _45deg = (float)Math.PI / 4;
    public static float _2deg = (float)Math.PI / 90;
    public static float _20deg = (float)Math.PI / 9;
    public static float _0_5deg = (float)Math.PI / 360;
    public static float _15deg = (float)Math.PI / 12;
    public static float _5deg = (float)Math.PI / 36;
    public static float _180deg = (float)Math.PI;
    public static float _90deg = (float)Math.PI / 2;
    public static float _720deg = (float)Math.PI * 4;
    public static float _4deg = (float)Math.PI / 90;

    // altitude & velocity
    public static float MIN_ALTITUDE = 0.0f;
    public static float MAX_ALTITUDE = 56.0f;
    public static float ALTITUDE_PRECISION = 1.0f;
    public static float MIN_VELOCITY = -35.0f;
    public static float MAX_VELOCITY = 5.0f;
    public static float VELOCITY_PRECISION = 1.0f;
    // thurst
    public static float MIN_THRUST = 0.0f;
    public static float MAX_THRUST = 1.0f;
    public static float MIN_THRUST_INCREMENT_PER_TIMESTEP = 0.5f;  // TODO: RESTORE TO 0.25!!!
    public static float MAX_THRUST_INCREMENT_PER_TIMESTEP = 1.0f;  // TODO: RESTORE TO 1.0!!!
    // orientation
    public static float MIN_TERMINAL_ORIENTATION_Z = 0.0f; // 30deg
    public static float MAX_TERMINAL_ORIENTATION_Z = _30deg; // 30deg
    // orientation angles
    public static float ANGLE_X_PRECISION = _45deg;  // TODO: RESTORE TO _45deg!!!
    public static float ANGLE_Z_PRECISION = _2deg;  // TODO: RESTORE TO _2deg!!!
    // gimble angles
    public static float MIN_GIMBLE_Y = -_720deg;
    public static float MAX_GIMBLE_Y = _720deg;
    public static float MIN_GIMBLE_Z = 0.0f;
    public static float MAX_GIMBLE_Z = _5deg;  // TODO: RESTORE TO _15deg!!!
    public static float MAX_HALF_CIRCLE = _180deg;

    public static float MIN_GIMBLE_Y_INCREMENT_PER_TIMESTEP = _45deg;  // TODO: RESTORE TO 45!!!
    //public static float MAX_GIMBLE_Y_INCREMENT_PER_TIMESTEP = _45deg;
    public static float MAX_GIMBLE_Y_INCREMENT_PER_TIMESTEP = _180deg;
    public static float MIN_GIMBLE_Z_INCREMENT_PER_TIMESTEP = _0_5deg; // TODO: RESTORE TO 1!!!
    //public static float MAX_GIMBLE_Z_INCREMENT_PER_TIMESTEP = _2deg;
    public static float MAX_GIMBLE_Z_INCREMENT_PER_TIMESTEP = MAX_GIMBLE_Z;

    public static float TIME_PRECISION = 1.0f;
    public static float MIN_TIME = 0.0f;
    public static float MAX_TIME = 7.0f;

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
        public int thrust = 0;
        public int gimbleY = 0;
        public int gimbleZ = 0;

        protected static int group_by_precision(double value, double precision) {
            return (int) Math.floor((double)value / (double)precision);
        }

        // Thrust

        public double getThrustDouble() { return ((double)this.thrust * MIN_THRUST_INCREMENT_PER_TIMESTEP); }

        public StateActionClass setThrust(double thrust) {
            if ((thrust < MIN_THRUST) || (thrust > MAX_THRUST + 0.0001)) {
                throw new IllegalArgumentException("Invalid Thrust");
            }
            this.thrust = group_by_precision(thrust, MIN_THRUST_INCREMENT_PER_TIMESTEP);
            return this;
        }

        protected int group_angle_by_precision(double angle, double precision) {
            double shiftedAngle = ((angle + (2 * Math.PI)) % (2 * Math.PI));
            return group_by_precision(shiftedAngle, precision);
        }

        // GimbleY

        protected StateActionClass setGimbleY(double gimbleY) {
            this.gimbleY = group_angle_by_precision(gimbleY, MIN_GIMBLE_Y_INCREMENT_PER_TIMESTEP);
            return this;
        }

        public double getGimbleYDouble() {
            // Radians
            return gimbleY * MIN_GIMBLE_Y_INCREMENT_PER_TIMESTEP;
        }

        // GimbleZ

        protected StateActionClass setGimbleZ(double gimbleZ) {
            this.gimbleZ = group_angle_by_precision(gimbleZ, MIN_GIMBLE_Z_INCREMENT_PER_TIMESTEP);
            return this;
        }

        public double getGimbleZDouble() {
            // Radians
            return gimbleZ * MIN_GIMBLE_Z_INCREMENT_PER_TIMESTEP;
        }
    }

    // Required data structures.

    public static class State extends StateActionClass {
        public int altitude = 0;
        public int velocity = 0;
        public int angleX = 0;
        public int angleZ = 0;
        public int time = 0;

        public State(SimulationStatus status) {
            if (status == null) return;
            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);
            setAltitude(status.getRocketPosition().z);
            setVelocity(status.getRocketVelocity().z);
            setAngleX(Math.acos(rocketDirection.x) * Math.signum(rocketDirection.y));
            setAngleZ(Math.acos(rocketDirection.z));
            setTime(status.getSimulationTime());
            /*
            // OpenRocket traditional approach - not working

            // Vertical orientation (zenith)
            double theta = Math.atan2(rocketDirection.z, MathUtil.hypot(rocketDirection.x, rocketDirection.y)) - Math.PI /2;  // offset vertical to 0 deg
            // Lateral orientation (azimuth)
            double phi = Math.atan2(rocketDirection.y, rocketDirection.x);
            if (phi < -(Math.PI - 0.0001))
                phi = Math.PI;
            setAngleX(phi);
            setAngleZ(theta);
             */
        }

        public State setAltitude(double altitude) {
            this.altitude = group_by_precision(altitude, ALTITUDE_PRECISION);
            return this;
        }

        public double getAltitudeDouble() {
            return altitude * ALTITUDE_PRECISION;
        }

        public State setVelocity(double velocity) {
            this.velocity = group_by_precision(velocity, VELOCITY_PRECISION);
            return this;
        }

        public double getVelocityDouble() {
            return velocity * VELOCITY_PRECISION;
        }

        public State setAngleX(double angle) {
            this.angleX = group_angle_by_precision(angle, ANGLE_X_PRECISION);
            return this;
        }

        public double getAngleXDouble() {
            return angleX * ANGLE_X_PRECISION;
        }

        public State setAngleZ(double angle) {
            this.angleZ = group_angle_by_precision(angle, ANGLE_Z_PRECISION);
            return this;
        }

        public double getAngleZDouble() {
            return angleZ * ANGLE_Z_PRECISION;
        }

        public State setTime(double time) {
            this.time = group_by_precision(time, TIME_PRECISION);
            return this;
        }

        public double getTimeDouble() {
            return time * TIME_PRECISION;
        }

        @Override
        public int hashCode() {
            return velocity * 100000 + altitude * 10000 + thrust * 100 + angleX * 10 + angleZ;
        }

        private int special_area_angle_hashcode() {
            if (this.angleZ == group_angle_by_precision(Math.PI / 2, ANGLE_Z_PRECISION)) {
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
                    gimbleY == other.gimbleY && gimbleZ == other.gimbleZ && time == other.time;
        }

        @Override
        public String toString() { return stringifyObject(this); }
    }

    public static class Action extends StateActionClass {
        public Action(double thrust, double gimbleY, double gimbleZ) {
            setThrust(thrust);
            setGimbleY(gimbleY);
            setGimbleZ(gimbleZ);
        }

        @Override
        public int hashCode() {
            return thrust * 100 + gimbleY * 10 + gimbleZ;
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

    public static Coordinate OLDconvertRocketStatusQuaternionToDirection(SimulationStatus status) {
        Quaternion q = status.getRocketOrientationQuaternion();
        Quaternion p = new Quaternion(0, 0, 0, 1);  // z direction - un-rotated
        Quaternion q_conjugate = getConjugateQuaternion(q);
        Quaternion p_result = q.multiplyRight(p).multiplyRight(q_conjugate);
        // NOTE: This code has been verified extensively.
        // TODO: CHECK THESE / 2 BECAUSE THEY SEEM WRONG.
        return new Coordinate(p_result.getX() / 2, p_result.getY() / 2, p_result.getZ());
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
}

