package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Quaternion;

import java.io.Serializable;
import java.util.*;
import java.util.concurrent.Semaphore;
import java.util.function.BiFunction;
import java.util.function.Function;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;
    private HashMap<StateActionTuple, Double> valueFunctionTable = null;
    private static boolean ONLY_GREEDY = true;
    private static double MOTOR_INCREMENT_PER_TIMESTEP = 0.25;
    private static double THRUST_TOGGLE_PENALTY = 0.1;
    private static double THRUST_ON_PENALTY = -5;

    private Semaphore mutex = new Semaphore(1);

    private static class InstanceHolder {
        private static final RLModel instance = new RLModel();
    }

    public static RLModel getInstance() {
        return InstanceHolder.instance;
    }

    private RLModel(){}

    public void initializeModel() {
        episodeManager = RLEpisodeManager.getInstance();
        episodeManager.safeActionValueFunctionInitialization();
    }

    private ArrayList<Double> generatePossibleThrust(double prevThrust) {
        ArrayList<Double> possibleThurst = new ArrayList<>();
        double CHANGE = MOTOR_INCREMENT_PER_TIMESTEP;
        possibleThurst.add(prevThrust);
        if (prevThrust <= CHANGE) {
            possibleThurst.add(CHANGE);
        } else if (prevThrust >= 1.0 - CHANGE) {
            possibleThurst.add(1.0 - CHANGE);
        } else {
            possibleThurst.add(prevThrust - CHANGE);
            possibleThurst.add(prevThrust + CHANGE);
        }

        // TODO: remove this
        //possibleThurst = new ArrayList<>();
        //possibleThurst.add(0.0);
        return possibleThurst;
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/
    public ArrayList<Action> generatePossibleActions(State state, double prevThurst) {
        ArrayList<Action> possibleActions = new ArrayList<>();

        /*
        ArrayList<Double> possibleThrustValues = generatePossibleThrust(prevThurst);
        for (Double possibleThrust: possibleThrustValues)
            possibleActions.add(new Action(possibleThrust, 0, 0));
        */

        if (state.altitude > 1.0) {
            possibleActions.add(new Action(prevThurst, 0, 0));
            if (prevThurst == 0.0) {
                possibleActions.add(new Action(0.25, 0, 0));
            } else if (prevThurst == 0.25) {
                possibleActions.add(new Action(0.0, 0, 0));
                possibleActions.add(new Action(0.5, 0, 0));
            } else if (prevThurst == 0.5) {
                possibleActions.add(new Action(0.25, 0, 0));
                possibleActions.add(new Action(0.75, 0, 0));
            } else if (prevThurst == 0.75) {
                possibleActions.add(new Action(0.5, 0, 0));
                possibleActions.add(new Action(1.0, 0, 0));
            } else if (prevThurst == 1.00) {
                possibleActions.add(new Action(0.75, 0, 0));
            }
        } else {
            possibleActions.add(new Action(0.0, 0, 0));
            possibleActions.add(new Action(0.25, 0, 0));
            possibleActions.add(new Action(0.5, 0, 0));
            possibleActions.add(new Action(0.75, 0, 0));
        }

        return possibleActions;
    }


    private Double valueFunction(StateActionTuple stateActionTuple) {
        Action action = stateActionTuple.action;
        if (action.thrust > 0) {
            if (stateActionTuple.state.velocity > 0) {
                double penalty = THRUST_ON_PENALTY * action.thrust * 10;
                if (!valueFunctionTable.containsKey(stateActionTuple))
                    return penalty; // penalty
                else
                    return valueFunctionTable.get(stateActionTuple) - Math.abs(penalty);
            }
        }
        // given the state, be a lookup table and return a value for that state-action pair
        /*
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0;
         */
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0;
        return valueFunctionTable.get(stateActionTuple);
    }

    private Double valueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return valueFunction(stateActionTuple);
    }

    public Action run_policy(SimulationStatus status, ArrayList<StateActionTuple> episodeStateAction) {
        double prevThrust = 0.0;
        if (episodeStateAction.size() != 0)
            prevThrust = episodeStateAction.get(episodeStateAction.size() - 1).action.thrust;
        State state = new State(status);
        Action action = policy(state, prevThrust, this::valueFunction);
        episodeManager.addStateActionTuple(state, action, episodeStateAction);
        return action;
    }

    private Action policy(State state, double prevThurst, BiFunction<State, Action, Double> func) {
        ArrayList<Action> possibleActions = generatePossibleActions(state, prevThurst);

        double val = Double.NEGATIVE_INFINITY;
        ArrayList<Action> bestActions = new ArrayList<>();

        boolean greedy = ONLY_GREEDY;
        if (randomGenerator.nextDouble() <= 0.05) {
            greedy = false;  // false
        }

        for (Action action: possibleActions) {
            double v = func.apply(state, action);
            if (greedy) {
                if (v > val) {
                    // value is best compared to all previous encounters.  Reset bestAction ArrayList.
                    val = v;
                    bestActions = new ArrayList<>();
                    bestActions.add(action);
                } else if (v == val) {
                    // value is equal to other best value.  Add to bestAction ArrayList.
                    bestActions.add(action);
                }
            } else {
                bestActions.add(action);
            }
        }
        // ties broken completely at random
        return bestActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStateActionValueFuncton(ArrayList<StateActionTuple> stateActionTuples) {
        try {
            mutex.acquire();
            actuallyUpdateStateActionValueFuncton(stateActionTuples);
        } catch (InterruptedException e) {
            // exception handling code
        } finally {
            mutex.release();
        }
    }

    private void actuallyUpdateStateActionValueFuncton(ArrayList<StateActionTuple> stateActionTuples) {
        // todo: Change this reference to the stateactiontuples
        int maxTimeStep = stateActionTuples.size();

        StateActionTuple lastStateActionTuple = stateActionTuples.get(maxTimeStep - 1);
        double lowSpeedLandingBonus = 0.0;
        if (Math.abs(lastStateActionTuple.state.velocity) <= 2.0) {
            lowSpeedLandingBonus = 50;
            if (Math.abs(lastStateActionTuple.state.velocity) <= 1.0)
                lowSpeedLandingBonus = 100;
        }

        double altitudeMultiplier = lastStateActionTuple.state.altitude;
        if (altitudeMultiplier < 1) {
            altitudeMultiplier = 1;
        } else {
            lowSpeedLandingBonus = 0;
        }

        double landingVelocity = Math.abs(lastStateActionTuple.state.velocity);
        // this edge case will tell the system it was doing well because the multiplication wasn't penalizing enough
        if ((landingVelocity < 100) && (altitudeMultiplier != 1)) {
            landingVelocity = 100;
        }
        //System.out.println(landingVelocity + " " + altitudeMultiplier + " " + lowSpeedLandingBonus);
        double G = -(landingVelocity * altitudeMultiplier) - altitudeMultiplier + lowSpeedLandingBonus;  // landing velocity
        //System.out.println("G: " + G);
        // System.out.println("Starting with penalty: " + G);
        double discount = 0.999;
        double alpha = 0.3;
        double reward = -0.1;

        for (int i = maxTimeStep - 1; i >= 0; i--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(i);
            double stateActionValue = valueFunction(stateActionTuple);
            double positiveVelocityPenalty = 0;
            double thrustTogglePenalty = 0;

            if (i + 1 <= maxTimeStep - 1) {
                // penalize thrust changes by the variation
                double currentThrust = Math.abs(stateActionTuple.action.thrust);
                double nextThrust = Math.abs(stateActionTuples.get(i + 1).action.thrust);
                double absDelta = Math.abs(currentThrust - nextThrust);
                thrustTogglePenalty = THRUST_TOGGLE_PENALTY * absDelta;
            }
            if (stateActionTuple.state.velocity > 0)
                positiveVelocityPenalty = stateActionTuple.state.velocity / 10;
            valueFunctionTable.put(stateActionTuple, stateActionValue + alpha * (G - stateActionValue));
            G = (discount * G) - Math.abs(reward) - Math.abs(thrustTogglePenalty) - Math.abs(positiveVelocityPenalty);
        }
    }

    public void resetValueFunctionTable() {
        valueFunctionTable = new HashMap<>();
    }



    // Required data structures.

    private static int group_by_precision (double value, double precision) {
        return (int) Math.round(value / precision);
    }

    private static double angle_precision = Math.PI / 16;

    public static class State implements Serializable {
        double altitude = 0.0;
        double velocity = 0.0;
        double angle_x = 0.0;
        double angle_z = 0.0;

        private State(SimulationStatus status) { 
            double X = status.getRocketOrientationQuaternion().getX();
            double Y = status.getRocketOrientationQuaternion().getY();
            double Z = status.getRocketOrientationQuaternion().getZ();
            double W = status.getRocketOrientationQuaternion().getW();
            double xDir =  2 * (X * Z - W * Y);
            double yDir = 2 * (Y * Z + W * X);
            double zDir  = 1 - 2 * (X * X + Y * Y);
            setAltitude(status.getRocketPosition().z);
            setVelocity(Math.sqrt(Math.pow(status.getRocketRotationVelocity().x,2)
                    +Math.pow(status.getRocketRotationVelocity().y,2)
                    +Math.pow(status.getRocketRotationVelocity().z,2)));
            setAngle_x(Math.acos(xDir)*Math.signum(yDir));
            setAngle_z(Math.acos(zDir));
        }

        public void setAltitude(double altitude) {
            this.altitude = group_by_precision(altitude, 0.1);
        }

        public double getAltitude() {
            return altitude;
        }

        public void setVelocity(double velocity) {
            this.velocity = group_by_precision(velocity, 0);
        }

        public double getVelocity() {
            return altitude;
        }

        public void setAngle_x(double angle) {
            this.angle_x = group_by_precision(angle, angle_precision);
        }

        public double getAngle_x() {
            return angle_x;
        }

        public void setAngle_z(double angle) {
            this.angle_z = group_by_precision(angle, angle_precision);
        }

        public double getAngle_z() {
            return angle_z;
        }

        @Override
        public String toString() {
            return "State(" + this.altitude + ", " + this.velocity + ")";
        }

        private int getDoubleHashCode(Double val) {
            return Double.valueOf(val).hashCode();
        }

        @Override
        public int hashCode() {
            return getDoubleHashCode(altitude) + getDoubleHashCode(velocity) + getDoubleHashCode(special_area_angle_hashcode());
        }

        private Double special_area_angle_hashcode() {
            if (this.angle_z == group_by_precision(Math.PI / 2, angle_precision)) {
                this.angle_x = 0.0;  // adapt for this special case.  May be needed when comparing equals code.
                return (double) getDoubleHashCode(this.angle_z);
            }

            return (double) getDoubleHashCode(this.angle_x) + getDoubleHashCode(this.angle_z);
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
            return altitude == other.altitude && velocity == other.velocity && angle_x == other.angle_x && angle_z == other.angle_z;
        }
    }

    private static double gimble_precision = Math.PI / 30;

    public static class Action implements Serializable {
        double thrust = 0.0;
        double gimble_x = 0.0;
        double gimble_y = 0.0;

        Action(double thrust, double gimble_x, double gimble_y) {
            this.thrust = thrust;
            setGimble_x(gimble_x);
            setGimble_y(gimble_y);
        }

        public void setThrust(double thrust) {
            this.thrust = thrust;
        }

        public double getThrust() {
            return thrust;
        }

        public void setGimble_x(double gimble_x) {
            this.gimble_x = group_by_precision(gimble_x, gimble_precision);
        }

        public double getGimble_x() {
            return gimble_x;
        }

        public void setGimble_y(double gimble_y) {
            this.gimble_y = group_by_precision(gimble_y, gimble_precision);
        }

        public double getGimble_y() {
            return gimble_y;
        }

        @Override
        public String toString() {
            return "Action(" + this.thrust + ", " + this.gimble_x + ", " + this.gimble_y + ")";
        }

        @Override
        public int hashCode() {
            return Double.valueOf(thrust).hashCode() + Double.valueOf(gimble_x).hashCode() + Double.valueOf(gimble_y).hashCode();
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
            return thrust == other.thrust && gimble_x == other.gimble_x && gimble_y == other.gimble_y;
        }
    }

    public HashMap<StateActionTuple, Double> getValueFunctionTable() {
        return valueFunctionTable;
    }

    public void setValueFunctionTable(HashMap<StateActionTuple, Double> valueFunctionTable) {
        this.valueFunctionTable = valueFunctionTable;
    }





    @FunctionalInterface
    interface TriFunction<A,B,C,R> {

        R apply(A a, B b, C c);

        default <V> TriFunction<A, B, C, V> andThen(
                Function<? super R, ? extends V> after) {
            Objects.requireNonNull(after);
            return (A a, B b, C c) -> after.apply(apply(a, b, c));
        }
    }
}




