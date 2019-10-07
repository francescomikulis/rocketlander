package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;
import java.util.Random;
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

    public State generateStateFromStatus(SimulationStatus status) {
        return new State(status.getRocketPosition().z, status.getRocketVelocity().z);
    }

    private Double valueFunction(StateActionTuple stateActionTuple) {
        Action action = stateActionTuple.action;
        if (action.thrust > 0) {
            if (stateActionTuple.state.velocity > 0) {
                double penalty = THRUST_ON_PENALTY * action.thrust * 100;
                if (!valueFunctionTable.containsKey(stateActionTuple))
                    return THRUST_ON_PENALTY; // penalty
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
        State state = generateStateFromStatus(status);
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
        if (Math.abs(lastStateActionTuple.state.velocity) < 2.0) {
            lowSpeedLandingBonus = 50;
            if (Math.abs(lastStateActionTuple.state.velocity) < 1.0)
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

        double G = -landingVelocity * altitudeMultiplier + lowSpeedLandingBonus;  // landing velocity
        // System.out.println("Starting with penalty: " + G);
        double discount = 0.999;
        double alpha = 0.1;
        double reward = -0.01;

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

    public static class State implements Serializable {
        double altitude = 0.0;
        double velocity = 0.0;

        State(double altitude, double velocity) {
            setAltitude(altitude);
            setVelocity(velocity);
        }

        public void setAltitude(double altitude) {
            this.altitude = round(altitude, 0);
        }

        public double getAltitude() {
            return altitude;
        }

        public void setVelocity(double velocity) {
            this.velocity = round(velocity, 0);
        }

        public double getVelocity() {
            return altitude;
        }

        @Override
        public String toString() {
            return "State(" + this.altitude + ", " + this.velocity + ")";
        }

        @Override
        public int hashCode() {
            return Double.valueOf(altitude).hashCode() + Double.valueOf(velocity).hashCode();
        }
    }

    public static class Action implements Serializable {
        double thrust = 0.0;
        double gimble_x = 0.0;
        double gimble_y = 0.0;

        Action(double thrust, double gimble_x, double gimble_y) {
            this.thrust = thrust;
            this.gimble_x = gimble_x;
            this.gimble_y = gimble_y;
        }

        public void setThrust(double thrust) {
            this.thrust = thrust;
        }

        public double getThrust() {
            return thrust;
        }

        public void setGimble_x(double gimble_x) {
            this.gimble_x = gimble_x;
        }

        public double getGimble_x() {
            return gimble_x;
        }

        public void setGimble_y(double gimble_y) {
            this.gimble_y = gimble_y;
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
    }

    private static double round (double value, int precision) {
        double scale = Math.pow(10, precision);
        return (double) Math.round(value * scale) / scale;
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




