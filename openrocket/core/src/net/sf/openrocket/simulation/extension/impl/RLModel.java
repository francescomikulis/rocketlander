package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.concurrent.Semaphore;
import java.util.function.BiFunction;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;
    private HashMap<StateActionTuple, Double> valueFunctionTable = null;

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

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/
    public ArrayList<Action> generatePossibleActions(State state) {
        ArrayList<Action> possibleActions = new ArrayList<>();
        possibleActions.add(new Action(0.0, 0, 0));
        possibleActions.add(new Action(1.0, 0, 0));
        return possibleActions;
    }

    public State generateStateFromStatus(SimulationStatus status) {
        return new State(status.getRocketPosition().z, status.getRocketVelocity().z);
    }

    private Double valueFunction(StateActionTuple stateActionTuple) {
        // given the state, be a lookup table and return a value for that state-action pair
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0;
        return valueFunctionTable.get(stateActionTuple);
    }

    private Double valueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return valueFunction(stateActionTuple);
    }

    public Action run_policy(SimulationStatus status, ArrayList<StateActionTuple> episodeStateAction) {
        State state = generateStateFromStatus(status);
        Action action = policy(state, this::valueFunction);
        episodeManager.addStateActionTuple(state, action, episodeStateAction);
        return action;
    }

    private Action policy(State state, BiFunction<State, Action, Double> func) {
        ArrayList<Action> possibleActions = generatePossibleActions(state);

        double val = Double.NEGATIVE_INFINITY;
        ArrayList<Action> bestActions = new ArrayList<>();
        for (Action action: possibleActions) {
            double v = func.apply(state, action);
            if (v > val) {
                // value is best compared to all previous encounters.  Reset bestAction ArrayList.
                val = v;
                bestActions = new ArrayList<>();
                bestActions.add(action);
            } else if (v == val) {
                // value is equal to other best value.  Add to bestAction ArrayList.
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
        double G = -Math.abs(stateActionTuples.get(maxTimeStep - 1).state.velocity);  // landing velocity
        double discount = 0.999;
        double alpha = 0.1;
        double reward = -0.01;

        for (int i = maxTimeStep - 1; i >= 0; i--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(i);
            if (valueFunctionTable == null) {
                System.out.println("VALUE FUNCTION TABLE WAS NULL AT TIMESTEP: " + i);
            }
            double stateActionValue = valueFunction(stateActionTuple);
            valueFunctionTable.put(stateActionTuple, stateActionValue + alpha * (G - stateActionValue));
            G = (discount * G) + reward;
        }
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
}




