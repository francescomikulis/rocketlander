package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.simulation.SimulationStatus;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.function.BiFunction;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManagment episodeManagment;
    public RLModel(RLEpisodeManagment episodeManagment) {
        this.episodeManagment = episodeManagment;
    }

    public ArrayList<Action> generatePossibleActions(State state) {
        ArrayList<Action> possibleActions = new ArrayList<>();
        possibleActions.add(new Action(0.0, 0, 0));
        possibleActions.add(new Action(1.0, 0, 0));
        return possibleActions;
    }

    public State generateStateFromStatus(SimulationStatus status) {
        return new State(status.getRocketPosition().z, status.getRocketVelocity().z);
    }

    private Double valueFunction(State state, Action action) {
        // given the state, be a lookup table and return a value for that state-action pair
        HashMap<StateActionTuple, Double> valueFunctionTable = RLEpisodeManagment.getValueFunctionTable();
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        // initialization
        if (!valueFunctionTable.containsKey(stateActionTuple))
            valueFunctionTable.put(stateActionTuple, 0.0);
        return valueFunctionTable.get(stateActionTuple);
    }

    public Action run_policy(SimulationStatus status) {
        return policy(status, this::valueFunction);
    }

    private Action policy(SimulationStatus status, BiFunction<State, Action, Double> func) {
        State state = generateStateFromStatus(status);
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

    public void updateStateActionValueFuncton(HashMap<String, ArrayList<Double>> episode) {
        HashMap<StateActionTuple, Double> valueFunctionTable = RLEpisodeManagment.valueFunctionTable;
        double G = 0;
        double discount = 1;
        double alpha = 0.1;
        int maxTimeStep = RLEpisodeManagment.getMaxTimestepOfEpisode(episode);
        for (int i = maxTimeStep - 1; i >= 0; i--) {
            State state = new State(episode.get("position_z").get(i), episode.get("velocity_z").get(i));

            G = discount * G;
            /// what action was taken?
            Action action = new Action(0, 0, 0);


            StateActionTuple stateActionTuple = new StateActionTuple(state, action);
            double stateActionValue = valueFunctionTable.get(stateActionTuple);
            valueFunctionTable.put(stateActionTuple, stateActionValue + alpha * (G - stateActionValue));
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
            this.altitude = round(altitude, 1);
        }

        public double getAltitude() {
            return altitude;
        }

        public void setVelocity(double velocity) {
            this.velocity = round(velocity, 1);
        }

        public double getVelocity() {
            return altitude;
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
    }

    private static double round (double value, int precision) {
        int scale = (int) Math.pow(10, precision);
        return (double) Math.round(value * scale) / scale;
    }
}




