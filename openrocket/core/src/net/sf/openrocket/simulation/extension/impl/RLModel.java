package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.simulation.SimulationStatus;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiFunction;


public class RLModel {
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
        // TODO:
        // given the state, be a lookup table and return a value for that state-action pair
        return 0.0;
    }

    public Action run_policy(SimulationStatus status) {
        return policy(status, this::valueFunction);
    }

    private Action policy(SimulationStatus status, BiFunction<State, Action, Double> func) {
        State state = generateStateFromStatus(status);
        ArrayList<Action> possibleActions = generatePossibleActions(state);

        double val = Double.NEGATIVE_INFINITY;
        Action applyAction = new Action(0, 0, 0);
        for (Action action: possibleActions) {
            double v = func.apply(state, action);
            if (v > val) {
                val = v;
                applyAction = action;
            }
        }

        return applyAction;
    }



    // Required data structures.

    public static class State implements Serializable {
        double altitude = 0;
        double velocity = 0;

        State(double altitude, double velocity) {
            this.altitude = altitude;
            this.velocity = velocity;
        }
    }

    public static class Action implements Serializable {
        double thrust;
        double gimble_x = 0;
        double gimble_y = 0;

        Action(double thrust, double gimble_x, double gimble_y) {
            this.thrust = thrust;
            this.gimble_x = gimble_x;
            this.gimble_y = gimble_y;
        }
    }
}




