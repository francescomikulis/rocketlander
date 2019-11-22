package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Quaternion;

import java.io.Serializable;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Semaphore;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;
    private OptimizedMap valueFunctionTable = null;
    private static boolean ONLY_GREEDY = true;
    private Semaphore mutex = new Semaphore(1);

    private static float THRUST_ON_PENALTY = -5;

    private RLMethod currentMethod = RLMethod.MONTE;
    public SimulationType simulationType = SimulationType._3D;

    enum RLMethod {
        MONTE, TD0, SEMI_SARSA
    }
    enum SimulationType {
        _1D, _2D, _3D
    }

    float discount = 0.999f;
    float alpha = 0.3f;

    private static volatile RLModel instance;

    private RLModel(){}

    public static RLModel getInstance() {
        if (instance == null) { // first time lock
            synchronized (RLModel.class) {
                if (instance == null) {  // second time lock
                    instance = new RLModel();
                }
            }
        }
        return instance;
    }


    public void initializeModel() {
        episodeManager = RLEpisodeManager.getInstance();
        episodeManager.safeActionValueFunctionInitialization();
    }

    private ArrayList<Float> generatePossibleActionValues(float value, float smallest_increment, float max_increment, float MIN_VAL, float MAX_VAL) {
        ArrayList<Float> possibleActionValues = new ArrayList<>();

        int number_of_loops = (int) (2.0 * max_increment / smallest_increment) + 1;
        float starting_low_value =  value - max_increment;
        for (int i = 0; i < number_of_loops; i++) {
            float possibleValue = starting_low_value + i * smallest_increment;
            if ((possibleValue >= MIN_VAL) && (possibleValue <= MAX_VAL)) {
                possibleActionValues.add(possibleValue);
            }
        }

        return possibleActionValues;
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/
    public ArrayList<Action> generatePossibleActions(State state) {
        float currentThrust = (float)state.getThrustDouble();
        ArrayList<Action> possibleActions = new ArrayList<>();

        ArrayList<Float> possibleThrustValues = generatePossibleActionValues(
                currentThrust, MIN_THRUST_INCREMENT_PER_TIMESTEP, MAX_THRUST_INCREMENT_PER_TIMESTEP, MIN_THRUST, MAX_THRUST);
        ArrayList<Float> possibleGimbleYValues = generatePossibleActionValues(
                (float)state.getGimbleYDouble(), MIN_GIMBLE_Y_INCREMENT_PER_TIMESTEP, MAX_GIMBLE_Y_INCREMENT_PER_TIMESTEP,
                MIN_GIMBLE_Y, MAX_GIMBLE_Y
        );
        ArrayList<Float> possibleGimbleZValues = generatePossibleActionValues(
                (float)state.getGimbleZDouble(), MIN_GIMBLE_Z_INCREMENT_PER_TIMESTEP, MAX_GIMBLE_Z_INCREMENT_PER_TIMESTEP,
                MIN_GIMBLE_Z, MAX_GIMBLE_Z
        );
        for (float possibleThrust: possibleThrustValues) {
            for (float possibleGimbleY: possibleGimbleYValues) {
                for (float possibleGimbleZ: possibleGimbleZValues) {
                    possibleActions.add(new Action(possibleThrust, possibleGimbleY, possibleGimbleZ));
                }
            }
        }

        return possibleActions;
    }

    private Float SemiSarsaUpdateStateActionValueFunction(ArrayList<StateActionTuple> episodeStateActions) {
        double[] weight = new double[]{1.0,1.0};
        double discount = 0.999;
        double alpha = 0.3;

        int lastTimeStep = episodeStateActions.size() - 1;
        StateActionTuple currentStateAction = episodeStateActions.get(lastTimeStep);
        double rew = reward(currentStateAction.state);

        double[] evaluation = new double[]{0.0,0.0};
        evaluation[0] = discount * (weight[0] * currentStateAction.action.thrust);
        evaluation[1] = discount * (weight[1] * currentStateAction.state.velocity);


        StateActionTuple lastStateActionTuple = episodeStateActions.get(lastTimeStep - 1);

        double[] previous = new double[]{0.0,0.0};
        previous[0] = weight[0] * lastStateActionTuple.action.thrust;
        previous[1] = weight[1] * lastStateActionTuple.state.velocity;

        double[] gradient = new double[]{1.0,1.0};
        gradient[0] = lastStateActionTuple.action.thrust;
        gradient[1] = lastStateActionTuple.state.velocity;

        weight[0] = weight[0] + alpha * (rew + evaluation[0] - previous[0]) * gradient[0];
        weight[1] = weight[1] + alpha * (rew + evaluation[1] - previous[1]) * gradient[1];


        //return valueFunction(episodeStateActions, stateActionTuple);
        return 0.0f;
    }

    // traditional value function

    private float valueFunction(StateActionTuple stateActionTuple) {
        Action action = stateActionTuple.action;
        if (action.thrust > 0) {
            if (stateActionTuple.state.velocity > 0) {
                float penalty = THRUST_ON_PENALTY * action.thrust * 10;
                if (!valueFunctionTable.containsKey(stateActionTuple))
                    return penalty; // penalty
                else
                    return valueFunctionTable.get(stateActionTuple) - Math.abs(penalty);
            }
        }

        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.get(stateActionTuple);
    }

    private float valueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return valueFunction(stateActionTuple);
    }

    // lander value function

    private float landerValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getLander(stateActionTuple);
    }

    private float landerValueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return landerValueFunction(stateActionTuple);
    }

    // stabilizer value function

    private float stabilizerValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        float value = valueFunctionTable.getStabilizer(stateActionTuple);
        if (value != 0.0f) {
            //System.out.println("Already visited this state!");
            value = value;
        }
        return value;
    }

    private float stabilizerValueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return stabilizerValueFunction(stateActionTuple);
    }

    // policy management

    private Action run_policy(State state) {
        ArrayList<Action> possibleActions = generatePossibleActions(state);
        Action action = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {

            action = policy(state, possibleActions, this::valueFunction);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            Action bestLanderAction = policy(state, possibleActions, this::landerValueFunction);
            Action bestStabilizerAction = policy(state, possibleActions, this::stabilizerValueFunction);
            action = OptimizedMap.combineCoupledActions(bestLanderAction, bestStabilizerAction);
        }
        return action;
    }

    public Action generateAction(State state) {
        return run_policy(state);
    }

    private Action policy(State state, ArrayList<Action> possibleActions, BiFunction<State, Action, Float> func) {
        ArrayList<Action> bestActions = new ArrayList<>();
        bestActions.add(new Action(state.getThrustDouble(), state.getGimbleYDouble(), state.getGimbleZDouble()));

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        if (randomGenerator.nextDouble() <= 0.05) {
            greedy = false;
        }

        for (Action action: possibleActions) {
            try {
                float v = func.apply(state, action);
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
            } catch (Exception e) {
                System.out.println("Failed to compute the value in the hashmap");
            }
        }
        // ties broken completely at random
        return bestActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
            if(currentMethod == RLMethod.SEMI_SARSA)
                SemiSarsaUpdateStateActionValueFunction(stateActionTuples);
            else if(currentMethod == RLMethod.TD0)
                TD0UpdateStateActionValueFunction(stateActionTuples, this::reward);
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        updateTerminalStateActionValueFunction(stateActionTuples, false);
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples, boolean forcedTermination) {
        forcedTermination = forcedTermination || stateActionTuples.get(stateActionTuples.size() - 1).state.getVelocityDouble() == MIN_VELOCITY;
        if (forcedTermination) {  // set the last 'good' state to a bad behavior
            // bad velocity
            stateActionTuples.get(stateActionTuples.size() - 1).state.setVelocity(MIN_VELOCITY);
        }

        if(currentMethod == RLMethod.MONTE) {
            if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional)
                monteCarloUpdateStateActionValueFunction(stateActionTuples);
            else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
                // don't learn how to land if not hitting ground WHEN NOT IN 3D
                if ((simulationType == SimulationType._1D) || (!forcedTermination)) {
                    monteCarloUpdateLandingStateActionValueFunction(stateActionTuples);
                }
                monteCarloUpdateStabilizingStateActionValueFunction(stateActionTuples);
            }
        }
        else if(currentMethod == RLMethod.TD0)
            TD0UpdateStateActionValueFunction(stateActionTuples, this::terminalReward);
    }

    private float terminalReward(State lastState) {
        float lowSpeedLandingBonus = 0.0f;
        if (Math.abs(lastState.getVelocityDouble()) <= 2.0) {
            lowSpeedLandingBonus = 50;
            if (Math.abs(lastState.getVelocityDouble()) <= 1.0)
                lowSpeedLandingBonus = 100;
        }

        float altitudeMultiplier = (float)lastState.getAltitudeDouble();
        if (altitudeMultiplier <= 2.0) {
            altitudeMultiplier = 1;
        } else {
            lowSpeedLandingBonus = 0;
        }

        float landingVelocity = (float)Math.abs(lastState.getVelocityDouble());
        // this edge case will tell the system it was doing well because the multiplication wasn't penalizing enough
        if ((landingVelocity < 100) && (altitudeMultiplier != 1)) {
            landingVelocity = 100;
        }

        return -(landingVelocity * altitudeMultiplier) - altitudeMultiplier + lowSpeedLandingBonus;  // landing velocity
    }

    private float reward(State state) {
        float reward = 0.0f;

        reward = - (float)Math.abs(state.getVelocityDouble()) / 10;
        return reward;
    }

    private void monteCarloUpdateStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);

        float G = terminalReward(lastStateActionTuple.state);
        float discount = 0.999f;
        float alpha = 0.3f;

        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            try {
                float originalValue = valueFunction(stateActionTuple);
                valueFunctionTable.put(stateActionTuple, originalValue + alpha * (G - originalValue));
                G = (discount * G) - reward(stateActionTuple.state);
            } catch (Exception e) {
                System.out.println("EXCEPTION HERE WHEN EVALUATING VALUE FUNCTION");
            }
        }
    }

    private float terminalLandingReward(State lastState) {
        return - (float)Math.abs(lastState.getVelocityDouble()) * 100;
    }

    private float rewardLander(State state) {
        return - (float)Math.abs(state.getVelocityDouble()) / 10;
    }

    private void monteCarloUpdateLandingStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        float G = terminalLandingReward(lastStateActionTuple.state);
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            float originalValue = landerValueFunction(stateActionTuple);
            valueFunctionTable.putLander(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (discount * G) + rewardLander(stateActionTuple.state);
        }
    }

    private float terminalStabilizingReward(State lastState) {
        return -(float)(Math.pow(2, (Math.abs(lastState.getAngleXDouble()) + Math.abs(lastState.getAngleZDouble()))));
    }

    private float rewardStabilizer(State state) {
        return -(float)(Math.abs(state.getAngleXDouble()) + Math.abs(state.getAngleZDouble()));
    }

    private void monteCarloUpdateStabilizingStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        float G = terminalStabilizingReward(lastStateActionTuple.state);
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            float originalValue = stabilizerValueFunction(stateActionTuple);
            valueFunctionTable.putStabilizer(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (discount * G) + rewardStabilizer(stateActionTuple.state);
        }
    }

    private void TD0UpdateStateActionValueFunction(ArrayList<StateActionTuple> episodeStateActions, Function<State, Float> rewardFunction) {
        if(episodeStateActions.size() <= 2) { return; }

        StateActionTuple old = episodeStateActions.get(episodeStateActions.size() - 2);
        StateActionTuple current = episodeStateActions.get(episodeStateActions.size() - 1);
        float alpha = 0.3f;
        float gamma = 0.999f;

        if (!valueFunctionTable.containsKey(old)) valueFunctionTable.put(old, 0.0f);
        if (!valueFunctionTable.containsKey(current)) valueFunctionTable.put(current, 0.0f);

        valueFunctionTable.put(old, valueFunctionTable.get(old) +
                alpha * (rewardFunction.apply(current.state) + gamma * valueFunctionTable.get(current) - valueFunctionTable.get(old)));
    }

    public void resetValueFunctionTable() {
        valueFunctionTable = new OptimizedMap();
    }

    public OptimizedMap getValueFunctionTable() {
        return valueFunctionTable;
    }

    public void setValueFunctionTable(OptimizedMap valueFunctionTable) {
        this.valueFunctionTable = valueFunctionTable;
    }

    public RLMethod getCurrentMethod() {
        return currentMethod;
    }

    public void setCurrentMethod(RLMethod method) {
        currentMethod = method;
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




