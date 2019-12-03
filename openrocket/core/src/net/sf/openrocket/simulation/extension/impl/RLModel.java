package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;

import net.sf.openrocket.simulation.extension.impl.methods.*;
import net.sf.openrocket.util.Quaternion;

import java.io.Serializable;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Semaphore;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;
    private static boolean ONLY_GREEDY = true;

    private static float THRUST_ON_PENALTY = -5;

    // MonteCarlo or TD0
    private ModelBaseImplementation primaryMethod = new MonteCarlo();
    private ModelBaseImplementation secondaryMethod = new TD0();
    public SimulationType simulationType = SimulationType._3D;

    enum SimulationType {
        _1D, _2D, _3D
    }

    float discount = 0.99f;
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
        if (simulationType == SimulationType._1D) {
            possibleGimbleYValues = new ArrayList<>();
            possibleGimbleYValues.add(0.0f);
            possibleGimbleZValues = new ArrayList<>();
            possibleGimbleZValues.add(0.0f);
        } else if (simulationType == SimulationType._2D) {
            possibleGimbleYValues = new ArrayList<>();
            possibleGimbleYValues.add(0.0f);
            possibleGimbleYValues.add((float)Math.PI);
        }
        for (float possibleThrust: possibleThrustValues) {
            for (float possibleGimbleY: possibleGimbleYValues) {
                for (float possibleGimbleZ: possibleGimbleZValues) {
                    possibleActions.add(new Action(possibleThrust, possibleGimbleY, possibleGimbleZ));
                }
            }
        }

        return possibleActions;
    }

    // policy management

    private Action run_policy(State state) {
        ArrayList<Action> possibleActions = generatePossibleActions(state);
        Action action = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {

            action = policy(state, possibleActions, primaryMethod::valueFunction);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            Action bestLanderAction = policy(state, possibleActions, primaryMethod::landingValueFunction);
            Action bestStabilizerAction = policy(state, possibleActions, secondaryMethod::stabilizingValueFunction);
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
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= 0.05) {
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
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            primaryMethod.updateStepFunction(stateActionTuples);

        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            primaryMethod.updateStepLandingFunction(stateActionTuples);
            secondaryMethod.updateStepStabilizingFunction(stateActionTuples);
        }
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples, TerminationBooleanTuple terminationBooleanTuple) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            primaryMethod.updateTerminalFunction(stateActionTuples);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            // don't learn how to land if not hitting ground WHEN NOT IN 3D
            if ((simulationType == SimulationType._1D) || (terminationBooleanTuple.landingSucceeded())) {
                primaryMethod.updateTerminalLandingFunction(stateActionTuples);
            }
            if ((simulationType != SimulationType._1D) && (terminationBooleanTuple.verticalSuccess))
                secondaryMethod.updateTerminalStabilizingFunction(stateActionTuples);
        }
    }

    public void resetValueFunctionTable() {
        setValueFunctionTable(new OptimizedMap());
    }

    public OptimizedMap getValueFunctionTable() {
        return primaryMethod.getValueFunctionTable();
    }

    public void setValueFunctionTable(OptimizedMap valueFunctionTable) {
        primaryMethod.setValueFunctionTable(valueFunctionTable);
        secondaryMethod.setValueFunctionTable(valueFunctionTable);
    }

    public ModelBaseImplementation getPrimaryMethod() {
        return primaryMethod;
    }

    public void setPrimaryMethod(ModelBaseImplementation method) {
        // conserve the instance of the OptimizedMap
        OptimizedMap valueFunctionTable = primaryMethod.getValueFunctionTable();
        primaryMethod = method;
        secondaryMethod = primaryMethod;
        method.setValueFunctionTable(valueFunctionTable);
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




