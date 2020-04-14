package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.*;

import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;

    // MonteCarlo or TD0
    private ModelBaseImplementation primaryMethod = new MonteCarlo(landerDefinition);
    private ModelBaseImplementation secondaryMethod = new TD0(reacherDefinition);
    private ModelBaseImplementation tertiaryMethod = new TD0(stabilizerDefinition);
    public SimulationType simulationType = SimulationType._3D;

    enum SimulationType {
        _1D, _2D, _3D
    }

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

    private HashSet<Double> generatePossibleActionValues(double value, float[] definition) {
        return generatePossibleActionValues(value, definition[0], definition[1], definition[2], definition[1] - definition[0]);
    }

    private HashSet<Double> generatePossibleActionValues(double value, double MIN_VAL, double MAX_VAL, double smallest_increment, double max_increment) {
        HashSet<Double> possibleActionValues = new HashSet<>();

        int number_of_loops = (int) (2.0 * max_increment / smallest_increment) + 1;
        double starting_low_value =  value - max_increment;
        for (int i = 0; i < number_of_loops; i++) {
            double possibleValue = starting_low_value + i * smallest_increment;
            if ((possibleValue >= MIN_VAL) && (possibleValue <= MAX_VAL)) {
                possibleActionValues.add(possibleValue);
            }
        }

        return possibleActionValues;
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/
    public HashSet<Action> generatePossibleActions(State state) {
        return generatePossibleActions(state, new HashSet<>(), new HashSet<>(), new HashSet<>());
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> overrideThrustValues) {
        return generatePossibleActions(state, overrideThrustValues, new HashSet<>(), new HashSet<>());
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> possibleGimbleXValues, HashSet<Double> possibleGimbleYValues) {
        return generatePossibleActions(state, new HashSet<>(), possibleGimbleXValues, possibleGimbleYValues);
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> overrideThrustValues, HashSet<Double> overrideGimbleXValues, HashSet<Double> overrideGimbleYValues) {
        double currentThrust = state.getDouble("thrust");
        HashSet<Action> possibleActions = new HashSet<>();

        if (simulationType == SimulationType._1D) {
            overrideGimbleXValues = new HashSet<>(Arrays.asList(0.0));
            overrideGimbleYValues = new HashSet<>(Arrays.asList(0.0));
        } else if (simulationType == SimulationType._2D) {
            overrideGimbleYValues = new HashSet<>(Arrays.asList(0.0, Math.PI + 0.00001f));
        }

        HashMap<String, float[]> actionDefinition = state.definition.get("actionDefinition");

        Set<String> actionFields = (actionDefinition).keySet();

        HashSet<Double> possibleThrustValues = new HashSet<>(Collections.singletonList(0.0));
        if (overrideThrustValues.size() > 0) possibleThrustValues = overrideThrustValues;
        else {
            if (actionFields.contains("thrust"))
                possibleThrustValues = generatePossibleActionValues(currentThrust, actionDefinition.get("thrust"));
        }

        HashSet<Double> possibleGimbleXValues = new HashSet<>(Collections.singletonList(0.0));
        if (overrideGimbleXValues.size() > 0) possibleGimbleXValues = overrideGimbleXValues;
        else {
            if (actionFields.contains("gimbalX"))
                possibleGimbleXValues = generatePossibleActionValues((float)state.getDouble("gimbalX"), actionDefinition.get("gimbalX"));
            if (actionFields.contains("gimbal"))
                possibleGimbleXValues = generatePossibleActionValues((float)state.getDouble("gimbalX"), actionDefinition.get("gimbal"));
        }

        HashSet<Double> possibleGimbleYValues = new HashSet<>(Collections.singletonList(0.0));
        if (overrideGimbleYValues.size() > 0) possibleGimbleYValues = overrideGimbleYValues;
        else {
            if (actionFields.contains("gimbalY"))
                possibleGimbleYValues = generatePossibleActionValues((float)state.getDouble("gimbalY"), actionDefinition.get("gimbalY"));
            if (actionFields.contains("gimbal"))
                possibleGimbleYValues = generatePossibleActionValues((float)state.getDouble("gimbalY"), actionDefinition.get("gimbal"));
        }

        for (double possibleThrust: possibleThrustValues) {
            for (double possibleGimbleY: possibleGimbleXValues) {
                for (double possibleGimbleZ: possibleGimbleYValues) {
                    possibleActions.add(new Action(possibleThrust, possibleGimbleY, possibleGimbleZ, state.definition));
                }
            }
        }

        return possibleActions;
    }

    // policy management

    private Action run_policy(State state, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        State originalState = state;
        Action action = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            State lastState = getLastStateOrNull(SAPrimary);
            if (state.equals(lastState)) { return null; }
            action = policy(state, generatePossibleActions(state), primaryMethod::valueFunction, primaryMethod.getExplorationPercentage());
            addStateActionTupleIfNotDuplicate(new StateActionTuple(state, action, primaryMethod.definition), SAPrimary);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {

            State lastLanderState = getLastStateOrNull(SAPrimary);
            Action bestLanderAction = getLastActionOrNull(SAPrimary);
            state = state.deepcopy(primaryMethod.definition);
            if (!OptimizedMap.equivalentState(state, lastLanderState)) {
                bestLanderAction = policy(state, generatePossibleActions(state), primaryMethod::valueFunction, primaryMethod.getExplorationPercentage());
                addStateActionTupleIfNotDuplicate(new StateActionTuple(state, bestLanderAction, primaryMethod.definition), SAPrimary);
            }

            double currentThrust = state.getDouble("thrust");
            state.setDouble("thrust", bestLanderAction.getDouble("thrust"));
            HashSet<Action> possibleActions = generatePossibleActions(state, new HashSet<>(Arrays.asList(bestLanderAction.getDouble("thrust"))));

            // gimbal action selection below

            State lastGimbalXState = getLastStateOrNull(SAGimbalX);
            Action bestGimbalXAction = getLastActionOrNull(SAGimbalX);
            // calculate gimbalX
            if (state.getDouble("positionX") < 4.0) {
                // stabilize X
                state = state.deepcopy(tertiaryMethod.definition);
                if (!OptimizedMap.equivalentState(state, lastGimbalXState)) {
                    possibleActions = generatePossibleActions(state, new HashSet<>(Arrays.asList(bestLanderAction.getDouble("thrust"))));
                    bestGimbalXAction = policy(state, possibleActions, tertiaryMethod::valueFunction, tertiaryMethod.getExplorationPercentage());
                    bestGimbalXAction.setDouble("gimbal", bestGimbalXAction.getDouble("gimbalX"));
                    addStateActionTupleIfNotDuplicate(new StateActionTuple(state, bestGimbalXAction, tertiaryMethod.definition), SAGimbalX);
                }
            } else {
                // reach X
                state = state.deepcopy(secondaryMethod.definition);
                if (!OptimizedMap.equivalentState(state, lastGimbalXState)) {
                    possibleActions = generatePossibleActions(state, new HashSet<>(Arrays.asList(bestLanderAction.getDouble("thrust"))));
                    bestGimbalXAction = policy(state, possibleActions, secondaryMethod::valueFunction, secondaryMethod.getExplorationPercentage());
                    bestGimbalXAction.setDouble("gimbal", bestGimbalXAction.getDouble("gimbalX"));
                    addStateActionTupleIfNotDuplicate(new StateActionTuple(state, bestGimbalXAction, secondaryMethod.definition), SAGimbalX);
                }
            }

            State lastGimbalYState = getLastStateOrNull(SAGimbalY);
            Action bestGimbalYAction = getLastActionOrNull(SAGimbalY);
            // calculate gimbalY
            if (state.getDouble("positionY") < 4.0) {
                // stabilize Y
                state = state.deepcopy(tertiaryMethod.definition);
                if (!OptimizedMap.equivalentState(state, lastGimbalYState)) {
                    possibleActions = generatePossibleActions(state, new HashSet<>(Arrays.asList(bestLanderAction.getDouble("thrust"))));
                    bestGimbalYAction = policy(state, possibleActions, tertiaryMethod::valueFunction, tertiaryMethod.getExplorationPercentage());
                    bestGimbalYAction.setDouble("gimbal", bestGimbalYAction.getDouble("gimbalY"));
                    addStateActionTupleIfNotDuplicate(new StateActionTuple(state, bestGimbalYAction, tertiaryMethod.definition), SAGimbalY);
                }
            } else {
                // reach Y
                state = state.deepcopy(secondaryMethod.definition);
                if (!OptimizedMap.equivalentState(state, lastGimbalYState)) {
                    possibleActions = generatePossibleActions(state, new HashSet<>(Arrays.asList(bestLanderAction.getDouble("thrust"))));
                    bestGimbalYAction = policy(state, possibleActions, secondaryMethod::valueFunction, secondaryMethod.getExplorationPercentage());
                    bestGimbalYAction.setDouble("gimbal", bestGimbalYAction.getDouble("gimbalY"));
                    addStateActionTupleIfNotDuplicate(new StateActionTuple(state, bestGimbalYAction, secondaryMethod.definition), SAGimbalY);
                }
            }
            state.setDouble("thrust", currentThrust);

            action = OptimizedMap.combineCoupledTripleActions(bestLanderAction, bestGimbalXAction, bestGimbalYAction);
        }
        return action;
    }

    private void addStateActionTupleIfNotDuplicate(StateActionTuple stateActionTuple, ArrayList<StateActionTuple> SA) {
        if (SA.size() == 0)
            SA.add(stateActionTuple);
        else if (!OptimizedMap.equivalentState(SA.get(SA.size() - 1).state, stateActionTuple.state)) {
            SA.add(stateActionTuple);
        }
    }

    public void updateStateBeforeNextAction(State state, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            if (!SAPrimary.isEmpty()) {
                Action lastAction = SAPrimary.get(SAPrimary.size() - 1).action;
                state.set("thrust", lastAction.get("thrust"));
                state.set("gimbalX", lastAction.get("gimbalX"));
                state.set("gimbalY", lastAction.get("gimbalY"));
            }
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if (!SAPrimary.isEmpty()) {
                Action lastLanderAction = SAPrimary.get(SAPrimary.size() - 1).action;
                state.set("thrust", lastLanderAction.get("thrust"));
                /*
                // DYNAMIC
                for (Object objectField: lastLanderAction.definition.get("actionDefinition").keySet()) {
                    String stateField = (String)objectField;
                    state.set(stateField, lastLanderAction.get(stateField));
                }*/
            }
            if (!SAGimbalX.isEmpty()) {
                Action lastGimbalXAction = SAGimbalX.get(SAGimbalX.size() - 1).action;
                state.set("gimbalX", lastGimbalXAction.get("gimbalX"));
            }
            if (!SAGimbalY.isEmpty()) {
                Action lastGimbalYAction = SAGimbalY.get(SAGimbalY.size() - 1).action;
                state.set("gimbalY", lastGimbalYAction.get("gimbalY"));
            }
        }
    }

    public Action generateAction(State state, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        return run_policy(state, SAPrimary, SAGimbalX, SAGimbalY);
    }

    public Action generateAction(State state, ArrayList<StateActionTuple> SA) {
        return generateAction(state, SA, new ArrayList<>(), new ArrayList<>());
    }

    private Action policy(State state, HashSet<Action> possibleActions, BiFunction<State, Action, Float> func, float explorationPercentage) {
        HashSet<Action> bestActions = new HashSet<>();
        bestActions.add(new Action(state.getDouble("thrust"), state.getDouble("gimbalX"), state.getDouble("gimbalY")));

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= explorationPercentage) {
            greedy = false;
        }

        for (Action action: possibleActions) {
            //try {
                float v = func.apply(state, action);
                if (greedy) {
                    if (v > val) {
                        // value is best compared to all previous encounters.  Reset bestAction ArrayList.
                        val = v;
                        bestActions = new HashSet<>();
                        bestActions.add(action);
                    } else if (v == val) {
                        // value is equal to other best value.  Add to bestAction ArrayList.
                        bestActions.add(action);
                    }
                } else {
                    bestActions.add(action);
                }
            //} catch (Exception e) {
            //    System.out.println("Failed to compute the value in the hashmap");
            //}
        }
        // ties broken completely at random
        ArrayList<Action> selectableActions = new ArrayList<>(bestActions);
        return selectableActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            primaryMethod.updateStepFunction(SAPrimary);

        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            primaryMethod.updateStepLanderFunction(SAPrimary);

            Action lastActionX = getLastActionOrNull(SAGimbalX);
            if (lastActionX != null) {
                String MDPName = (String)lastActionX.definition.get("meta").get("name");
                if (MDPName.equals("reacher"))
                    secondaryMethod.updateStepReachingFunction(SAGimbalX);
                else if (MDPName.equals("stabilizer"))
                    tertiaryMethod.updateStepStabilizerFunction(SAGimbalX);
                else
                    System.out.println("Something is fucked up in the value function (X)");
            }

            Action lastActionY = getLastActionOrNull(SAGimbalY);
            if (lastActionY != null) {
                String MDPName = (String)lastActionY.definition.get("meta").get("name");
                if (MDPName.equals("reacher"))
                    secondaryMethod.updateStepReachingFunction(SAGimbalY);
                else if (MDPName.equals("stabilizer"))
                    tertiaryMethod.updateStepStabilizerFunction(SAGimbalY);
                else
                    System.out.println("Something is fucked up in the value function (Y)");
            }
        }
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY, TerminationBooleanTuple terminationBooleanTuple) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            primaryMethod.updateTerminalFunction(SAPrimary);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if ((simulationType == SimulationType._1D) || (terminationBooleanTuple.landerSucceeded())) {
                System.out.println("Number of unique lander states: " + SAPrimary.size());
                primaryMethod.updateTerminalLanderFunction(SAPrimary);
            }
            if (simulationType != SimulationType._1D) {
                ArrayList<StateActionTuple> SAGimbalXReacher = new ArrayList<>();
                ArrayList<StateActionTuple> SAGimbalXStabilizer = new ArrayList<>();
                for (StateActionTuple stateActionTuple: SAGimbalX) {
                    String MDPName = (String)stateActionTuple.action.definition.get("meta").get("name");
                    if (MDPName.equals("reacher"))
                        SAGimbalXReacher.add(stateActionTuple);
                    else if (MDPName.equals("stabilizer"))
                        SAGimbalXStabilizer.add(stateActionTuple);
                }
                System.out.println("Number of unique reacher states (X): " + SAGimbalXReacher.size());
                secondaryMethod.updateTerminalReachingFunction(SAGimbalXReacher);
                System.out.println("Number of unique stabilizer states (X): " + SAGimbalXStabilizer.size());
                tertiaryMethod.updateTerminalStabilizerFunction(SAGimbalXStabilizer);

                ArrayList<StateActionTuple> SAGimbalYReacher = new ArrayList<>();
                ArrayList<StateActionTuple> SAGimbalYStabilizer = new ArrayList<>();
                for (StateActionTuple stateActionTuple: SAGimbalX) {
                    String MDPName = (String)stateActionTuple.action.definition.get("meta").get("name");
                    if (MDPName.equals("reacher"))
                        SAGimbalYReacher.add(stateActionTuple);
                    else if (MDPName.equals("stabilizer"))
                        SAGimbalYStabilizer.add(stateActionTuple);
                }
                System.out.println("Number of unique reacher states (Y): " + SAGimbalYReacher.size());
                secondaryMethod.updateTerminalReachingFunction(SAGimbalYReacher);
                System.out.println("Number of unique stabilizer states (Y): " + SAGimbalYStabilizer.size());
                tertiaryMethod.updateTerminalStabilizerFunction(SAGimbalYStabilizer);
            }
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
        tertiaryMethod.setValueFunctionTable(valueFunctionTable);
    }

    public ModelBaseImplementation getPrimaryMethod() {
        return primaryMethod;
    }

    public void setPrimaryMethod(ModelBaseImplementation method) {
        // conserve the instance of the OptimizedMap
        OptimizedMap valueFunctionTable = primaryMethod.getValueFunctionTable();
        primaryMethod = method;
        secondaryMethod = primaryMethod;
        tertiaryMethod = primaryMethod;
        method.setValueFunctionTable(valueFunctionTable);
    }

    public Action getLastAction(ArrayList<StateActionTuple> episodeStateActionsPrimary, ArrayList<StateActionTuple> episodeStateActionsGimbalX, ArrayList<StateActionTuple> episodeStateActionsGimbalY) {
        if (episodeStateActionsPrimary.size() == 0)
            return null;

        Action action = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            action = episodeStateActionsPrimary.get(episodeStateActionsPrimary.size() - 1).action;
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            action = OptimizedMap.combineCoupledTripleActions(
                    episodeStateActionsPrimary.get(episodeStateActionsPrimary.size() - 1).action,
                    episodeStateActionsGimbalX.get(episodeStateActionsGimbalX.size() - 1).action,
                    episodeStateActionsGimbalY.get(episodeStateActionsGimbalY.size() - 1).action
            );
        }
        return action;
    }



    private State getLastStateOrNull(ArrayList<StateActionTuple> episodeStateActions){
        return (episodeStateActions.isEmpty()) ? null : episodeStateActions.get(episodeStateActions.size() - 1).state;
    }

    private Action getLastActionOrNull(ArrayList<StateActionTuple> episodeStateActions){
        return (episodeStateActions.isEmpty()) ? null : episodeStateActions.get(episodeStateActions.size() - 1).action;
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




