package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.methods.*;

import java.util.*;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import static java.lang.Math.PI;
import static net.sf.openrocket.simulation.extension.impl.OptimizedMap.combineCoupledTripleActions;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;

    // MonteCarlo or TD0
    private ModelBaseImplementation primaryMethod = new MonteCarlo(landerDefinition);
    private ModelBaseImplementation secondaryMethod = new TD0(reacherDefinition);
    private ModelBaseImplementation tertiaryMethod = new TD0(stabilizerDefinition);
    public SimulationType simulationType = SimulationType._2D;

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

    private HashSet<Integer> generatePossibleActionValuesInts(int value, int[] definition) {
        HashSet<Integer> possibleActionValues = new HashSet<>();
        for (int i = definition[0]; i <= definition[1]; i++) {
            possibleActionValues.add(i);
        }
        return possibleActionValues;
    }

    private HashSet<Float> generatePossibleActionValues(float value, float[] definition) {
        return generatePossibleActionValues(value, definition[0], definition[1], definition[2], definition[1] - definition[0]);
    }

    private HashSet<Float> generatePossibleActionValues(float value, float MIN_VAL, float MAX_VAL, float smallest_increment, float max_increment) {
        HashSet<Float> possibleActionValues = new HashSet<>();

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

    private boolean incrementIndeces(int[] indeces, int[] maxIndeces) {
        return _incrementIndeces(indeces, maxIndeces, indeces.length - 1);
    }

    public boolean _incrementIndeces(int[] indeces, int[] maxIndeces, int position) {
        if (position == -1) return false;
        if (indeces[position] < maxIndeces[position]) {
            indeces[position] += 1;
            return true;
        }
        indeces[position] = 0;
        return _incrementIndeces(indeces, maxIndeces, position - 1);
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/

    public HashSet<Action> generatePossibleActions(State state) {
        HashMap<String, int[]> actionDefinition = state.definition.get("actionDefinitionIntegers");
        ArrayList<String> actionFields = new ArrayList<>(actionDefinition.keySet());

        ArrayList<ArrayList<Integer>> possibleActionInts = new ArrayList<>();

        HashSet<String> symmetryAxes = null;
        if (state.definition.get("meta").containsKey("symmetry")) {
            String[] symmetryAxesArray = ((String) state.definition.get("meta").get("symmetry")).split(",");
            symmetryAxes = new HashSet<>(Arrays.asList(symmetryAxesArray));
        }

        ArrayList<String> actionStrings = new ArrayList<>();
        for (String actionField: actionFields) {
            String realField = actionField;
            if (symmetryAxes != null) {
                if (symmetryAxes.contains(actionField))
                    realField += state.symmetry;
            }
            actionStrings.add(realField);
            possibleActionInts.add(new ArrayList<>(generatePossibleActionValuesInts(state.get(realField), actionDefinition.get(actionField))));
        }

        int size = actionStrings.size();
        int[] indeces = new int[size];
        int[] maxIndeces = new int[size];
        for (int i = 0; i < size; i++) {
            indeces[i] = 0;
            maxIndeces[i] = possibleActionInts.get(i).size() - 1;
        }

        HashSet<Action> possibleActions = new HashSet<>();
        while (true) {
            HashMap<String, Integer> actionValueMap = new HashMap<>();
            for (int i = 0; i < size; i++) {
                actionValueMap.put(actionStrings.get(i), possibleActionInts.get(i).get(indeces[i]));
            }
            Action action = new Action(actionValueMap, state.definition);
            action.setAllSymmetries(state.symmetry);
            possibleActions.add(action);

            if (!incrementIndeces(indeces, maxIndeces)) {
                break;
            }
        }
        return possibleActions;
    }

    // policy management

    private CoupledActions run_policy(CoupledStates coupledStates, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        double currentThrust = coupledStates.get(0).getDouble("thrust");
        CoupledActions coupledActions = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            State lastState = getLastStateOrNull(SAPrimary);
            State state = coupledStates.get(0);
            if (state.equals(lastState)) { return null; }
            Action action = policy(state, generatePossibleActions(state), primaryMethod);
            coupledActions = combineCoupledTripleActions(action, action, action);
            addStateActionTupleIfNotDuplicate(new StateActionTuple(state, action), SAPrimary);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {

            State lastLanderState = getLastStateOrNull(SAPrimary);
            Action bestLanderAction = getLastActionOrNull(SAPrimary);
            State landerState = coupledStates.get(0);
            if (!OptimizedMap.equivalentState(landerState, lastLanderState)) {
                bestLanderAction = policy(landerState, generatePossibleActions(landerState), primaryMethod);
                addStateActionTupleIfNotDuplicate(new StateActionTuple(landerState, bestLanderAction), SAPrimary);
            }

            // propagate selection
            for (Object entryObject: bestLanderAction.definition.get("actionDefinition").entrySet()) {
                Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>) entryObject;
                String field = entry.getKey();
                double value = bestLanderAction.getDouble(field);

                coupledStates.get(0).setDouble(field, value);
                coupledStates.get(1).setDouble(field, value);
                coupledStates.get(2).setDouble(field, value);
            }

            // gimbal action selection below

            State lastGimbalXState = getLastStateOrNull(SAGimbalX);
            Action bestGimbalXAction = getLastActionOrNull(SAGimbalX);
            // calculate gimbalX
            State gimbalXState = coupledStates.get(1);
            if (OptimizedMap.needToChooseNewAction(gimbalXState, lastGimbalXState, bestGimbalXAction)) {
                HashSet<Action> possibleActions = generatePossibleActions(gimbalXState);
                if (conditionsForSecondaryMethod(coupledStates.get(0), "X")) {
                    bestGimbalXAction = policy(gimbalXState, possibleActions, secondaryMethod);
                } else {
                    bestGimbalXAction = policy(gimbalXState, possibleActions, tertiaryMethod);
                }
                bestGimbalXAction.setDouble("gimbal", bestGimbalXAction.getDouble("gimbalX"));
                addStateActionTupleIfNotDuplicate(new StateActionTuple(gimbalXState, bestGimbalXAction), SAGimbalX);
            }

            State lastGimbalYState = getLastStateOrNull(SAGimbalY);
            Action bestGimbalYAction = getLastActionOrNull(SAGimbalY);
            // calculate gimbalY
            State gimbalYState = coupledStates.get(2);
            if (OptimizedMap.needToChooseNewAction(gimbalYState, lastGimbalYState, bestGimbalYAction)) {
                HashSet<Action> possibleActions = generatePossibleActions(gimbalYState);
                if (conditionsForSecondaryMethod(coupledStates.get(0), "Y")) {
                    bestGimbalYAction = policy(gimbalYState, possibleActions, secondaryMethod);
                } else {
                    bestGimbalYAction = policy(gimbalYState, possibleActions, tertiaryMethod);
                }
                bestGimbalYAction.setDouble("gimbal", bestGimbalYAction.getDouble("gimbalY"));
                addStateActionTupleIfNotDuplicate(new StateActionTuple(gimbalYState, bestGimbalYAction), SAGimbalY);
            }

            if ((bestGimbalXAction != null) && (Math.abs(bestGimbalXAction.getDouble("gimbal")) > 200.0)) {
                System.out.println("Something is fucked up with this gimbal (X): " + bestGimbalXAction.getDouble("gimbal"));
            }
            if ((bestGimbalYAction != null) && (Math.abs(bestGimbalYAction.getDouble("gimbal")) > 200.0)) {
                System.out.println("Something is fucked up with this gimbal (Y): " + bestGimbalYAction.getDouble("gimbal"));
            }
            coupledActions = combineCoupledTripleActions(bestLanderAction, bestGimbalXAction, bestGimbalYAction);
        }
        return coupledActions;
    }

    private void addStateActionTupleIfNotDuplicate(StateActionTuple stateActionTuple, ArrayList<StateActionTuple> SA) {
        if (SA.size() == 0)
            SA.add(stateActionTuple);
        else if (!OptimizedMap.equivalentState(SA.get(SA.size() - 1).state, stateActionTuple.state)) {
            SA.add(stateActionTuple);
        }
    }

    public CoupledStates generateNewCoupledStates(SimulationStatus status) {
        State primaryState = new State(status, primaryMethod.definition);
        State secondaryState = null;
        State tertiaryState = null;
        if (conditionsForSecondaryMethod(primaryState, "X")) {
            secondaryState = new State(status, secondaryMethod.definition);
        } else
            secondaryState = new State(status, tertiaryMethod.definition);
        if (conditionsForSecondaryMethod(primaryState, "Y"))
            tertiaryState = new State(status, secondaryMethod.definition);
        else
            tertiaryState = new State(status, tertiaryMethod.definition);

        secondaryState.setAllSymmetries("X");
        tertiaryState.setAllSymmetries("Y");
        return new CoupledStates(primaryState, secondaryState, tertiaryState);
    }

    private boolean conditionsForSecondaryMethod(StateActionClass stateActionClass, String axis) {
        return ((Math.abs(stateActionClass.getDouble("position" + axis)) >= 5.0) && (Math.abs(stateActionClass.getDouble("angle" + axis)) <= Math.asin(PI/8)));
    }


    public void updateStateBeforeNextAction(CoupledStates coupledStates, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            if (!SAPrimary.isEmpty()) {
                Action lastAction = SAPrimary.get(SAPrimary.size() - 1).action;
                State state = coupledStates.get(0);
                state.set("thrust", lastAction.get("thrust"));
                state.set("gimbalX", lastAction.get("gimbalX"));
                state.set("gimbalY", lastAction.get("gimbalY"));
            }
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if (!SAPrimary.isEmpty()) {
                Action lastLanderAction = SAPrimary.get(SAPrimary.size() - 1).action;
                State primaryState = coupledStates.get(0);
                lastLanderAction.applyDefinitionValuesToState(primaryState);
                /*
                // DYNAMIC
                for (Object objectField: lastLanderAction.definition.get("actionDefinition").keySet()) {
                    String stateField = (String)objectField;
                    state.set(stateField, lastLanderAction.get(stateField));
                }*/
            }
            if (!SAGimbalX.isEmpty()) {
                Action lastGimbalXAction = SAGimbalX.get(SAGimbalX.size() - 1).action;
                State gimbalXState = coupledStates.get(1);
                lastGimbalXAction.applyDefinitionValuesToState(gimbalXState);
            }
            if (!SAGimbalY.isEmpty()) {
                Action lastGimbalYAction = SAGimbalY.get(SAGimbalY.size() - 1).action;
                State gimbalYState = coupledStates.get(2);
                lastGimbalYAction.applyDefinitionValuesToState(gimbalYState);
            }
        }
    }

    public CoupledActions generateAction(CoupledStates state, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        return run_policy(state, SAPrimary, SAGimbalX, SAGimbalY);
    }

    public CoupledActions generateAction(CoupledStates state, ArrayList<StateActionTuple> SA) {
        return generateAction(state, SA, new ArrayList<>(), new ArrayList<>());
    }

    private Action policy(State state, HashSet<Action> possibleActions, ModelBaseImplementation method) {
        HashSet<Action> bestActions = new HashSet<>();
        //bestActions.add(new Action(state.getDouble("thrust"), state.getDouble("gimbalX"), state.getDouble("gimbalY")));

        float explorationPercentage = method.getExplorationPercentage();

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= explorationPercentage) {
            greedy = false;
        }

        for (Action action: possibleActions) {
            //try {
                float v = method.valueFunction(state, action);
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
        if (bestActions.size() == 0) {
            System.out.println("Size of best actions is 0.  This will cause failure - WHY IS IT HAPPENING?");
        }
        // ties broken completely at random
        ArrayList<Action> selectableActions = new ArrayList<>(bestActions);
        return selectableActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY, int[] lastUpdateSizes) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            if (SAPrimary.size() != lastUpdateSizes[0]) primaryMethod.updateStepFunction(SAPrimary);

        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if (stateHasChanged(SAPrimary))
                primaryMethod.updateStepLanderFunction(SAPrimary);

            if (simulationType != SimulationType._1D) {
                if (SAGimbalX.size() != lastUpdateSizes[1]) {
                    Action lastActionX = getLastActionOrNull(SAGimbalX);
                    if ((lastActionX != null) && (stateHasChanged(SAGimbalX))) {
                        String MDPName = (String) lastActionX.definition.get("meta").get("name");
                        if (MDPName.equals("reacher"))
                            secondaryMethod.updateStepReachingFunction(SAGimbalX);
                        else if (MDPName.equals("stabilizer"))
                            tertiaryMethod.updateStepStabilizerFunction(SAGimbalX);
                        else
                            System.out.println("Something is fucked up in the value function (X)");
                    }
                }

                if (SAGimbalY.size() != lastUpdateSizes[2]) {
                    Action lastActionY = getLastActionOrNull(SAGimbalY);
                    if ((lastActionY != null) && (stateHasChanged(SAGimbalY))) {
                        String MDPName = (String) lastActionY.definition.get("meta").get("name");
                        if (MDPName.equals("reacher"))
                            secondaryMethod.updateStepReachingFunction(SAGimbalY);
                        else if (MDPName.equals("stabilizer"))
                            tertiaryMethod.updateStepStabilizerFunction(SAGimbalY);
                        else
                            System.out.println("Something is fucked up in the value function (Y)");
                    }
                }
            }
        }
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY, TerminationBooleans terminationBooleans) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            primaryMethod.updateTerminalFunction(SAPrimary);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {

            // if ((simulationType == SimulationType._1D) || (terminationBooleans.landerSucceeded())) {

            primaryMethod.updateTerminalLanderFunction(SAPrimary);

            ArrayList<StateActionTuple> SAGimbalXReacher = new ArrayList<>();
            ArrayList<StateActionTuple> SAGimbalXStabilizer = new ArrayList<>();

            ArrayList<StateActionTuple> SAGimbalYReacher = new ArrayList<>();
            ArrayList<StateActionTuple> SAGimbalYStabilizer = new ArrayList<>();

            if (simulationType != SimulationType._1D) {
                for (StateActionTuple stateActionTuple: SAGimbalX) {
                    String MDPName = (String)stateActionTuple.action.definition.get("meta").get("name");
                    if (MDPName.equals("reacher"))
                        SAGimbalXReacher.add(stateActionTuple);
                    else if (MDPName.equals("stabilizer"))
                        SAGimbalXStabilizer.add(stateActionTuple);
                }
                for (StateActionTuple stateActionTuple: SAGimbalY) {
                    String MDPName = (String)stateActionTuple.action.definition.get("meta").get("name");
                    if (MDPName.equals("reacher"))
                        SAGimbalYReacher.add(stateActionTuple);
                    else if (MDPName.equals("stabilizer"))
                        SAGimbalYStabilizer.add(stateActionTuple);
                }

                double updateOrder = randomGenerator.nextDouble();
                if (updateOrder < 0.5) {
                    secondaryMethod.updateTerminalReachingFunction(SAGimbalXReacher);
                    tertiaryMethod.updateTerminalStabilizerFunction(SAGimbalXStabilizer);
                    secondaryMethod.updateTerminalReachingFunction(SAGimbalYReacher);
                    tertiaryMethod.updateTerminalStabilizerFunction(SAGimbalYStabilizer);
                } else {
                    secondaryMethod.updateTerminalReachingFunction(SAGimbalYReacher);
                    tertiaryMethod.updateTerminalStabilizerFunction(SAGimbalYStabilizer);
                    secondaryMethod.updateTerminalReachingFunction(SAGimbalXReacher);
                    tertiaryMethod.updateTerminalStabilizerFunction(SAGimbalXStabilizer);
                }
            }

            if (terminationBooleans.totalSuccess()) {
                System.out.print("+");
                /*
                System.out.println("Number of unique lander states: " + SAPrimary.size());
                System.out.println("Number of gimbal X reacher states: " + SAGimbalXReacher.size());
                System.out.println("Number of gimbal X stabilizer states: " + SAGimbalXStabilizer.size());
                System.out.println("Number of gimbal Y reacher states: " + SAGimbalYReacher.size());
                System.out.println("Number of gimbal Y stabilizer states: " + SAGimbalYStabilizer.size());
                 */
            } else {
                System.out.print("-");
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

    public CoupledActions getLastAction(ArrayList<StateActionTuple> episodeStateActionsPrimary, ArrayList<StateActionTuple> episodeStateActionsGimbalX, ArrayList<StateActionTuple> episodeStateActionsGimbalY) {
        if (episodeStateActionsPrimary.size() == 0)
            return null;

        CoupledActions coupledActions = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            Action action = episodeStateActionsPrimary.get(episodeStateActionsPrimary.size() - 1).action;
            coupledActions = combineCoupledTripleActions(action, action, action);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            coupledActions = combineCoupledTripleActions(
                    episodeStateActionsPrimary.get(episodeStateActionsPrimary.size() - 1).action,
                    episodeStateActionsGimbalX.get(episodeStateActionsGimbalX.size() - 1).action,
                    episodeStateActionsGimbalY.get(episodeStateActionsGimbalY.size() - 1).action
            );
        }
        return coupledActions;
    }


    private boolean stateHasChanged(ArrayList<StateActionTuple> episodeStateActions){
        State lastStateOrNull = getLastStateOrNull(episodeStateActions);
        if (lastStateOrNull == null) return false;
        State lastLastStateOrNull = getLastLastStateOrNull(episodeStateActions);
        return !lastStateOrNull.equals(lastLastStateOrNull);
    }

    private State getLastStateOrNull(ArrayList<StateActionTuple> episodeStateActions){
        return (episodeStateActions.isEmpty()) ? null : episodeStateActions.get(episodeStateActions.size() - 1).state;
    }

    private State getLastLastStateOrNull(ArrayList<StateActionTuple> episodeStateActions){
        return (episodeStateActions.size() <= 1) ? null : episodeStateActions.get(episodeStateActions.size() - 2).state;
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




