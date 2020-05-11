package net.sf.openrocket.simulation.extension.impl.rocketlander;

import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple.CoupledStates;

import java.util.HashMap;
import java.util.concurrent.locks.ReentrantLock;

public class RLDataStoreSmartPlotValues extends HashMap<String, HashMap<String, Object[]>> {
    public transient ReentrantLock lock = new ReentrantLock();

    /**
     * Below here is code related to the storing of data for the flight status.  Used for plotting.
     * Very magical code - auto-parses the values even when non-traditional nomenclature is used.
     *
     * Initialize the hashmaps with memoizeSmartGuesses()
     * Then simply call the storeUpdatedFlightConditions() and applyCustomFlightConditions()
     * This will modify the flightStatus and trigger the data storage for plotting.
     * **/

    // Object[0th] entry is the integer of the access to the index of the state arralist
    // Object[1st] entry is the real name of that field

    public Coordinate getSmartGuessCoordinate(CoupledStates state, String field) {
        return new Coordinate(getSmartGuess(state, field + "X"), getSmartGuess(state, field + "Y"), getSmartGuess(state,field + "Z"));
    }

    public Double getSmartGuess(CoupledStates state, String originalField) {
        String stateNames = state.toStringNames();
        if (!containsKey(stateNames) || (get(stateNames).size() == 0)) {
            memoizeSmartGuesses(state);
        }

        if (!get(stateNames).containsKey(originalField)) {
            return state.get(0).getDouble(originalField);
        } else {
            int index = (int)get(stateNames).get(originalField)[0];
            String realField = (String)get(stateNames).get(originalField)[1];

            // return state.get(index).getDouble(realField);
            return getRealRepresentationDouble(state.get(index), realField);
        }
    }

    // this forces the value to be within boundaries
    private double getRealRepresentationDouble(StateActionTuple.State state, String field) {
        int[] minMax = state.definition.stateDefinitionIntegers[state.definition.reverseIndeces.get(field)];
        int originalIntValue = state.get(field);
        double result = 0.0;
        if (originalIntValue < minMax[0]) {
            result = state.set(field, minMax[0]).getDouble(field);
            state.set(field, originalIntValue);
        } else if (originalIntValue > minMax[1]) {
            result = state.set(field, minMax[1]).getDouble(field);
            state.set(field, originalIntValue);
        } else {
            result = state.getDouble(field);
        }
        return result;
    }

    public void memoizeSmartGuesses(CoupledStates state) {
        String stateNames = state.toStringNames();
        lock.lock();
        if (containsKey(stateNames)) {
            lock.unlock();
            return;
        }
        put(stateNames, new HashMap<>());
        storeSmartGuess(state, "positionX");
        storeSmartGuess(state, "positionY");
        storeSmartGuess(state, "positionZ");
        storeSmartGuess(state, "angleX");
        storeSmartGuess(state, "angleY");
        storeSmartGuess(state, "angleZ");
        storeSmartGuess(state, "angleVelocityX");
        storeSmartGuess(state, "angleVelocityY");
        storeSmartGuess(state, "angleVelocityZ");
        storeSmartGuess(state, "velocityX", "angle");
        storeSmartGuess(state, "velocityY", "angle");
        storeSmartGuess(state, "velocityZ", "angle");
        lock.unlock();
    }


    private Double storeSmartGuess(CoupledStates state, String originalField) {
        return storeSmartGuess(state, originalField, null);
    }

    private Double storeSmartGuess(CoupledStates state, String originalField, String skipContainsString) {
        String stateNames = state.toStringNames();
        String field = originalField + "";
        String potentialAxis = field.substring(field.length() - 1);
        boolean preferSymmetry = false;
        if (potentialAxis.equals("X") || potentialAxis.equals("Y")) {
            preferSymmetry = true;
            field = field.substring(0, field.length() - 1);
        } else {
            if (!potentialAxis.equals("Z")) potentialAxis = null;
        }
        get(stateNames).put(originalField, new Object[2]);
        Double result = storeSmartGuessCoordinateComponent(state, originalField, field, potentialAxis, preferSymmetry, skipContainsString);
        if (result == null) {
            // remove that field because it was not found!
            get(stateNames).remove(originalField);
            // no MDP has that field defined!
            result = state.get(0).getDouble(originalField);
        }
        return result;
    }

    private Double storeSmartGuessCoordinateComponent(CoupledStates state, String originalField, String field, String enforceSymmetryAxis, boolean preferSymmetry, String skipContainsString) {
        String lowercaseField = field.toLowerCase();
        if (preferSymmetry) {
            return storeBestGuessField(state, originalField, lowercaseField, enforceSymmetryAxis, skipContainsString);
        } else {
            // lowercaseField has the required axis
            return storeBestGuessField(state, originalField, lowercaseField, null, skipContainsString);
        }
    }

    private Double storeBestGuessField(CoupledStates state, String originalField, String lowercaseField, String enforceSymmetryAxis, String skipContainsString) {
        String stateNames = state.toStringNames();
        for (int i = state.size() - 1; i >= 0; i--) {
            StateActionTuple.State s = state.get(i);
            if (((s.symmetry == null) && (enforceSymmetryAxis == null)) || ((s.symmetry != null) && (enforceSymmetryAxis != null) && (s.symmetry.equals(enforceSymmetryAxis)))) {
                for (String definitionField: s.definition.stateDefinitionFields) {
                    if ((skipContainsString != null) && definitionField.toLowerCase().contains(skipContainsString))
                        continue;
                    if (definitionField.toLowerCase().contains(lowercaseField)) {
                        get(stateNames).get(originalField)[0] = i;
                        get(stateNames).get(originalField)[1] = definitionField;
                        return s.getDouble(definitionField);
                    }
                }
            }
        }
        for (int i = state.size() - 1; i >= 0; i--) {
            StateActionTuple.State s = state.get(i);
            if (((s.symmetry == null) && (enforceSymmetryAxis == null)) || ((s.symmetry != null) && (enforceSymmetryAxis != null) && (s.symmetry.equals(enforceSymmetryAxis)))) {
                for (String definitionField : s.definition.stateDefinitionFields) {
                    if (definitionField.toLowerCase().equals(lowercaseField)) {
                        get(stateNames).get(originalField)[0] = i;
                        get(stateNames).get(originalField)[1] = definitionField;
                        return s.getDouble(definitionField);
                    }
                }
            }
        }
        // edge case for a mal-enforced symmetryAxis - fallback to verifying for non-symmetrical MDPs
        for (int i = state.size() - 1; i >= 0; i--) {
            StateActionTuple.State s = state.get(i);
            if ((s.symmetry == null) && (enforceSymmetryAxis != null)) {
                for (String definitionField: s.definition.stateDefinitionFields) {
                    if ((skipContainsString != null) && definitionField.toLowerCase().contains(skipContainsString))
                        continue;
                    if (definitionField.toLowerCase().contains(lowercaseField + enforceSymmetryAxis.toLowerCase())) {
                        get(stateNames).get(originalField)[0] = i;
                        get(stateNames).get(originalField)[1] = definitionField;
                        return s.getDouble(definitionField);
                    }
                }
            }
        }
        for (int i = state.size() - 1; i >= 0; i--) {
            StateActionTuple.State s = state.get(i);
            if ((s.symmetry == null) && (enforceSymmetryAxis != null)) {
                for (String definitionField : s.definition.stateDefinitionFields) {
                    if (definitionField.toLowerCase().equals(lowercaseField + enforceSymmetryAxis.toLowerCase())) {
                        get(stateNames).get(originalField)[0] = i;
                        get(stateNames).get(originalField)[1] = definitionField;
                        return s.getDouble(definitionField);
                    }
                }
            }
        }
        return null;
    }
}
