package net.sf.openrocket.simulation.extension.impl;

import java.io.Serializable;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.ArrayList;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;


import java.lang.reflect.Field;
import java.util.*;

import static java.lang.Math.abs;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

public class StateActionTuple implements Serializable {
    public State state;
    public Action action;
    public StateActionTuple(State state, Action action) {
        this.state = state;
        this.action = action;
        if (state.definition != action.definition) {
            System.out.println("State and action in same tuple have different definitions");
            this.action.definition = this.state.definition;
        }
        tupleRunAllDefinitionSpecial(this.state.definition);
    }

    private void tupleRunAllDefinitionSpecial(HashMap<String, HashMap> definition) {
        // formula logic
        if (!definition.containsKey("formulas")) return;

        HashSet<String> symmetryAxes = null;
        if (definition.get("meta").containsKey("symmetry")) {
            String[] symmetryAxesArray = ((String)definition.get("meta").get("symmetry")).split(",");
            symmetryAxes = new HashSet<>(Arrays.asList(symmetryAxesArray));

            String stateSym = state.symmetry;
            String actionSym = action.symmetry;

            if ((stateSym == null) && (actionSym == null)) {
                System.out.println("SYMMETRY MUST BE SET IN EITHER ACTION OR STATE!");
                return;
            }
            if ((stateSym != null) && (actionSym != null) && (!stateSym.equals(actionSym))){
                System.out.println("DIFFERENT VALUES OF SYMMETRY ARE INCOMPATIBLE!");
                return;
            }
            if (stateSym != null) action.symmetry = state.symmetry;
            else if (actionSym != null) state.symmetry = action.symmetry;
        }

        for (Object entryObject : definition.get("formulas").entrySet()) {
            Map.Entry<String, String> entry = (Map.Entry<String, String>) entryObject;
            String assignToField = entry.getKey();
            String formula = entry.getValue();
            if (!formula.contains("(")) {
                String term = formula;
                if ((symmetryAxes != null) && (symmetryAxes.contains(assignToField))) {
                    String formulaAxis = term.substring(term.length() - 1);
                    if ((this.state.symmetry != null) && (!formulaAxis.equals(this.state.symmetry)))
                        continue;  // attempting to override a symmetry assignment (e.g. X, Y)
                }
                double value = bestGuessTermFromString(term, state, action);
                this.state.setDouble(assignToField, value);
                value = bestGuessTermFromString(term, action, state);
                this.action.setDouble(assignToField, value);
            } else if (formula.equals("add(abs(positionX),abs(positionY))")) {
                double posX = bestGuessTermFromString("positionX", state, action);
                double posY = bestGuessTermFromString("positionY", state, action);
                this.state.setDouble(assignToField, abs(posX) + abs(posY));
                posX = bestGuessTermFromString("positionX", action, state);
                posY = bestGuessTermFromString("positionY", action, state);
                this.action.setDouble(assignToField, abs(posX) + abs(posY));
            } else if (formula.equals("log8(add(positionZ,1))")) {
                int log_base = extractLogBase(formula);
                double posZ = bestGuessTermFromString("positionZ", state, action);
                this.state.setDouble(assignToField, log_base(posZ + 1, log_base));
                posZ = bestGuessTermFromString("positionZ", action, state);
                this.action.setDouble(assignToField, log_base(posZ + 1, log_base));
            } else if (formula.equals("signum(velocityZ)*log8(add(abs(velocityZ),1))")) {
                int log_base = extractLogBase(formula);
                double velZ = bestGuessTermFromString("velocityZ", state, action);
                this.state.setDouble(assignToField, Math.signum(velZ)*log_base(abs(velZ) + 1, log_base));
                velZ = bestGuessTermFromString("velocityZ", action, state);
                this.action.setDouble(assignToField, Math.signum(velZ)*log_base(abs(velZ) + 1, log_base));
            }  else {
                System.out.println("EQUATION NOT DEFINED IN STATE ACTION TUPLE CLASS");
            }
        }
    }

    private static int extractLogBase(String formula) {
        int log_base_index = formula.indexOf("log") + 3;
        int end_log_index = formula.indexOf("(", log_base_index);
        return Integer.parseInt(formula.substring(log_base_index, end_log_index));
    }

    private static double log_base(double input, int base) { return Math.log(input) / Math.log(base); }

    private double bestGuessTermFromString(String term, StateActionClass primary, StateActionClass fallback) {
        double value = 0;
        if (primary.valueMap.containsKey(term)) value = primary.getDouble(term);
        else {
            if (fallback.valueMap.containsKey(term)) value = fallback.getDouble(term);
            else System.out.println("MAJOR ISSUE IN MISSING PROPERTY: " + term);
        }
        return value;
    }

    @Override
    public String toString() {
        String stateString = (state != null) ? state.toString() : "";
        String actionString = (action != null) ? action.toString() : "";
        return ("(" + stateString + ", " + actionString + ")");
    }

    @Override
    public int hashCode() {
        return state.hashCode() + action.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        StateActionTuple other = (StateActionTuple) obj;
        return this.action.equals(other.action) && this.state.equals(other.state);
    }

    public static class StateActionClass implements Serializable {
        public HashMap<String, Integer> valueMap = new HashMap<>();
        public HashMap<String, HashMap> definition;
        public String symmetry = null;

        protected void runALLMDPDefinitionFormulas() {
            HashMap<String, HashMap> originalDefinition = this.definition;
            this.definition = landerDefinition;
            runMDPDefinitionFormulas();
            this.definition = reacherDefinition;
            runMDPDefinitionFormulas();
            this.definition = stabilizerDefinition;
            runMDPDefinitionFormulas();
            this.definition = originalDefinition;
        }

        protected void runMDPDefinitionFormulas() {
            if (!definition.containsKey("formulas")) return;
            for (Object entryObject : definition.get("formulas").entrySet()) {
                Map.Entry<String, String> entry = (Map.Entry<String, String>) entryObject;
                String assignToField = entry.getKey();
                String formula = entry.getValue();
                if (!formula.contains("(")) {
                    setDouble(assignToField, getDouble(formula));
                } else if (formula.equals("add(abs(positionX),abs(positionY))")) {
                    double posX = getDouble("positionX");
                    double posY = getDouble("positionY");
                    setDouble(assignToField, abs(posX) + abs(posY));
                } else if (formula.equals("log8(add(positionZ,1))")) {
                    int log_base = extractLogBase(formula);
                    double posZ = getDouble("positionZ");
                    setDouble(assignToField, log_base(posZ + 1, log_base));
                } else if (formula.equals("signum(velocityZ)*log8(add(abs(velocityZ),1))")) {
                    int log_base = extractLogBase(formula);
                    double velZ = getDouble("velocityZ");
                    setDouble(assignToField, Math.signum(velZ)*log_base(abs(velZ) + 1, log_base));
                } else {
                    System.out.println("EQUATION NOT DEFINED IN STATE ACTION TUPLE CLASS");
                }
            }
        }

        public void setSymmetry(String field, String axis) {
            symmetry = axis;

            String originalField = field + axis;
            String copyToField = field;
            if (!definition.containsKey("formulas")) {
                definition.put("formulas", new HashMap<String, String>());
            }
            definition.get("formulas").put(copyToField, originalField);
            runMDPDefinitionFormulas();
        }

        protected static int group_by_precision(double value, double precision) {
            return (int) Math.floor((double)value / (double)precision);
        }

        /**
         * e.g. symmetric values around 0 [-10, 10]
         *
         * Without symmetric grouping:
         *  (-10, -5) --> -2
         *  ( -5, -0) --> -1
         *  ( 0,   5) --> 0
         *  ( 5,  10) --> 1
         *
         * With symmetric grouping [if (result < 0) result += 1;]
         *  (-10, -5) --> -2 + 1 = -1
         *  ( -5, -0) --> -1 + 1 = 0
         *  ( 0,   5) --> 0
         *  ( 5,  10) --> 1
         */
        protected static int group_by_precision_symmetric(double value, double precision) {
            int result = (int) Math.floor((double)value / (double)precision);
            if (result < 0)
                result += 1;
            return result;
        }

        protected static int group_angle_by_precision(double angle, double precision) {
            //if (angle == 2 * Math.PI) angle -= 0.00001;  // fall into right sector
            //double shiftedAngle = ((angle + (4 * Math.PI)) % (2 * Math.PI));
            return group_by_precision(angle, precision);
        }

        public int get(String field) {
            Object result = valueMap.get(field);
            if (result == null) return 0;
            return (int)result;
        }

        public static int get(StateActionClass object, String field) {
            return object.get(field);
        }

        public double getDouble(String field) {
            return get(field) * getPrecisionConstant(field);
        }

        public float[] getConstants(String field) {
            return getDefinitionConstants(this, field);
        }

        public double getMinConstant(String field) {
            return getDefinitionConstants(this, field)[0];
        }

        public double getMaxConstant(String field) {
            return getDefinitionConstants(this, field)[1];
        }

        public double getPrecisionConstant(String field) {
            return getDefinitionConstants(this, field)[2];
        }

        public static float[] getDefinitionConstants(StateActionClass object, String field) {
            float[] defaultDefinition = new float[]{-100.0f, 100.0f, 0.0000001f};
            float[] fieldDefinition;
            if ((object == null) || (object.definition == null))
                System.out.println("SERIOUS PROBLEM IN DEFINITION INITIALIZATION");
            if (object.definition.get("stateDefinition").containsKey(field)) {
                fieldDefinition = (float[])object.definition.get("stateDefinition").get(field);
            } else {
                fieldDefinition = (float[])object.definition.get("actionDefinition").get(field);
            }
            if (fieldDefinition == null) {
                // attempt to resolve by removing axial component
                if (field.contains("X")) {
                    fieldDefinition = getDefinitionConstants(object, field.replace("X", ""));
                } else if (field.contains("Y")) {
                    fieldDefinition = getDefinitionConstants(object, field.replace("Y", ""));
                }
            }
            if ((fieldDefinition == null) || (fieldDefinition == defaultDefinition)) {
                // System.out.println("FIELD NOT DEFINED after attemping resolution: " + field);
                // printDefinition(object.definition);
                return defaultDefinition;
            }
            return fieldDefinition;
        }


        public StateActionClass set(String field, int value) {
            valueMap.put(field, value);
            return this;
        }

        public static StateActionClass set(StateActionClass object, String field, int value){
            object.valueMap.put(field, value);
            return object;
        }

        public StateActionClass setDouble(String field, double value) {
            int realValue = 0;
            float[] definitionConstants = getDefinitionConstants(this, field);
            /*
            if (value <= definitionConstants[0])
                value = definitionConstants[0] + (definitionConstants[2] / 2);
            if (value >= definitionConstants[1])
                value = definitionConstants[1] - (definitionConstants[2] / 2);
             */

            if (field.contains("gimbal") || field.contains("angle"))
                realValue = group_angle_by_precision(value, definitionConstants[2]);
            //else if (field.contains("position"))
            //    realValue = group_by_precision_symmetric(value, getPrecisionConstant(field));
            else
                realValue = group_by_precision(value, definitionConstants[2]);
            valueMap.put(field, realValue);
            return this;
        }

        protected String stringifyStateOrAction(){
            String simpleClassName = this.getClass().getSimpleName();
            String definitionField = "";
            if (simpleClassName.toLowerCase().equals("state")) definitionField = "stateDefinition";
            else if (simpleClassName.toLowerCase().equals("action")) definitionField = "actionDefinition";
            StringBuilder stringBuilder = new StringBuilder();
            stringBuilder.append(simpleClassName);
            stringBuilder.append("(");
            ArrayList<String> keys = new ArrayList<>(
                    ((HashMap<String, HashMap>)definition.get(definitionField)).keySet()
            );
            for (int i = 0; i < keys.size(); i++) {
                String field = keys.get(i);
                stringBuilder.append(field);
                stringBuilder.append(": ");
                stringBuilder.append(this.getDouble(field));
                stringBuilder.append(" -> ");
                stringBuilder.append(this.get(field));
                stringBuilder.append(", ");
            }
            stringBuilder.append(")");
            String resultString = stringBuilder.toString();
            return resultString.replace(", )", ")");
        }
    }

    // Required data structures.

    public static class State extends StateActionClass {
        public State(SimulationStatus status){
            if (status == null) return;
            constructorCode(status, landerDefinition);
            runALLMDPDefinitionFormulas();
        }

        public State(SimulationStatus status, HashMap<String, HashMap> definition) {
            if (status == null) return;
            constructorCode(status, definition);
        }

        private void constructorCode(SimulationStatus status, HashMap<String, HashMap> definition) {
            if (status == null) return;
            this.definition = definition;
            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);

            setDouble("positionX", status.getRocketPosition().x);
            setDouble("positionY", status.getRocketPosition().y);
            setDouble("positionZ", status.getRocketPosition().z);

            setDouble("velocityX", status.getRocketVelocity().x);
            setDouble("velocityY", status.getRocketVelocity().y);
            setDouble("velocityZ", status.getRocketVelocity().z);
            //setVelocity(Math.signum(status.getRocketVelocity().z) * status.getRocketVelocity().length());

            // new angle approach
            double angleX = 0;
            double angleY = 0;
            if (rocketDirection.z > 0.0) {  // rocket is still upright
                angleX = Math.asin(rocketDirection.x);
                angleY = Math.asin(rocketDirection.y);
            } else {
                double _45_deg = Math.asin(Math.sqrt(2)/2.0) - 0.0000001;
                // double _90_deg = Math.asin(1.0);
                angleX = _45_deg * Math.signum(rocketDirection.x);
                angleY = _45_deg * Math.signum(rocketDirection.y);
            }
            setDouble("angleX", angleX);
            setDouble("angleY", angleY);

            // original angle approach
            //setDouble("angleX", Math.acos(rocketDirection.x) * Math.signum(rocketDirection.y));
            //setDouble("angleZ", Math.acos(rocketDirection.z));

            // NOTE THAT THIS IS INTENTIONALLY FLIPPED!!!
            setDouble("angleVelocityX", status.getRocketRotationVelocity().y);
            setDouble("angleVelocityY", status.getRocketRotationVelocity().x);

            setDouble("time", status.getSimulationTime());

            runMDPDefinitionFormulas();
        }


        @Override
        public int hashCode() {
            return DynamicValueFunctionTable.computeIndexState(this);
        }
        /*

        private int special_area_angle_hashcode() {
            if (this.angleZ == group_angle_by_precision(Math.PI / 2, StateActionConstants.ANGLEZ_PRECISION)) {
                this.angleX = 0;  // adapt for this special case.  May be needed when comparing equals code.
                return this.angleZ;
            }
            return this.angleX * this.angleZ;
        }
         */

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            State other = (State) obj;
            boolean equivalent = true;
            for (Object objectField : definition.get("stateDefinition").keySet()) {
                String stateField = (String)objectField;
                equivalent = equivalent && this.get(stateField) == other.get(stateField);
            }
            return equivalent;
        }

        @Override
        public String toString() {
            // return stringifyObject(this);
            return stringifyStateOrAction();
        }

        public State deepcopy(HashMap<String, HashMap> definition) {
            State newState = new State(null);
            newState.definition = definition;
            newState.valueMap = new HashMap<>();
            for (Map.Entry<String, Integer> entry: this.valueMap.entrySet()) {
                newState.valueMap.put(entry.getKey(), entry.getValue());
            }
            newState.runALLMDPDefinitionFormulas();
            return newState;
        }
    }

    public static class Action extends StateActionClass {
        public Action(Object nullObj){
            if (nullObj != null) System.out.println("FAILURE!");
        }
        public Action(double thrust, double gimbalX, double gimbalY) {
            constructorCode(thrust, gimbalX, gimbalY, landerDefinition);
            runALLMDPDefinitionFormulas();
        }

        public Action(double thrust, double gimbalX, double gimbalY, HashMap<String, HashMap> definition) {
            constructorCode(thrust, gimbalX, gimbalY, definition);
        }

        private void constructorCode(double thrust, double gimbalX, double gimbalY, HashMap<String, HashMap> definition) {
            this.definition = definition;

            setDouble("thrust", thrust);
            setDouble("gimbalX", gimbalX);
            setDouble("gimbalY", gimbalY);

            runMDPDefinitionFormulas();
        }

        @Override
        public int hashCode() {
            return DynamicValueFunctionTable.computeIndexAction(this);
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
            boolean equivalent = true;
            for (Object objectField : definition.get("actionDefinition").keySet()) {
                String actionField = (String)objectField;
                equivalent = equivalent && this.get(actionField) == other.get(actionField);
            }
            return equivalent;
        }

        @Override
        public String toString() {
            // return stringifyObject(this);
            return stringifyStateOrAction();
        }

        public Action deepcopy(HashMap<String, HashMap> definition) {
            Action newAction = new Action(0, 0,0 , this.definition);
            newAction.definition = definition;
            newAction.valueMap = new HashMap<>();
            for (Map.Entry<String, Integer> entry: this.valueMap.entrySet()) {
                newAction.valueMap.put(entry.getKey(), entry.getValue());
            }
            newAction.runALLMDPDefinitionFormulas();
            return newAction;
        }

        public void applyDefinitionValuesToState(State state) {
            HashSet<String> symmetryAxes = new HashSet<String>();
            if (definition.get("meta").containsKey("symmetry")) {
                String[] symmetryAxesArray = ((String) definition.get("meta").get("symmetry")).split(",");
                symmetryAxes = new HashSet<>(Arrays.asList(symmetryAxesArray));
            }

            for (Object objectField : definition.get("actionDefinition").keySet()) {
                String actionField = (String)objectField;
                double actionFieldDouble = getDouble(actionField);
                state.setDouble(actionField, actionFieldDouble);
                if (symmetryAxes.contains(actionField)) {
                    state.setDouble(actionField + symmetry, actionFieldDouble);
                }
            }
        }
    }

    public static class CoupledStates extends ArrayList<State> {
        public CoupledStates(State... state) {
            for (State s : state) {
                super.add(s);
            }
        }
    }

    public static class CoupledActions extends StateActionClass {
        private Action thrustAction;
        private Action gimbalXAction;
        private Action gimbalYAction;
        public CoupledActions(Action thrustAction, Action gimbalXAction, Action gimbalYAction) {
            this.thrustAction = thrustAction;
            this.gimbalXAction = gimbalXAction;
            this.gimbalYAction = gimbalYAction;
        }
        public Action getAction(String field) {
            if (field.equals("thrust")) {
                return this.thrustAction;
            } else if (field.equals("gimbalX")) {
                return this.gimbalXAction;
            } else if (field.equals("gimbalY")) {
                return this.gimbalYAction;
            }
            System.out.println("attempting to pull field not contained in actions: " + field);
            return null;
        }

        @Override
        public double getDouble(String field) {
            return getAction(field).getDouble(field);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            CoupledActions other = (CoupledActions) obj;
            return (thrustAction == other.thrustAction && gimbalXAction == other.gimbalXAction && gimbalYAction == other.gimbalYAction);
        }
    }


    public static Quaternion getConjugateQuaternion(Quaternion quaternion) {
        return new Quaternion(quaternion.getW(), -quaternion.getX(), -quaternion.getY(), -quaternion.getZ());
    }

    public static Coordinate convertRocketStatusQuaternionToDirection(SimulationStatus status) {
        return status.getRocketOrientationQuaternion().rotateZ();
    }

    public static String stringifyObject(StateActionClass object) {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(object.getClass().getSimpleName());
        stringBuilder.append("(");
        for (Map.Entry<String, Integer> entry: object.valueMap.entrySet()) {
            stringBuilder.append(entry.getKey());
            stringBuilder.append(": ");
            stringBuilder.append(entry.getValue());
            stringBuilder.append(", ");
        }
        stringBuilder.replace(stringBuilder.length() - 2, stringBuilder.length(), "");
        stringBuilder.append(")");
        return stringBuilder.toString();
        /*

        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(object.getClass().getSimpleName());
        stringBuilder.append("(");
        for(Field field : object.getClass().getFields()){
            try {
                String fieldName = field.getName();
                int value = (int)field.get(object);
                stringBuilder.append(fieldName);
                stringBuilder.append(": ");
                stringBuilder.append(value);
                stringBuilder.append(", ");
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        stringBuilder.replace(stringBuilder.length() - 2, stringBuilder.length(), "");
        stringBuilder.append(")");
        return stringBuilder.toString();
         */
    }
}

