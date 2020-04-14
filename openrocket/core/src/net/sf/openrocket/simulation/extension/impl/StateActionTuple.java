package net.sf.openrocket.simulation.extension.impl;

import java.io.Serializable;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;


import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import static java.lang.Math.abs;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

public class StateActionTuple implements Serializable {
    public State state;
    public Action action;
    public StateActionTuple(State state, Action action, HashMap<String, HashMap> definition) {
        this.state = state;
        this.action = action;
        this.state.definition = definition;
        this.action.definition = definition;
        // formula logic
        if (!definition.containsKey("formulas")) return;
        for (Object entryObject : definition.get("formulas").entrySet()) {
            Map.Entry<String, String> entry = (Map.Entry<String, String>) entryObject;
            String assignToField = entry.getKey();
            String formula = entry.getValue();
            if (!formula.contains("(")) {
                String term = formula;
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
            } else {
                System.out.println("EQUATION NOT DEFINED IN STATE ACTION TUPLE CLASS");
            }
        }
    }

    private double bestGuessTermFromString(String term, StateActionClass primary, StateActionClass fallback) {
        double value = 0;
        if (primary.valueMap.containsKey(term)) value = primary.getDouble(term);
        else {
            if (fallback.valueMap.containsKey(term)) value = fallback.getDouble(term);
            else System.out.println("MAJOR ISSUE IN MISSING PROPERTY");
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
                } else {
                    System.out.println("EQUATION NOT DEFINED IN STATE ACTION TUPLE CLASS");
                }
            }
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
            if (angle == 2 * Math.PI) angle -= 0.00001;  // fall into right sector
            double shiftedAngle = ((angle + (4 * Math.PI)) % (2 * Math.PI));
            return group_by_precision(shiftedAngle, precision);
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

        public static double getDouble(StateActionClass object, String field) {
            return object.get(field) * getPrecisionConstant(object, field);
        }

        public double getPrecisionConstant(String field) {
            return getPrecisionConstant(this, field);
        }

        public static double getPrecisionConstant(StateActionClass object, String field) {
            float[] fieldDefinition;
            if ((object == null) || (object.definition == null))
                System.out.println("SERIOUS PROBLEM IN DEFINITION INITIALIZATION");
            if (object.definition.get("stateDefinition").containsKey(field)) {
                fieldDefinition = (float[])object.definition.get("stateDefinition").get(field);
            } else {
                fieldDefinition = (float[])object.definition.get("actionDefinition").get(field);
            }
            if (fieldDefinition == null)
                return 0.000001;
            return fieldDefinition[2];
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
            if (field.contains("gimbal") || field.contains("angle"))
                realValue = group_angle_by_precision(value, getPrecisionConstant(field));
            else if (field.contains("position"))
                realValue = group_by_precision_symmetric(value, getPrecisionConstant(field));
            else
                realValue = group_by_precision(value, getPrecisionConstant(field));
            valueMap.put(field, realValue);
            return this;
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
            setDouble("altitude", status.getRocketPosition().z);
            setDouble("positionX", status.getRocketPosition().x);
            setDouble("positionY", status.getRocketPosition().y);
            setDouble("velocity", status.getRocketVelocity().z);
            //setVelocity(Math.signum(status.getRocketVelocity().z) * status.getRocketVelocity().length());

            // new angle approach
            setDouble("angleX", Math.asin(rocketDirection.x));
            setDouble("angleY", Math.asin(rocketDirection.y));
            // original angle approach
            //setDouble("angleX", Math.acos(rocketDirection.x) * Math.signum(rocketDirection.y));
            //setDouble("angleZ", Math.acos(rocketDirection.z));
            setDouble("time", status.getSimulationTime());

            runMDPDefinitionFormulas();
        }


        @Override
        public int hashCode() {
            int code = 0;
            for (Object objectField : definition.get("stateDefinition").keySet()) {
                String stateField = (String) objectField;
                code += get(stateField);
            }
            return code;
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
        public String toString() { return stringifyObject(this); }

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
            int code = 0;
            for (Object objectField : definition.get("actionDefinition").keySet()) {
                String stateField = (String) objectField;
                code += get(stateField);
            }
            return code;
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
        public String toString() { return stringifyObject(this); }

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

