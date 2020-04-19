package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.StateActionClass;

import java.util.*;

import static java.lang.Math.abs;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.landerDefinition;

public class ExpressionEvaluator {
    // public interfacing class
    public static final class Formula extends Term {
        Function function;
        String functionString = "";
        public Formula(Function function) {
            super("");
            this.function = function;
        }
        @Override
        public double evaluate(StateActionClass object) {
            return function.evaluate(object);
        }
        @Override
        public double evaluateBestGuess(StateActionClass primary, StateActionClass fallback) {
            return function.evaluateBestGuess(primary, fallback);
        }
        @Override
        public String toString() {
            if (functionString.equals("")) { functionString = function.toString(); }
            return functionString;
        }
    }

    public Formula generateFormula(String stringFormula) {
        if (memoizedFormulas.containsKey(stringFormula)) return memoizedFormulas.get(stringFormula);
        stringFormula = stringFormula.replaceAll(" ", "");
        Formula formula = new Formula(generate_terms_from_sentence(stringFormula));
        memoizedFormulas.put(stringFormula, formula);
        return formula;
    }

    public static double evaluateFormula(Formula formula, StateActionClass object) {
        return formula.evaluate(object);
    }

    public Formula generateAssignmentFormula(String stringFormula) {
        stringFormula = stringFormula.replaceAll(" ", "");
        Formula formula = generateFormula(stringFormula);
        assert (!formula.toString().contains("("));
        return formula;
    }

    public static double evaluateFormulaBestGuess(Formula formula, StateActionClass primary, StateActionClass fallback) {
        return formula.evaluateBestGuess(primary, fallback);
    }

    private static volatile ExpressionEvaluator instance;
    private HashMap<String, Formula> memoizedFormulas = new HashMap<>();

    private ExpressionEvaluator(){}

    public static ExpressionEvaluator getInstance() {
        if (instance == null) { // first time lock
            synchronized (ExpressionEvaluator.class) {
                if (instance == null) {  // second time lock
                    instance = new ExpressionEvaluator();
                }
            }
        }
        return instance;
    }

    private double bestGuessTermFromString(String term, StateActionClass primary, StateActionClass fallback) {
        double value = 0;
        if (primary.valueMap.containsKey(term)) value = primary.getDouble(term);
        else {
            if (fallback.valueMap.containsKey(term)) value = fallback.getDouble(term);
            else System.out.println("MAJOR ISSUE IN MISSING PROPERTY: " + term);
        }
        return value;
    }

    /** Private implementation */

    private static abstract class Term {
        String identifier;
        int sign = 1;
        public Term(String identifier) {
            this.identifier = identifier;
        }

        @Override
        public int hashCode() {
            return identifier.hashCode();
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            Term other = (Term) obj;
            return this.identifier.equals(other.identifier) && this.sign == other.sign;
        }

        @Override
        public String toString() { return "Term: " + identifier; }

        public abstract double evaluate(StateActionClass object);
        public abstract double evaluateBestGuess(StateActionClass primary, StateActionClass fallback);
    }

    private static class Constant extends Term {
        double value = Double.MAX_VALUE;
        public Constant(String identifier) {
            super(identifier);
        }
        public Constant(String identifier, double value) {
            super(identifier);
            this.value = value;
        }

        @Override
        public String toString() {
            if (sign == -1) return "-" + identifier;
            else return identifier;
        }

        public double evaluate(StateActionClass object) {
            if (value != Double.MAX_VALUE) return value;
            if (sign == -1) return -object.getDouble(identifier);
            else return object.getDouble(identifier);
        }

        public double evaluateBestGuess(StateActionClass primary, StateActionClass fallback) {
            if (value != Double.MAX_VALUE) return value;

            double value = 0;
            if (primary.valueMap.containsKey(identifier)) value = primary.getDouble(identifier);
            else {
                if (fallback.valueMap.containsKey(identifier)) value = fallback.getDouble(identifier);
                else System.out.println("MAJOR ISSUE IN MISSING PROPERTY: " + identifier);
            }

            if (sign == -1) return -value;
            else return value;
        }
    }

    private static class Function extends Term {
        ArrayList<Term> inputs;
        public Function(String identifier, ArrayList<Term> inputs) {
            super(identifier);
            this.inputs = inputs;
        }

        @Override
        public boolean equals(Object obj) {
            if (!super.equals(obj)) return false;
            Function other = (Function) obj;
            return this.inputs.equals(other.inputs);
        }

        @Override
        public int hashCode() {
            return identifier.hashCode() + inputs.size();
        }

        @Override
        public String toString() {
            String string = "";
            if (sign == -1) string = "-";

            if (inputs != null) {
                if (inputs.size() == 1) {
                    string += inputs.get(0).toString();
                } else {
                    ArrayList<String> stringArrayList = new ArrayList<>();
                    for (Term input : inputs) {
                        stringArrayList.add(input.toString());
                    }
                    string = string + "(" + String.join(",", stringArrayList) + ")";
                }
            }
            return string;
        }

        public double evaluate(StateActionClass object) {
            ArrayList<Double> doubleInputs = new ArrayList<>();
            for (Term input: inputs) {
                doubleInputs.add(input.evaluate(object));
            }
            double result = mathematicalEvaluation(identifier, doubleInputs);
            if (sign == -1) return -result;
            else return result;
        }
        public double evaluateBestGuess(StateActionClass primary, StateActionClass fallback) {
            ArrayList<Double> doubleInputs = new ArrayList<>();
            for (Term input: inputs) {
                doubleInputs.add(input.evaluateBestGuess(primary, fallback));
            }
            double result = mathematicalEvaluation(identifier, doubleInputs);
            if (sign == -1) return -result;
            else return result;
        }
    }

    private static double mathematicalEvaluation(String method, ArrayList<Double> inputs) {
        if (method.equals("")) return inputs.get(0);  // assignment
        else if (method.equals("Add")) return inputs.get(0) + inputs.get(1);
        else if (method.equals("Sub")) return inputs.get(0) - inputs.get(1);
        else if (method.equals("Mult")) return inputs.get(0) * inputs.get(1);
        else if (method.equals("Div")) return inputs.get(0) / inputs.get(1);
        else if (method.equals("Abs")) return Math.abs(inputs.get(0));
        else if (method.equals("Signum")) return Math.signum(inputs.get(0));
        else if (method.equals("Sin")) return Math.sin(inputs.get(0));
        else if (method.equals("Asin")) return Math.asin(inputs.get(0));
        else if (method.equals("Cos")) return Math.cos(inputs.get(0));
        else if (method.equals("Acos")) return Math.acos(inputs.get(0));
        else if (method.equals("Tan")) return Math.tan(inputs.get(0));
        else if (method.equals("Atan")) return Math.atan(inputs.get(0));
        else if (method.equals("Atan2")) return Math.atan2(inputs.get(0), inputs.get(1));

        else if (method.equals("And")) return ((inputs.get(0) != 0) && (inputs.get(1) != 0)) ? 1 : 0;
        else if (method.equals("Or")) return ((inputs.get(0) != 0) || (inputs.get(1) != 0)) ? 1 : 0;

        else if (method.equals("Lt")) return (inputs.get(0) < inputs.get(1)) ? 1 : 0;
        else if (method.equals("Le")) return (inputs.get(0) <= inputs.get(1)) ? 1 : 0;
        else if (method.equals("Gt")) return (inputs.get(0) > inputs.get(1)) ? 1 : 0;
        else if (method.equals("Ge")) return (inputs.get(0) >= inputs.get(1)) ? 1 : 0;

        else if (method.contains("Pow")) return Math.pow(inputs.get(0), inputs.get(1));
        else if (method.contains("Exp")) return Math.exp(inputs.get(0));
        else if (method.contains("Log")) {
            double logBase = Double.parseDouble(method.substring(3));
            return Math.log(inputs.get(0)) / Math.log(logBase);
        }

        System.out.println("EQUATION NOT DEFINED IN STATE ACTION TUPLE CLASS: " + method);
        return -1.0;
    }

    private static int find_correct_parenthesis_end(String input_string) { return find_correct_parenthesis_end(input_string, 0); }
    private static int find_correct_parenthesis_end(String input_string, int from_index) {
        int counter = 1;  // # start from open parenthesis
        for (int i = from_index; i < input_string.length(); i++) {
            String character = String.valueOf(input_string.charAt(i));
            if (character.equals("(")) counter +=1;
            else if (character.equals(")")) counter -=1;
            if (counter == 0) return i;
        }
        assert false;
        return counter; // never gets hit
    }

    private static boolean function_is_valid_format(String input_string) {
        int delta = 0;
        for (int i = 0; i < input_string.length(); i++) {
            String character = String.valueOf(input_string.charAt(i));
            if (character.equals("(")) delta +=1;
            else if (character.equals(")")) delta -=1;
        }
        System.out.println("Delta" + delta);
        return delta == 0;
    }

    private static boolean is_function_arguments_format(String input_string) {
        if (input_string.contains(",")) {
            int delta = 0;
            for (int i = 0; i < input_string.length(); i++) {
                String character = String.valueOf(input_string.charAt(i));
                if (character.equals("(")) delta +=1;
                else if (character.equals(",") && (delta == 0)) return true;
                else if (character.equals(")")) delta -=1;
            }
        }
        return false;
    }

    private static ArrayList<String> split_only_on_correct_commas(String input_string) {
        assert is_function_arguments_format(input_string);
        ArrayList<String> parameters = new ArrayList<>();
        int delta = 0;
        int index = 0;
        int last_index = 0;
        while (true) {
            if (index >= input_string.length()) break;
            String c = String.valueOf(input_string.charAt(index));
            if (c.equals("(")) delta += 1;
            if ((c.equals(",")) && (delta ==0)) {
                parameters.add(input_string.substring(last_index, index));
                last_index = index;
            }
            if (c.equals(")")) delta -= 1;
            index += 1;
        }
        parameters.add(input_string.substring(last_index + 1));
        return parameters;
    }

    private static Function generate_terms_from_sentence(String sentence) {
        if (!function_is_valid_format(sentence)) {
            System.out.println("Inputted string formula has mismatched number of open and closed parenthesis!");
        }
        System.out.println(sentence);

        Term resultingTerm = recursively_generate_terms(sentence);
        if (resultingTerm.getClass().equals(Constant.class))
            return new Function("", new ArrayList<Term>(Arrays.asList(resultingTerm)));
        return (Function)resultingTerm;
    }

    private static Term recursively_generate_terms(String partial_input) {
        Term return_term;
        Character firstCharacter = partial_input.charAt(0);
        String firstString = String.valueOf(firstCharacter);
        if (firstString.equals(",")) partial_input = partial_input.substring(1);
        int sign = 1;
        if (firstString.equals("-")) {
            sign = -1;
            partial_input = partial_input.substring(1);
        }

        // first letter is digit means it's a number
        if (Character.isDigit(firstCharacter) || partial_input.equals("PI")) {
            return_term = new Constant(partial_input, parseConstant(partial_input));
            return_term.sign = sign;
            return return_term;
        } else if (Character.isLowerCase(firstCharacter)) {  // first letter is lower means it's a variable of state
            return_term = new Constant(partial_input);
            return_term.sign = sign;
            return return_term;
        }

        // dealing with more complicated scenario
        int parenthesis_index = partial_input.indexOf("(");
        if (parenthesis_index == -1) {
            // TODO: CHECK THIS CASE!!!!!!
            return_term = new Constant(partial_input, parseConstant(partial_input));
            return_term.sign = sign;
            return return_term;
        }

        // function case
        String function_name = partial_input.substring(0, parenthesis_index);
        boolean are_function_arguments = is_function_arguments_format(partial_input);

        int end_p = find_correct_parenthesis_end(partial_input, parenthesis_index + 1);
        String input_string = partial_input.substring(parenthesis_index + 1, end_p);
        ArrayList<Term> input_terms = new ArrayList<>();

        int inner_parenthesis_index = input_string.indexOf("(");

        if (are_function_arguments) {
            for (String s : split_only_on_correct_commas(input_string)) {
                input_terms.add(recursively_generate_terms(s));
            }
            return new Function("", input_terms);
        } else if (is_function_arguments_format(input_string)) {
            for (String s : split_only_on_correct_commas(input_string)) {
                input_terms.add(recursively_generate_terms(s));
            }
        } else if ((inner_parenthesis_index == -1) && (!are_function_arguments)) {
            input_terms.add(recursively_generate_terms(input_string));
        } else if ((inner_parenthesis_index != -1) && (!are_function_arguments)) {
            int inner_end_parenth_index = find_correct_parenthesis_end(input_string, inner_parenthesis_index + 1);

            String tempStr = input_string.substring(0, inner_parenthesis_index);  // java adaptation
            int start_index = tempStr.lastIndexOf(",");
            String first_half = input_string;
            if (start_index != -1) {
                first_half = input_string.substring(0, start_index);
                Term first_half_result = recursively_generate_terms(first_half);
                // TODO: this could be broken
                input_terms.add(first_half_result);
            }

            String rec = input_string.substring(start_index+1, inner_end_parenth_index+1);
            Term generated_term = recursively_generate_terms(rec);
            input_terms.add(generated_term);

            String second_half = input_string.substring(inner_end_parenth_index+1);
            if (second_half.length() > 0) {
                if (String.valueOf(second_half.charAt(0)).equals(","))
                    second_half = second_half.substring(1);
            }
            if (second_half.length() != 0) {
                Term second_half_result = recursively_generate_terms(second_half);
                // TODO: this could be broken
                input_terms.add(second_half_result);
            }
        } else {
            assert false;
        }

        System.out.println("Function name: " + function_name + ", Input terms: " + input_terms.toString());

        return_term = new Function(function_name, input_terms);
        return_term.sign = sign;
        return return_term;
    }

    private static double parseConstant(String partial_input) {
        if (partial_input.equals("PI")) return Math.PI;
        else if (partial_input.equals("e")) return Math.exp(1);
        return Double.parseDouble(partial_input);
    }


    public static void main(String[] args) {
        String formula = "Add(Abs(positionX),Abs(positionY))";

        formula = "Mult(Signum(velocityZ),Log8(Add(Abs(velocityZ),1)))";
        formula = "And(Gt(Abs(positionX), 4.0), Le(Abs(angleX), Asin(Div(PI,8))))";

        StateActionClass object = new StateActionTuple.State(null);
        object.definition = landerDefinition;
        object.setDouble("velocityZ", 25.0);
        object.setDouble("positionX", 25.0);
        object.setDouble("positionY", 10.0);
        object.setDouble("angleX", 100);
        ExpressionEvaluator.getInstance().generateFormula(formula).evaluate(object);


        OptimizedMap.convertStringFormulasToFormulas(object.definition);
        object.setSymmetry("position", "X");
        System.out.println(object.definition);

        System.out.println(((Formula)object.definition.get("formulas").get("MDPDecision")).evaluate(object));
    }
}
