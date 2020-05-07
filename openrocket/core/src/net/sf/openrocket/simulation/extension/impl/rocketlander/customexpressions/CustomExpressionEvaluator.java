package net.sf.openrocket.simulation.extension.impl.rocketlander.customexpressions;

import net.sf.openrocket.simulation.extension.impl.rocketlander.*;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple.StateActionClass;

import java.util.*;

import static net.sf.openrocket.simulation.extension.impl.rocketlander.customexpressions.Constant.parseConstant;

/**
 * CustomExpressionEvaluator uses recursive descent to build an expression object.
 * Many pre-defined expressions are present in the Function class.
 *
 * The *only* interface for generating these objects is CustomExpressionEvaluator.
 * The *only* class returned to the user is an Expression.
 *
 * @author Francesco Alessandro Stefano Mikulis-Borsoi
 */

public class CustomExpressionEvaluator {
    public Expression generateExpression(String stringExpression) {
        stringExpression = stringExpression.replaceAll(" ", "");
        if (memoizedExpressions.containsKey(stringExpression))
            return memoizedExpressions.get(stringExpression);
        Expression expression = new Expression(generate_terms_from_sentence(stringExpression));
        memoizedExpressions.put(stringExpression, expression);
        return expression;
    }

    public static double evaluateExpression(Expression expression, StateActionClass object) {
        return expression.evaluate(object);
    }

    public Expression generateAssignmentExpression(String stringExpression) {
        Expression expression = generateExpression(stringExpression);
        // assert (String.valueOf(expression.toString().charAt(0)).equals("("));
        return expression;
    }

    public static double evaluateExpressionBestGuess(Expression expression, StateActionClass primary, StateActionClass fallback) {
        return expression.evaluateBestGuess(primary, fallback);
    }

    private static volatile CustomExpressionEvaluator instance;
    private HashMap<String, Expression> memoizedExpressions = new HashMap<>();

    private CustomExpressionEvaluator(){}

    public static CustomExpressionEvaluator getInstance() {
        if (instance == null) { // first time lock
            synchronized (CustomExpressionEvaluator.class) {
                if (instance == null) {  // second time lock
                    instance = new CustomExpressionEvaluator();
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
            System.out.println("Inputted string expression has mismatched number of open and closed parenthesis!");
        }
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
            firstCharacter = partial_input.charAt(0);
        }

        // first letter is digit means it's a number
        if (Character.isDigit(firstCharacter) || partial_input.equals(partial_input.toUpperCase())) {
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
            // TODO: CHECK THIS CASE!!!!!! --> Has never happened
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

        return_term = new Function(function_name, input_terms);
        return_term.sign = sign;
        return return_term;
    }


    public static void main(String[] args) {
        RLModelSingleton model = RLModelSingleton.getInstance();
        new ValueFunctionManager(model.getMethods());

        String expression = "Add(Abs(positionX),Abs(positionY))";

        expression = "Mult(Signum(velocityZ),Log8(Add(Abs(velocityZ),1)))";
        expression = "And(Gt(Abs(positionX), 4.0), Le(Abs(angleX), Asin(Div(PI,8))))";

        StateActionClass object = new StateActionTuple.State(null);
        object.definition = new MDPDefinition();
        object.setDouble("velocityZ", 25.0);
        object.setDouble("positionX", 25.0);
        object.setDouble("positionY", 10.0);
        object.setDouble("angleX", 100);
        CustomExpressionEvaluator.getInstance().generateExpression(expression).evaluate(object);

        // test - assigning a symmetry expression creates expression
        object.definition = new MDPDefinition();
        object.setSymmetry("X");
        assert object.definition.symmetryExpressions.containsKey("X");
        assert object.definition.symmetryExpressions.get("X").containsKey("position");

        // test - input string equals output string
        expression = "And(Gt(-Abs(positionX), 4.0), Le(-Abs(angleX), Asin(Div(PI,8))))";
        String stringExpression = CustomExpressionEvaluator.getInstance().generateExpression(expression).toString();
        assert stringExpression.equals(expression.replace(" ", ""));

        RLObjectFileStore.getInstance().storeDefinition(new MDPDefinition(), "sampleMap");
        System.out.println(RLObjectFileStore.getInstance().readDefinition("sampleMap"));

        // System.out.println(((Expression)object.definition.get("expressions").get("MDPDecision")).evaluate(object));

        String f = "Add(Abs(0), -Mult(Abs(velocityZ),10))";
        System.out.println(CustomExpressionEvaluator.getInstance().generateExpression(f).evaluate(object));

        // precision verification test - ensure correct rounding
        double[] values = new double[]{0.4, 0.5, 0.6};
        double[] multipliers = new double[]{1, -1};
        double[] precisions = new double[]{1};
        for (double precision: precisions) {
            for (double multiplier: multipliers) {
                for (double val: values) {
                    int result = StateActionClass.group_by_precision(multiplier * val, precision);
                    System.out.println(multiplier * val + " by " + precision + " = " + result);
                }
            }
        }
    }
}
