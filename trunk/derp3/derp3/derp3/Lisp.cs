using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public static class Lisp
    {
        public class Value
        {
            public enum Types
            {
                INT,
                FLOAT,
                STRING,
            }

            public Types Type { get; set; }

            public int IntValue { get; set; }

            public float FloatValue { get; set; }

            public string StringValue { get; set; }

            public List<Value> Children { get; set; }

            public Value(int intValue)
            {
                Type = Types.INT;
                IntValue = intValue;
            }

            public Value(float floatValue)
            {
                Type = Types.FLOAT;
                FloatValue = floatValue;
            }

            public Value(string stringValue)
            {
                Type = Types.STRING;
                StringValue = stringValue;
            }

            public override bool Equals(object obj)
            {
                Value value = obj as Value;
                return value != null && value.Type == Type &&
                    (
                        (Type == Types.INT && value.IntValue == IntValue) ||
                        (Type == Types.FLOAT && value.FloatValue == FloatValue) ||
                        (Type == Types.STRING && value.StringValue == StringValue)
                    );
            }

            public override int GetHashCode()
            {
                return base.GetHashCode();
            }
        }

        private class Expr
        {
            public enum Types
            {
                ID,
                INT,
                FLOAT,
                STRING,
                LIST,
            }

            public Types Type { get; set; }

            public int Int { get; set; }

            public float Float { get; set; }

            public string String { get; set; }

            public List<Expr> Children { get; set; }

            public Expr(Parser.Result exprParent)
            {
                if (exprParent.Rule.Name != "expr")
                {
                    throw new Exception("invalid expr");
                }
                else
                {
                    Parser.Result expr = exprParent.Children[0];
                    switch (expr.Rule.Name)
                    {
                        case "id":
                            String = expr.Value;
                            Type = Types.ID;
                            break;
                        case "int":
                            Int = int.Parse(expr.Value);
                            Type = Types.INT;
                            break;
                        case "float":
                            Float = float.Parse(expr.Value);
                            Type = Types.FLOAT;
                            break;
                        case "string":
                            String = expr.Value.Substring(1, expr.Value.Length - 2);
                            Type = Types.STRING;
                            break;
                        case "listExpr":
                            Children = expr.Children[1].Children.Select(child => new Expr(child)).ToList();
                            Type = Types.LIST;
                            break;
                        default:
                            throw new Exception("invalid expr type " + expr.Rule.Name);
                    }
                }
            }
        }

        private static Parser Parser = new Parser(@"
lparen = '\(';
rparen = '\)';
float = '-?\d+\.\d+';
int = '-?\d+';
string = '"".*""';
id = '[a-zA-Z0-9-_]+';
ws ~= '\s+';

exprs => expr+;
expr => int | float | string | id | listExpr;
listExpr => lparen listContent rparen;
listContent => expr+;
        ");

        public static Value Eval(string input)
        {
            Dictionary<string, Value> env = new Dictionary<string, Value>();
            return Eval(input, env);
        }

        public static Value Eval(string input, Dictionary<string, Value> env)
        {
            return Parser.Parse(input).Children.Select(expr => Eval(new Expr(expr), env)).Last();
        }

        private static Value Eval(Expr expr, Dictionary<string, Value> env)
        {
            switch (expr.Type)
            {
                case Expr.Types.ID:
                    Value value;
                    if (env.TryGetValue(expr.String, out value))
                    {
                        return value;
                    }
                    else
                    {
                        throw new Exception("unknown var " + expr.String);
                    }
                case Expr.Types.INT:
                    return new Value(expr.Int);
                case Expr.Types.FLOAT:
                    return new Value(expr.Float);
                case Expr.Types.STRING:
                    return new Value(expr.String);
                case Expr.Types.LIST:
                    if (expr.Children.Count < 1)
                    {
                        throw new Exception("list exprs must have one or more children");
                    }
                    else
                    {
                        return Apply(expr.Children.First(), expr.Children.Skip(1).ToList(), env);
                    }
            }
        }

        private static Value Apply(Expr func, List<Expr> args, Dictionary<string, Value> env)
        {
            throw new NotImplementedException();
        }
    }
}
