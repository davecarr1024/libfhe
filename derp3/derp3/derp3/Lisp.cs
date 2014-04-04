using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public static class Lisp
    {
        public delegate Value BuiltinDelegate(List<Expr> args, Env env);

        public class Value
        {
            public enum Types
            {
                INT,
                FLOAT,
                STRING,
                BUILTIN,
                FUNCTION,
            }

            public Types Type { get; set; }

            public int Int { get; set; }

            public float Float { get; set; }

            public string String { get; set; }

            public BuiltinDelegate Builtin { get; set; }

            public List<string> Params { get; set; }

            public Expr Body { get; set; }

            public Value(int intValue)
            {
                Type = Types.INT;
                Int = intValue;
            }

            public Value(float floatValue)
            {
                Type = Types.FLOAT;
                Float = floatValue;
            }

            public Value(string stringValue)
            {
                Type = Types.STRING;
                String = stringValue;
            }

            public Value(BuiltinDelegate builtin)
            {
                Type = Types.BUILTIN;
                Builtin = builtin;
            }

            public Value(List<string> paramList, Expr body)
            {
                Type = Types.FUNCTION;
                Params = paramList;
                Body = body;
            }

            public Value Clone()
            {
                switch (Type)
                {
                    case Types.INT:
                        return new Value(Int);
                    case Types.FLOAT:
                        return new Value(Float);
                    case Types.STRING:
                        return new Value(String);
                    case Types.BUILTIN:
                        return new Value(Builtin);
                    case Types.FUNCTION:
                        return new Value(Params, Body);
                    default:
                        throw new Exception("invalid val type " + Type);
                }
            }

            public override bool Equals(object obj)
            {
                Value value = obj as Value;
                return value != null && value.Type == Type &&
                    (
                        (Type == Types.INT && value.Int == Int) ||
                        (Type == Types.FLOAT && value.Float == Float) ||
                        (Type == Types.STRING && value.String == String)
                    );
            }

            public override int GetHashCode()
            {
                return base.GetHashCode();
            }
        }

        public class Expr
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

            public string Id { get; set; }

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
                            Id = expr.Value;
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

        public class Env
        {
            public Dictionary<string, Value> Values { get; set; }

            public Env()
            {
                Values = new Dictionary<string, Value>()
                {
                    { "define", new Value(Lisp.Define) },
                    { "add", new Value(Lisp.Add) },
                    { "sub", new Value(Lisp.Sub) },
                    { "mul", new Value(Lisp.Mul) },
                    { "div", new Value(Lisp.Div) },
                };
            }

            public Env(Dictionary<string, Value> vals)
                : this()
            {
                foreach (KeyValuePair<string, Value> val in vals)
                {
                    Values.Add(val.Key, val.Value);
                }
            }

            public Env Clone()
            {
                Env env = new Env();
                foreach (KeyValuePair<string, Value> val in Values)
                {
                    env.Values[val.Key] = val.Value;
                }
                return env;
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
            return Eval(input, new Env());
        }

        public static Value Eval(string input, Env env)
        {
            return Parser.Parse(input).Children.Select(expr => Eval(new Expr(expr), env)).Last();
        }

        private static Value Eval(Expr expr, Env env)
        {
            switch (expr.Type)
            {
                case Expr.Types.ID:
                    Value value;
                    if (env.Values.TryGetValue(expr.Id, out value))
                    {
                        return value;
                    }
                    else
                    {
                        throw new Exception("unknown var " + expr.Id);
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
                        return Apply(Eval(expr.Children.First(), env), expr.Children.Skip(1).ToList(), env);
                    }
                default:
                    throw new Exception("unknown expr type " + expr.Type);
            }
        }

        private static Value Apply(Value func, List<Expr> args, Env env)
        {
            switch (func.Type)
            {
                case Value.Types.BUILTIN:
                    return func.Builtin(args, env);
                case Value.Types.FUNCTION:
                    if (func.Params.Count != args.Count)
                    {
                        throw new Exception("func arg count mismatch");
                    }
                    else
                    {
                        Env funcEnv = env.Clone();
                        for (int i = 0; i < args.Count; ++i)
                        {
                            funcEnv.Values[func.Params[i]] = Eval(args[i], env);
                        }
                        return Eval(func.Body, funcEnv);
                    }
                default:
                    throw new Exception("invalid func type " + func.Type);
            }
        }

        private static Value Define(List<Expr> args, Env env)
        {
            if (args.Count == 2 && args[0].Type == Expr.Types.ID)
            {
                return env.Values[args[0].Id] = Eval(args[1], env);
            }
            else if (args.Count == 3 &&
                args[0].Type == Expr.Types.ID &&
                args[1].Type == Expr.Types.LIST &&
                args[1].Children.All(arg => arg.Type == Expr.Types.ID))
            {
                return env.Values[args[0].Id] = new Value(args[1].Children.Select(arg => arg.Id).ToList(), args[2]);
            }
            else
            {
                throw new Exception("invalid define");
            }
        }

        private static Value Add(List<Expr> args, Env env)
        {
            List<Value> vals = args.Select(arg => Eval(arg, env)).ToList();
            if (vals.Count == 2 && vals[0].Type == Value.Types.INT && vals[1].Type == Value.Types.INT)
            {
                return new Value(vals[0].Int + vals[1].Int);
            }
            else
            {
                throw new Exception("invalid add");
            }
        }

        private static Value Sub(List<Expr> args, Env env)
        {
            List<Value> vals = args.Select(arg => Eval(arg, env)).ToList();
            if (vals.Count == 2 && vals[0].Type == Value.Types.INT && vals[1].Type == Value.Types.INT)
            {
                return new Value(vals[0].Int - vals[1].Int);
            }
            else
            {
                throw new Exception("invalid add");
            }
        }

        private static Value Mul(List<Expr> args, Env env)
        {
            List<Value> vals = args.Select(arg => Eval(arg, env)).ToList();
            if (vals.Count == 2 && vals[0].Type == Value.Types.INT && vals[1].Type == Value.Types.INT)
            {
                return new Value(vals[0].Int * vals[1].Int);
            }
            else
            {
                throw new Exception("invalid add");
            }
        }

        private static Value Div(List<Expr> args, Env env)
        {
            List<Value> vals = args.Select(arg => Eval(arg, env)).ToList();
            if (vals.Count == 2 && vals[0].Type == Value.Types.INT && vals[1].Type == Value.Types.INT)
            {
                if (vals[1].Int == 0)
                {
                    throw new Exception("div by zero");
                }
                else
                {
                    return new Value(vals[0].Int / vals[1].Int);
                }
            }
            else
            {
                throw new Exception("invalid add");
            }
        }
    }
}
