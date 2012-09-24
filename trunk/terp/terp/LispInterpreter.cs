using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace terp
{
    public class LispInterpreter
    {
        public class Value
        {
            public enum Type
            {
                BuiltInFunction,
                Function,
                Int,
                Float,
                String,
            };

            public Type type { get; private set; }

            public string String { get; private set; }

            public int Int { get; private set; }

            public float Float { get; private set; }

            public List<string> Arguments { get; private set; }

            public Parser.Result Body { get; private set; }

            public delegate Value BuiltInFunctionDelegate(List<Parser.Result> argExprs, Scope scope);

            public BuiltInFunctionDelegate BuiltInFunction { get; private set; }

            public Value(string s)
            {
                type = Type.String;
                String = s;
            }

            public Value(int i)
            {
                type = Type.Int;
                Int = i;
            }

            public Value(float f)
            {
                type = Type.Float;
                Float = f;
            }

            public Value(List<string> arguments, Parser.Result body)
            {
                type = Type.Function;
                Arguments = new List<string>(arguments);
                Body = body;
            }

            public Value(BuiltInFunctionDelegate builtInFunction)
            {
                type = Type.BuiltInFunction;
                BuiltInFunction = builtInFunction;
            }
        };

        public class Scope
        {
            public Dictionary<string, Value> Values { get; set; }

            public Scope()
            {
                Values = new Dictionary<string, Value>();
            }

            public Scope(Scope scope)
            {
                Values = new Dictionary<string, Value>(scope.Values);
            }
        };

        public Value Eval(Parser.Result token, Scope scope)
        {
            if (token.Type != "expr")
            {
                throw new Exception("can't eval non-expr tokens");
            }
            else
            {
                Parser.Result expr = token.Children[0];
                if (expr.Type == "id")
                {
                    if (!scope.Values.ContainsKey(expr.Value))
                    {
                        throw new Exception("unknown variable " + expr.Value);
                    }
                    return scope.Values[expr.Value];
                }
                else if (expr.Type == "int")
                {
                    return new Value(int.Parse(expr.Value));
                }
                else if (expr.Type == "float")
                {
                    return new Value(float.Parse(expr.Value));
                }
                else if (expr.Type == "string")
                {
                    return new Value(expr.Value.Substring(1, expr.Value.Length - 2));
                }
                else if (expr.Type == "parenExpr")
                {
                    List<Parser.Result> exprList = expr.Children[1].Children;
                    if (!exprList.Any())
                    {
                        throw new Exception("trying to eval function call with no function");
                    }
                    Value function = Eval(exprList.First(), scope);
                    List<Parser.Result> argExprs = exprList.GetRange(1, exprList.Count - 1);
                    return Apply(function, argExprs, scope);
                }
                else
                {
                    throw new NotImplementedException("eval expr type " + expr.Type);
                }
            }
        }

        public Value Apply(Value function, List<Parser.Result> argExprs, Scope scope)
        {
            if (function.type == Value.Type.Function)
            {
                if (function.Arguments.Count != argExprs.Count)
                {
                    throw new Exception("incorrect number of args to function");
                }
                List<Value> args = argExprs.ConvertAll(argExpr => Eval(argExpr, scope));
                Scope functionScope = new Scope(scope);
                for (int i = 0; i < function.Arguments.Count; ++i)
                {
                    functionScope.Values[function.Arguments[i]] = args[i];
                }
                return Eval(function.Body, functionScope);
            }
            else if (function.type == Value.Type.BuiltInFunction)
            {
                return function.BuiltInFunction(argExprs, scope);
            }
            else
            {
                throw new Exception("unable to apply non-function value");
            }
        }

        public Scope DefaultScope()
        {
            Scope scope = new Scope();
            scope.Values["add"] = new Value(BuiltinAdd);
            scope.Values["define"] = new Value(BuiltinDefine);
            return scope;
        }

        private Value BuiltinAdd(List<Parser.Result> argExprs, Scope scope)
        {
            float result = 0;
            foreach (Parser.Result argExpr in argExprs)
            {
                Value arg = Eval(argExpr, scope);
                if (arg.type == Value.Type.Int)
                {
                    result += arg.Int;
                }
                else if (arg.type == Value.Type.Float)
                {
                    result += arg.Float;
                }
                else
                {
                    throw new Exception("unable to add non-numeric type " + arg.type);
                }
            }
            return new Value(result);
        }

        private Value BuiltinDefine(List<Parser.Result> argExprs, Scope scope)
        {
            if (argExprs.Count != 3)
            {
                throw new Exception("define takes 3 arguments");
            }
            else
            {
                Parser.Result nameExpr = argExprs[0].Children[0];
                if (nameExpr.Type != "id")
                {
                    throw new Exception("define arg 1 must be id");
                }
                string name = nameExpr.Value;

                Parser.Result argsExpr = argExprs[1].Children[0];
                if (argsExpr.Type != "parenExpr")
                {
                    throw new Exception("define arg 2 must be list of ids");
                }
                List<String> args = new List<string>();
                foreach (Parser.Result argExpr in argsExpr.Children[1].Children)
                {
                    if (argExpr.Children[0].Type != "id")
                    {
                        throw new Exception("define arg 2 must be list of ids");
                    }
                    args.Add(argExpr.Children[0].Value);
                }

                Parser.Result body = argExprs[2];

                Value function = new Value(args, body);
                scope.Values[name] = function;
                return function;
            }
            throw new NotImplementedException();
        }

        public Value Interpret(string input)
        {
            Scope scope = DefaultScope();
            List<Parser.Result> results = Parser.ParseMultiple(input);
            Value value = null;
            foreach (Parser.Result result in results)
            {
                value = Eval(result, scope);
            }
            return value;
        }

        private Parser Parser { get; set; }

        public LispInterpreter(string grammarFilename)
        {
            Parser = new Parser(File.ReadAllText(grammarFilename));
        }

    }
}
