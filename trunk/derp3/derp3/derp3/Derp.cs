using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public class Derp
    {
        public class Val
        {
            public enum Types
            {
                None,
                Bool,
                Int,
                Float,
                String,
                Func,
            }

            public Types Type { get; private set; }

            public bool Bool { get; set; }

            public int Int { get; set; }

            public float Float { get; set; }

            public string String { get; set; }

            public List<string> Params { get; set; }

            public List<Expr> Body { get; set; }

            public Val()
            {
                Type = Types.None;
            }

            public Val(bool b)
            {
                Type = Types.Bool;
                Bool = b;
            }

            public Val(int i)
            {
                Type = Types.Int;
                Int = i;
            }

            public Val(float f)
            {
                Type = Types.Float;
                Float = f;
            }

            public Val(string s)
            {
                Type = Types.String;
                String = s;
            }

            public Val(List<string> paramList, List<Expr> body)
            {
                Type = Types.Func;
                Params = paramList;
                Body = body;
            }

            public Val Clone()
            {
                switch (Type)
                {
                    case Types.None:
                        return new Val();
                    case Types.Bool:
                        return new Val(Bool);
                    case Types.Int:
                        return new Val(Int);
                    case Types.Float:
                        return new Val(Float);
                    case Types.String:
                        return new Val(String);
                    case Types.Func:
                        return new Val(Params, Body);
                    default:
                        throw new Exception("invalid val type " + Type);
                }
            }

            public override bool Equals(object obj)
            {
                Val val = obj as Val;
                return val != null && Type == val.Type && (
                    Type == Types.None ||
                    (Type == Types.Bool && Bool == val.Bool) ||
                    (Type == Types.Int && Int == val.Int) ||
                    (Type == Types.Float && Float == val.Float) ||
                    (Type == Types.String && String == val.String)
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
                Id,
                Int,
                Float,
                String,
                FuncDecl,
                FuncCall,
            }

            public Types Type { get; set; }

            public string Id { get; set; }

            public int Int { get; set; }

            public float Float { get; set; }

            public string String { get; set; }

            public List<string> Params { get; set; }

            public List<Expr> Body { get; set; }

            public List<Expr> Args { get; set; }

            public Expr(Parser.Result expr)
            {
                switch (expr.Rule.Name)
                {
                    case "statement":
                        InitStatement(expr.Children[0]);
                        break;
                    case "expr":
                        InitExpr(expr.Children[0]);
                        break;
                    default:
                        throw new Exception("invalid expr " + expr.Rule.Name);
                }
            }

            private void InitStatement(Parser.Result statement)
            {
                switch (statement.Rule.Name)
                {
                    case "line":
                        InitExpr(statement.Children[0].Children[0]);
                        break;
                    case "funcDecl":
                        Type = Types.FuncDecl;
                        Id = statement.Children[1].Value;
                        Parser.Result idList = statement.Children[3];
                        Params = new List<string>() { idList.Children[0].Value };
                        foreach (Parser.Result param in idList.Children[1].Children)
                        {
                            Params.Add(param.Children[1].Value);
                        }
                        Body = statement.Children[6].Children.Select(child => new Expr(child)).ToList();
                        break;
                    default:
                        throw new Exception("invalid statement type " + statement.Rule.Name);
                }
            }

            private void InitExpr(Parser.Result expr)
            {
                switch (expr.Rule.Name)
                {
                    case "id":
                        Type = Types.Id;
                        Id = expr.Value;
                        break;
                    case "int":
                        Type = Types.Int;
                        Int = int.Parse(expr.Value);
                        break;
                    case "float":
                        Type = Types.Float;
                        Float = float.Parse(expr.Value);
                        break;
                    case "string":
                        Type = Types.String;
                        String = expr.Value.Substring(1, expr.Value.Length - 2);
                        break;
                    case "funcCall":
                        Type = Types.FuncCall;
                        Id = expr.Children[0].Value;
                        Parser.Result exprList = expr.Children[2];
                        Args = new List<Expr>() { new Expr(exprList.Children[0]) };
                        foreach (Parser.Result exprIter in exprList.Children[1].Children)
                        {
                            Args.Add(new Expr(exprIter.Children[1]));
                        }
                        break;
                    case "funcCallEmpty":
                        Type = Types.FuncCall;
                        Id = expr.Children[0].Value;
                        Args = new List<Expr>();
                        break;
                    default:
                        throw new Exception("invalid expr type " + expr.Rule.Name);
                }
            }
        }

        public class Env
        {
            public Dictionary<string, Val> Vals { get; set; }

            public Env()
            {
                Vals = new Dictionary<string, Val>()
                {
                    { "None", new Val() },
                    { "True", new Val(true) },
                    { "False", new Val(false) },
                };
            }

            public Env(Dictionary<string, Val> vals)
                : this()
            {
                foreach (KeyValuePair<string, Val> val in vals)
                {
                    Vals[val.Key] = val.Value;
                }
            }

            public Env Clone()
            {
                Env env = new Env();
                foreach (KeyValuePair<string, Val> val in Vals)
                {
                    env.Vals[val.Key] = val.Value;
                }
                return env;
            }
        }

        private static Parser Parser = new Parser(@"
            lbrace = '\{';
            rbrace = '\}';
            lparen = '\(';
            rparen = '\)';
            semicolon = ';';
            comma = '\,';
            plus = '\+';
            dash = '\-';
            star = '\*';
            slash = '\/';
            def = 'def';
            float = '-?\d+\.\d+';
            int = '-?\d+';
            string = '"".*""';
            id = '[a-zA-Z0-9-_]+';
            ws ~= '\s+';
            program => statement+;
            statement => funcDecl | line;
            funcDecl => def id lparen idList rparen lbrace program rbrace;
            idList => id idListTail;
            idListTail => idListIter*;
            idListIter => comma id;
            line => expr semicolon;
            expr => funcCall | funcCallEmpty | binaryOperation | id | int | float | string;
            funcCallEmpty => id lparen rparen;
            funcCall => id lparen exprList rparen;
            exprList => expr exprListTail;
            exprListTail => exprListIter*;
            exprListIter => comma expr;
            binaryOperation => id operator id;
            operator => plus | dash | star | slash;
        ");

        public static Val Eval(string input)
        {
            return Eval(input, new Env());
        }

        public static Val Eval(string input, Env env)
        {
            return Parser.Parse(input).Children.Select(expr => Eval(new Expr(expr), env)).Last();
        }

        public static Val Eval(Expr expr, Env env)
        {
            switch (expr.Type)
            {
                case Expr.Types.Id:
                    Val val;
                    if (env.Vals.TryGetValue(expr.Id, out val))
                    {
                        return val;
                    }
                    else
                    {
                        throw new Exception("unknown var " + expr.Id);
                    }
                case Expr.Types.Int:
                    return new Val(expr.Int);
                case Expr.Types.Float:
                    return new Val(expr.Float);
                case Expr.Types.String:
                    return new Val(expr.String);
                case Expr.Types.FuncDecl:
                    env.Vals[expr.Id] = new Val(expr.Params, expr.Body);
                    return new Val();
                case Expr.Types.FuncCall:
                    Val func;
                    if (env.Vals.TryGetValue(expr.Id, out func))
                    {
                        return Apply(func, expr.Args, env);
                    }
                    else
                    {
                        throw new Exception("unknown func " + expr.Id);
                    }
                default:
                    throw new Exception("invalid expr type " + expr.Type);
            }
        }

        public static Val Apply(Val func, List<Expr> args, Env env)
        {
            switch (func.Type)
            {
                case Val.Types.Func:
                    if (func.Params.Count != args.Count)
                    {
                        throw new Exception("func call arg count mismatch");
                    }
                    else
                    {
                        Env funcEnv = env.Clone();
                        for (int i = 0; i < args.Count; ++i)
                        {
                            funcEnv.Vals[func.Params[i]] = Eval(args[i], env);
                        }
                        return func.Body.Select(expr => Eval(expr, funcEnv)).Last();
                    }
                    throw new NotImplementedException();
                default:
                    throw new Exception("invalid func type " + func.Type);
            }
        }
    }
}
