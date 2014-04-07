using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public static class Derp
    {
        private static Parser Parser = new Parser(@"
            lbrace = '\{';
            rbrace = '\}';
            lparen = '\(';
            rparen = '\)';
            semicolon = ';';
            comma = '\,';
            Add = '\+';
            Subtract = '\-';
            Multiply = '\*';
            Divide = '\/';
            Assign = '=';
            dot = '\.';
            def = 'def';
            class = 'class';
            float = '-?\d+\.\d+';
            int = '-?\d+';
            string = '"".*""';
            id = '[a-zA-Z0-9-_]+';
            ws ~= '\s+';
            program => statement+;
            statement => funcDecl | classDecl | line;
            funcDecl => def id lparen idList rparen lbrace program rbrace;
            idList => id idListTail;
            idListTail => idListIter*;
            idListIter => comma id;
            line => expr semicolon;
            expr => binaryOperation | call | callEmpty | literal | parenExpr | ref;
            callEmpty => ref lparen rparen;
            call => ref lparen exprList rparen;
            exprList => expr exprListTail;
            exprListTail => exprListIter*;
            exprListIter => comma expr;
            binaryOperation => operand binaryOperator operand;
            binaryOperator => Add | Subtract | Multiply | Divide | Assign;
            operand => call | callEmpty | parenExpr | literal | ref;
            literal => int | float | string;
            parenExpr => lparen expr rparen;
            classDecl => class id lbrace classDeclBody rbrace;
            classDeclBody => statement*;
            ref => id refTail;
            refTail => refIter*;
            refIter => dot id;
        ");

        public static Val Eval(string input)
        {
            return Eval(input, new Scope());
        }

        public static Val Eval(string input, Scope scope)
        {
            return Parser.Parse(input).Children.Select(expr => InitExpr(expr).Eval(scope)).Last();
        }

        private static Expr InitExpr(Parser.Result expr)
        {
            switch (expr.Rule.Name)
            {
                case "statement":
                case "line":
                case "expr":
                case "literal":
                case "operand":
                case "funcRef":
                    return InitExpr(expr.Children[0]);
                case "parenExpr":
                    return InitExpr(expr.Children[1]);
                case "int":
                    return new IntExpr(int.Parse(expr.Value));
                case "float":
                    return new FloatExpr(float.Parse(expr.Value));
                case "string":
                    return new StringExpr(expr.Value.Substring(1, expr.Value.Length - 2));
                case "call":
                    {
                        Expr func = InitExpr(expr.Children[0]);
                        Parser.Result exprList = expr.Children[2];
                        List<Expr> args = new List<Expr>() { InitExpr(exprList.Children[0]) };
                        foreach (Parser.Result exprIter in exprList.Children[1].Children)
                        {
                            args.Add(InitExpr(exprIter.Children[1]));
                        }
                        return new CallExpr(func, args);
                    }
                case "callEmpty":
                    return new CallExpr(InitExpr(expr.Children[0]), new List<Expr>());
                case "funcDecl":
                    {
                        string name = expr.Children[1].Value;
                        Parser.Result idList = expr.Children[3];
                        List<string> paramList = new List<string>() { idList.Children[0].Value };
                        foreach (Parser.Result param in idList.Children[1].Children)
                        {
                            paramList.Add(param.Children[1].Value);
                        }
                        List<Expr> body = expr.Children[6].Children.Select(child => InitExpr(child)).ToList();
                        return new FuncDeclExpr(name, paramList, body);
                    }
                case "binaryOperation":
                    return new CallExpr(InitExpr(expr.Children[1]), new List<Expr>() { InitExpr(expr.Children[0]), InitExpr(expr.Children[2]) });
                case "binaryOperator":
                    return new RefExpr(expr.Children[0].Rule.Name);
                case "classDecl":
                    return new ClassDeclExpr(expr.Children[1].Value, expr.Children[3].Children.Select(child => InitExpr(child)).ToList());
                case "ref":
                    {
                        List<string> objectIds = new List<string>() { expr.Children[0].Value };
                        objectIds.AddRange(expr.Children[1].Children.Select(child => child.Children[1].Value));
                        return new RefExpr(objectIds.ToArray());
                    }
                default:
                    throw new Exception("invalid expr type " + expr.Rule.Name);
            }
        }

        internal static Val Assign(List<Expr> args, Scope scope)
        {
            if (args.Count == 2 && args[0] is RefExpr)
            {
                RefExpr refArg = args[0] as RefExpr;
                return refArg.Resolve(scope).Vals[refArg.Ids.Last()] = args[1].Eval(scope);
            }
            else
            {
                throw new Exception("invalid Assign args");
            }
        }

        internal static Val Add(List<Expr> args, Scope scope)
        {
            List<Val> vals = args.Select(arg => arg.Eval(scope)).ToList();
            if (vals.Count == 2 && vals[0] is IntVal && vals[1] is IntVal)
            {
                return new IntVal((vals[0] as IntVal).Value + (vals[1] as IntVal).Value);
            }
            else
            {
                throw new Exception("invalid Add args");
            }
        }

        internal static Val Subtract(List<Expr> args, Scope scope)
        {
            List<Val> vals = args.Select(arg => arg.Eval(scope)).ToList();
            if (vals.Count == 2 && vals[0] is IntVal && vals[1] is IntVal)
            {
                return new IntVal((vals[0] as IntVal).Value - (vals[1] as IntVal).Value);
            }
            else
            {
                throw new Exception("invalid Subtract args");
            }
        }

        internal static Val Multiply(List<Expr> args, Scope scope)
        {
            List<Val> vals = args.Select(arg => arg.Eval(scope)).ToList();
            if (vals.Count == 2 && vals[0] is IntVal && vals[1] is IntVal)
            {
                return new IntVal((vals[0] as IntVal).Value * (vals[1] as IntVal).Value);
            }
            else
            {
                throw new Exception("invalid Multiply args");
            }
        }

        internal static Val Divide(List<Expr> args, Scope scope)
        {
            List<Val> vals = args.Select(arg => arg.Eval(scope)).ToList();
            if (vals.Count == 2 && vals[0] is IntVal && vals[1] is IntVal)
            {
                return new IntVal((vals[0] as IntVal).Value / (vals[1] as IntVal).Value);
            }
            else
            {
                throw new Exception("invalid Divide args");
            }
        }
    }
}
