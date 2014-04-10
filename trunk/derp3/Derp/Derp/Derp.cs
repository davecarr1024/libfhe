using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

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
            add = '\+';
            subtract = '\-';
            multiply = '\*';
            divide = '\/';
            equals = '==';
            assign = '=';
            not = '\!';
            dot = '\.';
            gte = '>=';
            gt = '>';
            lte = '<=';
            lt = '<';
            def = 'def';
            class = 'class';
            return = 'return';
            if = 'if';
            else = 'else';
            while = 'while';
            for = 'for';
            float = '\d+\.\d+';
            int = '\d+';
            string = '"".*?""';
            id = '[a-zA-Z0-9-_]+';
            ws ~= '\s+';
            program => statement+;
            statement => funcDecl | classDecl | line | ifElseStatement | ifStatement | whileStatement | forStatement;
            funcDecl => def id lparen idList rparen lbrace program rbrace;
            idList => id idListTail;
            idListTail => idListIter*;
            idListIter => comma id;
            line => lineExpr semicolon;
            lineExpr => returnExpr | expr;
            returnExpr => return expr;
            expr => unaryOperation | binaryOperation | call | callEmpty | literal | ref;
            callEmpty => ref lparen rparen;
            call => ref lparen exprList rparen;
            exprList => expr exprListTail;
            exprListTail => exprListIter*;
            exprListIter => comma expr;
            binaryOperation => operand binaryOperator operand;
            binaryOperator => add | subtract | multiply | divide | equals | assign | gte | gt | lte | lt;
            unaryOperation => unaryOperator operand;
            unaryOperator => not | subtract;
            operand => call | callEmpty | parenExpr | literal | ref;
            literal => int | float | string;
            parenExpr => lparen expr rparen;
            classDecl => class id compoundStatement;
            compoundStatement => lbrace compoundStatementBody rbrace;
            compoundStatementBody => statement*;
            ref => id refTail;
            refTail => refIter*;
            refIter => dot id;
            ifStatement => if parenExpr compoundStatement;
            ifElseStatement => if parenExpr compoundStatement else compoundStatement;
            whileStatement => while parenExpr compoundStatement;
            forStatement => for lparen expr semicolon expr semicolon expr rparen compoundStatement;
        ");

        public static Scope DefaultScope()
        {
            Scope scope = new Scope();
            scope["None"] = new Vals.NoneType();
            scope["True"] = new Vals.Bool(true);
            scope["False"] = new Vals.Bool(false);
            scope["Assert"] = new Vals.SystemBuiltin((args, s) =>
            {
                foreach (Expr arg in args)
                {
                    if (!arg.Eval(s).AsBool())
                    {
                        throw new Exception("assertion " + arg + " failed");
                    }
                }
                return new Vals.NoneType();
            });

            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes().Where(t => t.GetCustomAttributes().Any(attr => attr is BuiltinClass)))
            {
                scope[type.Name] = Vals.Class.Bind(type);
            }

            return scope;
        }

        public static Val Eval(string input)
        {
            return Eval(input, DefaultScope());
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
                case "lineExpr":
                    return InitExpr(expr.Children[0]);
                case "parenExpr":
                    return InitExpr(expr.Children[1]);
                case "int":
                    return new Exprs.Int(int.Parse(expr.Value));
                case "float":
                    return new Exprs.Float(float.Parse(expr.Value));
                case "string":
                    return new Exprs.String(expr.Value.Substring(1, expr.Value.Length - 2));
                case "call":
                    {
                        Expr func = InitExpr(expr.Children[0]);
                        Parser.Result exprList = expr.Children[2];
                        List<Expr> args = new List<Expr>() { InitExpr(exprList.Children[0]) };
                        foreach (Parser.Result exprIter in exprList.Children[1].Children)
                        {
                            args.Add(InitExpr(exprIter.Children[1]));
                        }
                        return new Exprs.Call(func, args);
                    }
                case "callEmpty":
                    return new Exprs.Call(InitExpr(expr.Children[0]), new List<Expr>());
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
                        return new Exprs.FuncDecl(name, paramList, body);
                    }
                case "binaryOperation":
                    {
                        Exprs.BinaryOperation.Operators op;
                        string operatorName = expr.Children[1].Children[0].Rule.Name;
                        switch (operatorName)
                        {
                            case "add":
                                op = Exprs.BinaryOperation.Operators.Add;
                                break;
                            case "subtract":
                                op = Exprs.BinaryOperation.Operators.Subtract;
                                break;
                            case "multiply":
                                op = Exprs.BinaryOperation.Operators.Multiply;
                                break;
                            case "divide":
                                op = Exprs.BinaryOperation.Operators.Divide;
                                break;
                            case "assign":
                                op = Exprs.BinaryOperation.Operators.Assign;
                                break;
                            case "equals":
                                op = Exprs.BinaryOperation.Operators.Equals;
                                break;
                            case "lte":
                                op = Exprs.BinaryOperation.Operators.LessThanOrEqual;
                                break;
                            case "lt":
                                op = Exprs.BinaryOperation.Operators.LessThan;
                                break;
                            case "gte":
                                op = Exprs.BinaryOperation.Operators.GreaterThanOrEqual;
                                break;
                            case "gt":
                                op = Exprs.BinaryOperation.Operators.GreaterThan;
                                break;
                            default:
                                throw new Exception("invalid binary operator " + operatorName);
                        }
                        return new Exprs.BinaryOperation(op, InitExpr(expr.Children[0]), InitExpr(expr.Children[2]));
                    }
                case "unaryOperation":
                    {
                        string operatorName = expr.Children[0].Children[0].Rule.Name;
                        Exprs.UnaryOperation.Operators op;
                        switch (operatorName)
                        {
                            case "not":
                                op = Exprs.UnaryOperation.Operators.Not;
                                break;
                            case "subtract":
                                op = Exprs.UnaryOperation.Operators.Negative;
                                break;
                            default:
                                throw new Exception("invalid unary operator " + operatorName);
                        }
                        return new Exprs.UnaryOperation(op, InitExpr(expr.Children[1]));
                    }
                case "classDecl":
                    //classDecl => class id compoundStatement;
                    //compoundStatement => lbrace compoundStatementBody rbrace;
                    //compoundStatementBody => statement*;
                    return new Exprs.ClassDecl(expr.Children[1].Value, expr.Children[2].Children[1].Children.Select(child => InitExpr(child)).ToList());
                case "ref":
                    {
                        List<string> objectIds = new List<string>() { expr.Children[0].Value };
                        objectIds.AddRange(expr.Children[1].Children.Select(child => child.Children[1].Value));
                        return new Exprs.Ref(objectIds.ToArray());
                    }
                case "returnExpr":
                    return new Exprs.ReturnExpr(InitExpr(expr.Children[1]));
                case "ifStatement":
                    //ifStatement => if parenExpr compoundStatement;
                    //parenExpr => lparen expr rparen;
                    //compoundStatement => lbrace compoundStatementBody rbrace;
                    //compoundStatementBody => statement*;
                    return new Exprs.IfStatement(InitExpr(expr.Children[1].Children[1]), expr.Children[2].Children[1].Children.Select(child => InitExpr(child)).ToList(), new List<Expr>());
                case "ifElseStatement":
                    //ifElseStatement => if parenExpr compoundStatement else compoundStatement;
                    //parenExpr => lparen expr rparen;
                    //compoundStatement => lbrace compoundStatementBody rbrace;
                    //compoundStatementBody => statement*;
                    return new Exprs.IfStatement(
                        InitExpr(expr.Children[1].Children[1]),
                        expr.Children[2].Children[1].Children.Select(child => InitExpr(child)).ToList(),
                        expr.Children[4].Children[1].Children.Select(child => InitExpr(child)).ToList());
                case "whileStatement":
                    //whileStatement => while parenExpr compoundStatement;
                    //parenExpr => lparen expr rparen;
                    //compoundStatement => lbrace compoundStatementBody rbrace;
                    //compoundStatementBody => statement*;
                    return new Exprs.WhileStatement(
                        InitExpr(expr.Children[1].Children[1]),
                        expr.Children[2].Children[1].Children.Select(child => InitExpr(child)).ToList());
                case "forStatement":
                    //forStatement => for lparen expr semicolon expr semicolon expr rparen compoundStatement;
                    //compoundStatement => lbrace compoundStatementBody rbrace;
                    //compoundStatementBody => statement*;
                    return new Exprs.ForStatement(
                        InitExpr(expr.Children[2]),
                        InitExpr(expr.Children[4]),
                        InitExpr(expr.Children[6]),
                        expr.Children[8].Children[1].Children.Select(child => InitExpr(child)).ToList());
                default:
                    throw new Exception("invalid expr type " + expr.Rule.Name);
            }
        }
    }
}
