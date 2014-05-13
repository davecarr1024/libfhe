using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Parser
{
    public class Parser
    {
        public Lexer.Lexer Lexer { get; private set; }

        public Rule Root { get; private set; }

        public Parser(Lexer.Lexer lexer, Rule root)
        {
            Lexer = lexer;
            Root = root;
        }

        public Parser(Lexer.Lexer lexer, params Exprs.Expr[] exprList)
        {
            Bind(lexer, exprList.ToList());
        }

        private void Bind(Lexer.Lexer lexer, List<Exprs.Expr> exprList)
        {
            Lexer = lexer;

            Dictionary<string, Rule> rules = new Dictionary<string, Rule>();

            Dictionary<string, Exprs.Expr> exprs = new Dictionary<string, Exprs.Expr>();
            foreach (Exprs.Expr expr in exprList)
            {
                if (!string.IsNullOrEmpty(expr.Name))
                {
                    exprs[expr.Name] = expr;
                }
            }

            Dictionary<string, Lexer.Rule> lexerRules = new Dictionary<string, Lexer.Rule>();
            foreach (Lexer.Rule lexerRule in Lexer.Rules)
            {
                lexerRules[lexerRule.Name] = lexerRule;
            }

            foreach (Exprs.Expr expr in exprList)
            {
                expr.Resolve(rules, exprs, lexerRules);
            }

            Rule root;
            if (!exprList.Any() || string.IsNullOrEmpty(exprList.First().Name))
            {
                throw new Exception("no root rule found");
            }
            else if (!rules.TryGetValue(exprList.First().Name, out root))
            {
                throw new Exception("unable to find root " + exprList.First().Name);
            }
            else
            {
                Root = root;
            }

            Lexer.Rules.AddRange(lexerRules.Values.Where(lr => !Lexer.Rules.Contains(lr)));
        }

        public Parser(string grammar)
        {
            Result result = new Parser(
                new Lexer.Lexer(
                    new Lexer.Rule("id", @"[a-zA-Z][a-zA-Z0-9_-]*", true),
                    new Lexer.Rule("str", @"'.*?'", true),
                    new Lexer.Rule("ws", @"\s+", false)
                ),
                new Exprs.Def(Rule.Types.OneOrMore, "grammar",
                    new Exprs.Def(Rule.Types.Or, null,
                        new Exprs.Ref("lexerDecl"),
                        new Exprs.Ref("negLexerDecl"),
                        new Exprs.Ref("ruleDecl")
                    )
                ),
                new Exprs.Def(Rule.Types.And, "lexerDecl",
                    new Exprs.Ref("id"),
                    new Exprs.Token(@"="),
                    new Exprs.Ref("str"),
                    new Exprs.Token(@";")
                ),
                new Exprs.Def(Rule.Types.And, "negLexerDecl",
                    new Exprs.Ref("id"),
                    new Exprs.Token(@"\~="),
                    new Exprs.Ref("str"),
                    new Exprs.Token(@";")
                ),
                new Exprs.Def(Rule.Types.And, "ruleDecl",
                    new Exprs.Ref("id"),
                    new Exprs.Token(@"=>"),
                    new Exprs.Ref("rule"),
                    new Exprs.Token(@";")
                ),
                new Exprs.Def(Rule.Types.Or, "rule",
                    new Exprs.Ref("andRule"),
                    new Exprs.Ref("orRule"),
                    new Exprs.Ref("oneOrMoreRule"),
                    new Exprs.Ref("zeroOrMoreRule"),
                    new Exprs.Ref("zeroOrOneRule"),
                    new Exprs.Ref("parenRule"),
                    new Exprs.Ref("id"),
                    new Exprs.Ref("str")
                ),
                new Exprs.Def(Rule.Types.And, "andRule",
                    new Exprs.Ref("andOrArg"),
                    new Exprs.Def(Rule.Types.OneOrMore, null,
                        new Exprs.Ref("andOrArg")
                    )
                ),
                new Exprs.Def(Rule.Types.And, "orRule",
                    new Exprs.Ref("andOrArg"),
                    new Exprs.Def(Rule.Types.OneOrMore, null,
                        new Exprs.Def(Rule.Types.And, null,
                            new Exprs.Token(@"\|"),
                            new Exprs.Ref("andOrArg")
                        )
                    )
                ),
                new Exprs.Def(Rule.Types.Or, "andOrArg",
                    new Exprs.Ref("oneOrMoreRule"),
                    new Exprs.Ref("zeroOrMoreRule"),
                    new Exprs.Ref("zeroOrOneRule"),
                    new Exprs.Ref("parenRule"),
                    new Exprs.Ref("id"),
                    new Exprs.Ref("str")
                ),
                new Exprs.Def(Rule.Types.And, "oneOrMoreRule",
                    new Exprs.Ref("repArg"),
                    new Exprs.Token(@"\+")
                ),
                new Exprs.Def(Rule.Types.And, "zeroOrMoreRule",
                    new Exprs.Ref("repArg"),
                    new Exprs.Token(@"\*")
                ),
                new Exprs.Def(Rule.Types.And, "zeroOrOneRule",
                    new Exprs.Ref("repArg"),
                    new Exprs.Token(@"\?")
                ),
                new Exprs.Def(Rule.Types.Or, "repArg",
                    new Exprs.Ref("parenRule"),
                    new Exprs.Ref("id"),
                    new Exprs.Ref("str")
                ),
                new Exprs.Def(Rule.Types.And, "parenRule",
                    new Exprs.Token(@"\("),
                    new Exprs.Ref("rule"),
                    new Exprs.Token(@"\)")
                )
            ).Apply(grammar);

            Lexer = new Lexer.Lexer();
            List<Exprs.Expr> exprList = new List<Exprs.Expr>();

            foreach (Result line in result.Children)
            {
                Result decl = line.Children[0];
                switch (decl.Rule.Name)
                {
                    case "lexerDecl":
                        {
                            string value = decl.Children[2].Value;
                            Lexer.Rules.Add(new Lexer.Rule(decl.Children[0].Value, value.Substring(1, value.Length - 2), true));
                        }
                        break;
                    case "negLexerDecl":
                        {
                            string value = decl.Children[2].Value;
                            Lexer.Rules.Add(new Lexer.Rule(decl.Children[0].Value, value.Substring(1, value.Length - 2), false));
                        }
                        break;
                    case "ruleDecl":
                        Exprs.Expr expr = Parse(decl.Children[2]);
                        expr.Name = decl.Children[0].Value;
                        exprList.Add(expr);
                        break;
                    default:
                        throw new NotImplementedException(decl.Rule.Name);
                }
            }

            Bind(Lexer, exprList);
        }

        private static Exprs.Expr Parse(Result result)
        {
            switch (result.Rule.Name)
            {
                case "rule":
                case "repArg":
                case "andOrArg":
                    return Parse(result.Children[0]);
                case "andRule":
                    {
                        //andRule => andOrArg andOrArg+
                        List<Exprs.Expr> children = new List<Exprs.Expr>() { Parse(result.Children[0]) };
                        children.AddRange(result.Children[1].Children.Select(child => Parse(child)));
                        return new Exprs.Def(Rule.Types.And, null, children.ToArray());
                    }
                case "orRule":
                    {
                        //orRule => andOrArg ( '\|' andOrArg )+;
                        List<Exprs.Expr> children = new List<Exprs.Expr>() { Parse(result.Children[0]) };
                        children.AddRange(result.Children[1].Children.Select(child => Parse(child.Children[1])));
                        return new Exprs.Def(Rule.Types.Or, null, children.ToArray());
                    }
                case "oneOrMoreRule":
                    return new Exprs.Def(Rule.Types.OneOrMore, null, Parse(result.Children[0]));
                case "zeroOrMoreRule":
                    return new Exprs.Def(Rule.Types.ZeroOrMore, null, Parse(result.Children[0]));
                case "zeroOrOneRule":
                    return new Exprs.Def(Rule.Types.ZeroOrOne, null, Parse(result.Children[0]));
                case "id":
                    return new Exprs.Ref(result.Value);
                case "str":
                    return new Exprs.Token(result.Value.Substring(1, result.Value.Length - 2));
                default:
                    throw new NotImplementedException(result.Rule.Name);
            }
        }

        public Result Apply(string input)
        {
            List<Lexer.Result> tokens = Lexer.Apply(input);
            int tokenPos = 0;
            Result result = Root.Apply(tokens, ref tokenPos);
            if (tokenPos < tokens.Count)
            {
                throw new Exception("parse error at " + string.Join(" ", tokens.Skip(tokenPos).Take(50).Select(token => token.Value).ToArray()));
            }
            else
            {
                return result;
            }
        }
    }
}
