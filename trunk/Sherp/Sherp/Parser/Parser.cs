using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Parser
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

        public Parser(Lexer.Lexer lexer, string rootName, Dictionary<string, RuleExprs.RuleExpr> unboundRules)
        {
            Lexer = lexer;
            InitRoot(rootName, unboundRules);
        }

        private void InitRoot(string rootName, Dictionary<string, RuleExprs.RuleExpr> unboundRules)
        {
            foreach (KeyValuePair<string, RuleExprs.RuleExpr> val in unboundRules)
            {
                val.Value.Name = val.Key;
            }
            Root = unboundRules[rootName].Bind(new Dictionary<string, Rule>(), unboundRules, Lexer);
        }

        public Parser(string input)
        {
            Lexer.Lexer lexer = new Lexer.Lexer(
                new Lexer.Rule("id", @"[a-zA-Z_][a-zA-Z0-9_]*", true),
                new Lexer.Rule("str", @"'.*?'", true),
                new Lexer.Rule("ws", @"\s+", false)
            );

            Parser parser = new Parser(
                lexer,
                "grammar",
                new Dictionary<string, RuleExprs.RuleExpr>()
                {
                    { "grammar", 
                        new RuleExprs.ParserRule(Rule.Types.OneOrMore,
                            new RuleExprs.RuleRef("decl")
                        )
                    },
                    { "decl", 
                        new RuleExprs.ParserRule(Rule.Types.Or, 
                            new RuleExprs.RuleRef("lexerDecl"),
                            new RuleExprs.RuleRef("negLexerDecl"),
                            new RuleExprs.RuleRef("parserDecl")
                        )
                    },
                    { "lexerDecl", 
                        new RuleExprs.ParserRule(Rule.Types.And, 
                            new RuleExprs.RuleRef("id"), 
                            new RuleExprs.LexerRule("="),
                            new RuleExprs.RuleRef("str"), 
                            new RuleExprs.LexerRule(";")
                        )
                    },
                    { "negLexerDecl", 
                        new RuleExprs.ParserRule(Rule.Types.And,
                            new RuleExprs.RuleRef("id"),
                            new RuleExprs.LexerRule(@"\~"),
                            new RuleExprs.LexerRule("="),
                            new RuleExprs.RuleRef("str"), 
                            new RuleExprs.LexerRule(";")
                        )
                    },
                    { "parserDecl", 
                        new RuleExprs.ParserRule(Rule.Types.And,
                            new RuleExprs.RuleRef("id"), 
                            new RuleExprs.LexerRule("="),
                            new RuleExprs.LexerRule(">"),
                            new RuleExprs.RuleRef("parserRule"), 
                            new RuleExprs.LexerRule(";")
                        )
                    },
                    { "parserRule", 
                        new RuleExprs.ParserRule(Rule.Types.Or, 
                            new RuleExprs.RuleRef("andRule"), 
                            new RuleExprs.RuleRef("orRule"), 
                            new RuleExprs.RuleRef("oneOrMoreRule"), 
                            new RuleExprs.RuleRef("zeroOrMoreRule"), 
                            new RuleExprs.RuleRef("zeroOrOneRule")
                        )
                    },
                    { "andRule", 
                        new RuleExprs.ParserRule(Rule.Types.And, 
                            new RuleExprs.RuleRef("andOrOperand"), 
                            new RuleExprs.ParserRule(Rule.Types.OneOrMore,
                                new RuleExprs.RuleRef("andOrOperand")
                            )
                        )
                    },
                    { "orRule",
                        new RuleExprs.ParserRule(Rule.Types.And,
                            new RuleExprs.RuleRef("andOrOperand"),
                            new RuleExprs.ParserRule(Rule.Types.OneOrMore,
                                new RuleExprs.ParserRule(Rule.Types.And,
                                    new RuleExprs.LexerRule(@"\|"),
                                    new RuleExprs.RuleRef("andOrOperand")
                                )
                            )
                        )
                    },
                    { "oneOrMoreRule",
                        new RuleExprs.ParserRule(Rule.Types.And,
                            new RuleExprs.RuleRef("parserOperand"),
                            new RuleExprs.LexerRule(@"\+")
                        )
                    },
                    { "zeroOrMoreRule",
                        new RuleExprs.ParserRule(Rule.Types.And,
                            new RuleExprs.RuleRef("parserOperand"),
                            new RuleExprs.LexerRule(@"\*")
                        )
                    },
                    { "zeroOrOneRule",
                        new RuleExprs.ParserRule(Rule.Types.And,
                            new RuleExprs.RuleRef("parserOperand"),
                            new RuleExprs.LexerRule(@"\?")
                        )
                    },
                    { "parserOperand",
                        new RuleExprs.ParserRule(Rule.Types.Or,
                            new RuleExprs.RuleRef("parenRule"),
                            new RuleExprs.RuleRef("str"),
                            new RuleExprs.RuleRef("id")
                        )
                    },
                    { "andOrOperand",
                        new RuleExprs.ParserRule(Rule.Types.Or,
                            new RuleExprs.RuleRef("oneOrMoreRule"),
                            new RuleExprs.RuleRef("zeroOrMoreRule"),
                            new RuleExprs.RuleRef("zeroOrOneRule"),
                            new RuleExprs.RuleRef("parenRule"),
                            new RuleExprs.RuleRef("str"),
                            new RuleExprs.RuleRef("id")
                        )
                    },
                    { "parenRule",
                        new RuleExprs.ParserRule(Rule.Types.And,
                            new RuleExprs.LexerRule(@"\("),
                            new RuleExprs.RuleRef("parserRule"),
                            new RuleExprs.LexerRule(@"\)")
                        )
                    },
                }
            );

            Result grammar = parser.Parse(input);

            Lexer = new Lexer.Lexer();

            Dictionary<string, RuleExprs.RuleExpr> unboundRules = new Dictionary<string, RuleExprs.RuleExpr>();
            string rootName = null;

            foreach (Result declParent in grammar.Children)
            {
                Result decl = declParent.Children[0];
                switch (decl.Rule.Name)
                {
                    case "lexerDecl":
                        {
                            string name = decl.Children[0].Value;
                            string patternStr = decl.Children[2].Value;
                            string pattern = patternStr.Substring(1, patternStr.Length - 2);
                            Lexer.Rules.Add(new Lexer.Rule(name, pattern, true));
                            break;
                        }
                    case "negLexerDecl":
                        {
                            string name = decl.Children[0].Value;
                            string patternStr = decl.Children[3].Value;
                            string pattern = patternStr.Substring(1, patternStr.Length - 2);
                            Lexer.Rules.Add(new Lexer.Rule(name, pattern, false));
                            break;
                        }
                    case "parserDecl":
                        {
                            string name = decl.Children[0].Value;
                            unboundRules[name] = InitRuleExpr(decl.Children[3]);
                            if (rootName == null)
                            {
                                rootName = name;
                            }
                        }
                        break;
                    default:
                        throw new Exception("invalid decl " + decl.Rule.Name);
                }
            }

            if (rootName == null)
            {
                throw new Exception("no root rule");
            }
            else
            {
                InitRoot(rootName, unboundRules);
            }
        }

        private RuleExprs.RuleExpr InitRuleExpr(Result expr)
        {
            switch (expr.Rule.Name)
            {
                case "parserRule":
                case "parserOperand":
                case "andOrOperand":
                    return InitRuleExpr(expr.Children[0]);
                case "andRule":
                    {
                        RuleExprs.ParserRule rule = new RuleExprs.ParserRule(Rule.Types.And, InitRuleExpr(expr.Children[0]));
                        rule.Children.AddRange(expr.Children[1].Children.Select(child => InitRuleExpr(child)));
                        return rule;
                    }
                case "orRule":
                    {
                        RuleExprs.ParserRule rule = new RuleExprs.ParserRule(Rule.Types.Or, InitRuleExpr(expr.Children[0]));
                        rule.Children.AddRange(expr.Children[1].Children.Select(child => InitRuleExpr(child.Children[1])));
                        return rule;
                    }
                case "oneOrMoreRule":
                    return new RuleExprs.ParserRule(Rule.Types.OneOrMore, InitRuleExpr(expr.Children[0]));
                case "zeroOrMoreRule":
                    return new RuleExprs.ParserRule(Rule.Types.ZeroOrMore, InitRuleExpr(expr.Children[0]));
                case "zeroOrOneRule":
                    return new RuleExprs.ParserRule(Rule.Types.ZeroOrOne, InitRuleExpr(expr.Children[0]));
                case "id":
                    return new RuleExprs.RuleRef(expr.Value);
                case "parenRule":
                    return InitRuleExpr(expr.Children[1]);
                case "str":
                    return new RuleExprs.LexerRule(expr.Value.Substring(1, expr.Value.Length - 2));
                default:
                    throw new Exception("invalid rule expr " + expr.Rule.Name);
            }
        }

        public Result Parse(string input)
        {
            List<Lexer.Result> tokens = Lexer.Lex(input);
            int pos = 0;
            Result result = Root.Apply(tokens, ref pos);
            if (pos != tokens.Count)
            {
                throw new Exception("parse error at " + string.Join(" ", tokens.Skip(pos).Take(25).Select(token => token.Value).ToArray()));
            }
            else
            {
                return result;
            }
        }
    }
}
