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

        public Parser(Lexer.Lexer lexer, string rootName, Dictionary<string, RuleExpr> unboundRules)
        {
            Lexer = lexer;
            InitRoot(rootName, unboundRules);
        }

        private void InitRoot(string rootName, Dictionary<string, RuleExpr> unboundRules)
        {
            foreach (KeyValuePair<string, RuleExpr> val in unboundRules)
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
                new Dictionary<string, RuleExpr>()
                {
                    { "grammar", 
                        new RuleDecl(Rule.Types.OneOrMore,
                            new RuleRef("decl")
                        )
                    },
                    { "decl", 
                        new RuleDecl(Rule.Types.Or, 
                            new RuleRef("lexerDecl"),
                            new RuleRef("negLexerDecl"),
                            new RuleRef("parserDecl")
                        )
                    },
                    { "lexerDecl", 
                        new RuleDecl(Rule.Types.And, 
                            new RuleRef("id"), 
                            new LexerRule("="),
                            new RuleRef("str"), 
                            new LexerRule(";")
                        )
                    },
                    { "negLexerDecl", 
                        new RuleDecl(Rule.Types.And,
                            new RuleRef("id"),
                            new LexerRule(@"\~"),
                            new LexerRule("="),
                            new RuleRef("str"), 
                            new LexerRule(";")
                        )
                    },
                    { "parserDecl", 
                        new RuleDecl(Rule.Types.And,
                            new RuleRef("id"), 
                            new LexerRule("="),
                            new LexerRule(">"),
                            new RuleRef("parserRule"), 
                            new LexerRule(";")
                        )
                    },
                    { "parserRule", 
                        new RuleDecl(Rule.Types.Or, 
                            new RuleRef("andRule"), 
                            new RuleRef("orRule"), 
                            new RuleRef("oneOrMoreRule"), 
                            new RuleRef("zeroOrMoreRule"), 
                            new RuleRef("zeroOrOneRule")
                        )
                    },
                    { "andRule", 
                        new RuleDecl(Rule.Types.And, 
                            new RuleRef("andOrOperand"), 
                            new RuleDecl(Rule.Types.OneOrMore,
                                new RuleRef("andOrOperand")
                            )
                        )
                    },
                    { "orRule",
                        new RuleDecl(Rule.Types.And,
                            new RuleRef("andOrOperand"),
                            new RuleDecl(Rule.Types.OneOrMore,
                                new RuleDecl(Rule.Types.And,
                                    new LexerRule(@"\|"),
                                    new RuleRef("andOrOperand")
                                )
                            )
                        )
                    },
                    { "oneOrMoreRule",
                        new RuleDecl(Rule.Types.And,
                            new RuleRef("parserOperand"),
                            new LexerRule(@"\+")
                        )
                    },
                    { "zeroOrMoreRule",
                        new RuleDecl(Rule.Types.And,
                            new RuleRef("parserOperand"),
                            new LexerRule(@"\*")
                        )
                    },
                    { "zeroOrOneRule",
                        new RuleDecl(Rule.Types.And,
                            new RuleRef("parserOperand"),
                            new LexerRule(@"\?")
                        )
                    },
                    { "parserOperand",
                        new RuleDecl(Rule.Types.Or,
                            new RuleRef("parenRule"),
                            new RuleRef("str"),
                            new RuleRef("id")
                        )
                    },
                    { "andOrOperand",
                        new RuleDecl(Rule.Types.Or,
                            new RuleRef("oneOrMoreRule"),
                            new RuleRef("zeroOrMoreRule"),
                            new RuleRef("zeroOrOneRule"),
                            new RuleRef("parenRule"),
                            new RuleRef("str"),
                            new RuleRef("id")
                        )
                    },
                    { "parenRule",
                        new RuleDecl(Rule.Types.And,
                            new LexerRule(@"\("),
                            new RuleRef("parserRule"),
                            new LexerRule(@"\)")
                        )
                    },
                }
            );

            Result grammar = parser.Parse(input);

            Lexer = new Lexer.Lexer();

            Dictionary<string, RuleExpr> unboundRules = new Dictionary<string, RuleExpr>();
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

        private RuleExpr InitRuleExpr(Result expr)
        {
            switch (expr.Rule.Name)
            {
                case "parserRule":
                case "parserOperand":
                case "andOrOperand":
                    return InitRuleExpr(expr.Children[0]);
                case "andRule":
                    {
                        RuleDecl rule = new RuleDecl(Rule.Types.And, InitRuleExpr(expr.Children[0]));
                        rule.Children.AddRange(expr.Children[1].Children.Select(child => InitRuleExpr(child)));
                        return rule;
                    }
                case "orRule":
                    {
                        RuleDecl rule = new RuleDecl(Rule.Types.Or, InitRuleExpr(expr.Children[0]));
                        rule.Children.AddRange(expr.Children[1].Children.Select(child => InitRuleExpr(child.Children[1])));
                        return rule;
                    }
                case "oneOrMoreRule":
                    return new RuleDecl(Rule.Types.OneOrMore, InitRuleExpr(expr.Children[0]));
                case "zeroOrMoreRule":
                    return new RuleDecl(Rule.Types.ZeroOrMore, InitRuleExpr(expr.Children[0]));
                case "zeroOrOneRule":
                    return new RuleDecl(Rule.Types.ZeroOrOne, InitRuleExpr(expr.Children[0]));
                case "id":
                    return new RuleRef(expr.Value);
                case "parenRule":
                    return InitRuleExpr(expr.Children[1]);
                case "str":
                    return new LexerRule(expr.Value.Substring(1, expr.Value.Length - 2));
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
