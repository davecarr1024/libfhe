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

        public Parser(string input)
        {
            Lexer.Lexer lexer = new Lexer.Lexer(
                new Lexer.Rule("equals", @"=", true),
                new Lexer.Rule("semicolon", @";", true),
                new Lexer.Rule("tilde", @"~", true),
                new Lexer.Rule("lparen", @"\(", true),
                new Lexer.Rule("rparen", @"\)", true),
                new Lexer.Rule("plus", @"\+", true),
                new Lexer.Rule("star", @"\*", true),
                new Lexer.Rule("bar", @"\|", true),
                new Lexer.Rule("question", @"\?", true),
                new Lexer.Rule("gt", @">", true),
                new Lexer.Rule("id", @"[a-zA-Z_][a-zA-Z0-9_]*", true),
                new Lexer.Rule("str", @"'.*'", true),
                new Lexer.Rule("ws", @"\s+", false)
            );

            Parser parser = new Parser(
                lexer,
                RuleDecl.Bind(
                    lexer,
                    new RuleDecl("grammar", Rule.Types.OneOrMore, "decl"),
                    new RuleDecl("decl", Rule.Types.Or, "lexerDecl", "negLexerDecl", "parserDecl"),
                    new RuleDecl("lexerDecl", Rule.Types.And, "id", "equals", "str", "semicolon"),
                    new RuleDecl("negLexerDecl", Rule.Types.And, "id", "tilde", "equals", "str", "semicolon"),
                    new RuleDecl("parserDecl", Rule.Types.And, "id", "equals", "gt", "parserRule", "semicolon"),
                    new RuleDecl("parserRule", Rule.Types.Or, "andRule", "orRule", "oneOrMoreRule", "zeroOrMoreRule", "zeroOrOneRule"),
                    new RuleDecl("andRule", Rule.Types.And, "id", "andRuleTail"),
                    new RuleDecl("andRuleTail", Rule.Types.OneOrMore, "id"),
                    new RuleDecl("orRule", Rule.Types.And, "id", "orRuleTail"),
                    new RuleDecl("orRuleTail", Rule.Types.OneOrMore, "orRuleIter"),
                    new RuleDecl("orRuleIter", Rule.Types.And, "bar", "id"),
                    new RuleDecl("oneOrMoreRule", Rule.Types.And, "id", "plus"),
                    new RuleDecl("zeroOrMoreRule", Rule.Types.And, "id", "star"),
                    new RuleDecl("zeroOrOneRule", Rule.Types.And, "id", "question")
                )
            );
            Result grammar = parser.Parse(input);

            Lexer = new Lexer.Lexer();

            Dictionary<string, RuleDecl> unboundRules = new Dictionary<string, RuleDecl>();
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
                            RuleDecl rule = new RuleDecl(decl);
                            unboundRules[rule.Name] = rule;
                            if (rootName == null)
                            {
                                rootName = rule.Name;
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
                Root = RuleDecl.Bind(rootName, Lexer, unboundRules);
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
