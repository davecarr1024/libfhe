using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class Parser
    {
        public class Rule
        {
            public enum Types
            {
                AND,
                OR,
                ONEORMORE,
                ZEROORMORE,
                TERMINAL,
            }

            public Types Type { get; set; }

            public string Name { get; set; }

            public List<Rule> Children { get; set; }

            public Rule(Types type, string name, params Rule[] children)
            {
                Type = type;
                Name = name;
                Children = children.ToList();
            }

            public override string ToString()
            {
                return string.Format("<Parser.Rule Type=\"{0}\" Name=\"{1}\">", Type, Name);
            }

            public Result Apply(List<Lexer.Result> tokens, ref int tokenPos)
            {
                switch (Type)
                {
                    case Types.AND:
                        {
                            Result result = new Result(this);
                            foreach (Rule child in Children)
                            {
                                Result childResult = child.Apply(tokens, ref tokenPos);
                                if (childResult == null)
                                {
                                    return null;
                                }
                                else
                                {
                                    result.Children.Add(childResult);
                                }
                            }
                            return result;
                        }
                    case Types.OR:
                        foreach (Rule child in Children)
                        {
                            int childTokenPos = tokenPos;
                            Result childResult = child.Apply(tokens, ref childTokenPos);
                            if (childResult != null)
                            {
                                tokenPos = childTokenPos;
                                return new Result(this, childResult);
                            }
                        }
                        return null;
                    case Types.ONEORMORE:
                        if (Children.Count != 1)
                        {
                            throw new Exception("one or more rules must have one child");
                        }
                        else
                        {
                            Result childResult = Children.First().Apply(tokens, ref tokenPos);
                            if (childResult == null)
                            {
                                return null;
                            }
                            else
                            {
                                Result result = new Result(this, childResult);
                                while (true)
                                {
                                    int childTokenPos = tokenPos;
                                    childResult = Children.First().Apply(tokens, ref childTokenPos);
                                    if (childResult == null)
                                    {
                                        return result;
                                    }
                                    else
                                    {
                                        tokenPos = childTokenPos;
                                        result.Children.Add(childResult);
                                    }
                                }
                            }
                        }
                    case Types.ZEROORMORE:
                        if (Children.Count != 1)
                        {
                            throw new Exception("zero or more rules must have one child");
                        }
                        else
                        {
                            Result result = new Result(this);
                            while (true)
                            {
                                int childTokenPos = tokenPos;
                                Result childResult = Children.First().Apply(tokens, ref childTokenPos);
                                if (childResult == null)
                                {
                                    return result;
                                }
                                else
                                {
                                    tokenPos = childTokenPos;
                                    result.Children.Add(childResult);
                                }
                            }
                        }
                    case Types.TERMINAL:
                        if (tokenPos < tokens.Count && tokens[tokenPos].Rule.Name == Name)
                        {
                            Result result = new Result(this, tokens[tokenPos].Value);
                            tokenPos++;
                            return result;
                        }
                        else
                        {
                            return null;
                        }
                    default:
                        throw new Exception("unknown Parser.Rule type " + Type);
                }
            }
        }

        public class Result
        {
            public Rule Rule { get; set; }

            public string Value { get; set; }

            public List<Result> Children { get; set; }

            public Result(Rule rule, string value)
            {
                Rule = rule;
                Value = value;
                Children = null;
            }

            public Result(Rule rule, params Result[] children)
            {
                Rule = rule;
                Value = null;
                Children = children.ToList();
            }

            public override string ToString()
            {
                return string.Format("<Parser.Result Rule=\"{0}\" Value=\"{1}\">", Rule.Name, Value);
            }
        }

        private class UnboundRule
        {
            public Rule.Types Type { get; set; }

            public string Name { get; set; }

            public List<string> Children { get; set; }

            public UnboundRule(Rule.Types type, string name, params string[] children)
            {
                Type = type;
                Name = name;
                Children = children.ToList();
            }
        }

        public Lexer Lexer { get; set; }

        public Rule Root { get; set; }

        public Parser(Lexer lexer, Rule root)
        {
            Lexer = lexer;
            Root = root;
        }

        public Parser(string grammarStr)
        {
            /*
             * pipe = '|';
             * plus = '+';
             * star = '*';
             * tilde = '~';
             * equals = '=';
             * semicolon = ';';
             * gt = '>';
             * str = '\'.*\'';
             * id = '[a-zA-Z0-9_-]+';
             * ws ~= '\s+';
             * grammar => rule*;
             * rule => lexerRule | negLexerRule | parserRule;
             * lexerRule => id equals str semicolon;
             * negLexerRule => id tilde equals str semicolon;
             * parserRule => id equals gt parserRuleImpl semicolon;
             * parserRuleImpl => andRule | orRule | oneOrMoreRule | zeroOrMoreRule;
             * andRule => id andRuleTail;
             * andRuleTail => id+;
             * orRule => id orRuleTail;
             * orRuleTail => orRuleTailInc+;
             * orRuleTailInc => pipe id;
             * oneOrMoreRule => id plus;
             * zeroOrMoreRule => id star;
             */
            Parser grammarParser = new Parser(
                new Lexer(
                    new Lexer.Rule("pipe", @"\|", true),
                    new Lexer.Rule("plus", @"\+", true),
                    new Lexer.Rule("star", @"\*", true),
                    new Lexer.Rule("tilde", @"\~", true),
                    new Lexer.Rule("equals", @"=", true),
                    new Lexer.Rule("semicolon", @";", true),
                    new Lexer.Rule("gt", @">", true),
                    new Lexer.Rule("str", @"'.*'", true),
                    new Lexer.Rule("id", @"[a-zA-Z0-9_-]+", true),
                    new Lexer.Rule("ws", @"\s+", false)
                ),
                new Rule(Rule.Types.ONEORMORE, "grammar",
                    new Rule(Rule.Types.OR, "rule",
                        new Rule(Rule.Types.AND, "lexerRule",
                            new Rule(Rule.Types.TERMINAL, "id"),
                            new Rule(Rule.Types.TERMINAL, "equals"),
                            new Rule(Rule.Types.TERMINAL, "str"),
                            new Rule(Rule.Types.TERMINAL, "semicolon")
                        ),
                        new Rule(Rule.Types.AND, "negLexerRule",
                            new Rule(Rule.Types.TERMINAL, "id"),
                            new Rule(Rule.Types.TERMINAL, "tilde"),
                            new Rule(Rule.Types.TERMINAL, "equals"),
                            new Rule(Rule.Types.TERMINAL, "str"),
                            new Rule(Rule.Types.TERMINAL, "semicolon")
                        ),
                        new Rule(Rule.Types.AND, "parserRule",
                            new Rule(Rule.Types.TERMINAL, "id"),
                            new Rule(Rule.Types.TERMINAL, "equals"),
                            new Rule(Rule.Types.TERMINAL, "gt"),
                            new Rule(Rule.Types.OR, "parserRuleImpl",
                                new Rule(Rule.Types.AND, "andRule",
                                    new Rule(Rule.Types.TERMINAL, "id"),
                                    new Rule(Rule.Types.ONEORMORE, "andRuleRail",
                                        new Rule(Rule.Types.TERMINAL, "id")
                                    )
                                ),
                                new Rule(Rule.Types.AND, "oneOrMoreRule",
                                    new Rule(Rule.Types.TERMINAL, "id"),
                                    new Rule(Rule.Types.TERMINAL, "plus")
                                ),
                                new Rule(Rule.Types.AND, "zeroOrMoreRule",
                                    new Rule(Rule.Types.TERMINAL, "id"),
                                    new Rule(Rule.Types.TERMINAL, "star")
                                ),
                                new Rule(Rule.Types.AND, "orRule",
                                    new Rule(Rule.Types.TERMINAL, "id"),
                                    new Rule(Rule.Types.ONEORMORE, "orRuleTail",
                                        new Rule(Rule.Types.AND, "orRuleInc",
                                            new Rule(Rule.Types.TERMINAL, "pipe"),
                                            new Rule(Rule.Types.TERMINAL, "id")
                                        )
                                    )
                                )
                            ),
                            new Rule(Rule.Types.TERMINAL, "semicolon")
                        )
                    )
                )
            );

            Result grammar = grammarParser.Parse(grammarStr);
            Dictionary<string, UnboundRule> unboundRules = new Dictionary<string, UnboundRule>();
            string rootRuleName = null;
            Lexer = new Lexer();
            foreach (Parser.Result ruleParent in grammar.Children)
            {
                Result rule = ruleParent.Children.First();
                switch (rule.Rule.Name)
                {
                    case "lexerRule":
                        {
                            string name = rule.Children[0].Value;
                            string pattern = rule.Children[2].Value;
                            Lexer.Rules.Add(new Lexer.Rule(
                                name,
                                pattern.Substring(1, pattern.Length - 2),
                                true));
                            break;
                        }
                    case "negLexerRule":
                        {
                            string name = rule.Children[0].Value;
                            string pattern = rule.Children[3].Value;
                            Lexer.Rules.Add(new Lexer.Rule(
                                name,
                                pattern.Substring(1, pattern.Length - 2),
                                false));
                            break;
                        }
                    case "parserRule":
                        {
                            string name = rule.Children[0].Value;
                            if (rootRuleName == null)
                            {
                                rootRuleName = name;
                            }
                            Result parserRule = rule.Children[3].Children[0];
                            switch (parserRule.Rule.Name)
                            {
                                case "andRule":
                                    {
                                        UnboundRule unboundRule = new UnboundRule(Rule.Types.AND, name, parserRule.Children[0].Value);
                                        foreach (Result child in parserRule.Children[1].Children)
                                        {
                                            unboundRule.Children.Add(child.Value);
                                        }
                                        unboundRules[name] = unboundRule;
                                        break;
                                    }
                                case "oneOrMoreRule":
                                    unboundRules[name] = new UnboundRule(Rule.Types.ONEORMORE, name, parserRule.Children[0].Value);
                                    break;
                                case "zeroOrMoreRule":
                                    unboundRules[name] = new UnboundRule(Rule.Types.ZEROORMORE, name, parserRule.Children[0].Value);
                                    break;
                                case "orRule":
                                    {
                                        UnboundRule unboundRule = new UnboundRule(Rule.Types.OR, name, parserRule.Children[0].Value);
                                        foreach (Result tail in parserRule.Children[1].Children)
                                        {
                                            unboundRule.Children.Add(tail.Children[1].Value);
                                        }
                                        unboundRules[name] = unboundRule;
                                        break;
                                    }
                                default:
                                    throw new Exception("unknown parserRule type " + parserRule.Rule.Name);
                            }
                            break;
                        }
                    default:
                        throw new Exception("unknown rule type " + rule.Rule.Name);
                }
            }

            if (rootRuleName == null)
            {
                throw new Exception("no root rule");
            }

            Dictionary<string, Rule> boundRules = new Dictionary<string, Rule>();
            Root = Bind(unboundRules, boundRules, rootRuleName);
        }

        public Result Parse(string input)
        {
            List<Lexer.Result> tokens = Lexer.Lex(input);
            int tokenPos = 0;
            Result result = Root.Apply(tokens, ref tokenPos);
            if (tokenPos != tokens.Count)
            {
                throw new Exception("parse underflow");
            }
            else
            {
                return result;
            }
        }

        private Rule Bind(Dictionary<string, UnboundRule> unboundRules, Dictionary<string, Rule> boundRules, string name)
        {
            UnboundRule unboundRule;
            Rule boundRule;
            if (boundRules.TryGetValue(name, out boundRule))
            {
                return boundRule;
            }
            else if (unboundRules.TryGetValue(name, out unboundRule))
            {
                boundRules[name] = boundRule = new Rule(unboundRule.Type, unboundRule.Name);
                foreach (string childName in unboundRule.Children)
                {
                    boundRule.Children.Add(Bind(unboundRules, boundRules, childName));
                }
                return boundRule;
            }
            else
            {
                Lexer.Rule lexerRule = Lexer.Rules.FirstOrDefault(lr => lr.Name == name);
                if (lexerRule != null)
                {
                    boundRules[name] = boundRule = new Rule(Rule.Types.TERMINAL, name);
                    return boundRule;
                }
                else
                {
                    throw new Exception("unknown rule " + name);
                }
            }
        }
    }
}
