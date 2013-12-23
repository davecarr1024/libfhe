using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp
{
    public class Parser
    {
        public class Result
        {
            public Rule Rule { get; set; }

            public List<Result> Children { get; set; }

            public string Value { get; set; }
        }

        public class Rule
        {
            public enum Types
            {
                And,
                Or,
                OneOrMore,
                ZeroOrMore,
                Terminal,
            }

            public string Name { get; set; }

            public Types Type { get; set; }

            public List<Rule> Children { get; set; }

            public Rule(string name, Types type, params Rule[] children)
            {
                Name = name;
                Type = type;
                Children = children.ToList();
            }

            public override string ToString()
            {
                return "Parser.Rule( " + Type + ", " + Name + " )";
            }

            public Result Apply(List<Lexer.Result> tokens, ref int tokenPosition)
            {
                switch (Type)
                {
                    case Types.And:
                        {
                            List<Result> childResults = new List<Result>();
                            foreach (Rule child in Children)
                            {
                                Result childResult = child.Apply(tokens, ref tokenPosition);
                                if (childResult == null)
                                {
                                    return null;
                                }
                                else
                                {
                                    childResults.Add(childResult);
                                }
                            }
                            return new Result() { Rule = this, Children = childResults, Value = null };
                        }
                    case Types.Or:
                        {
                            foreach (Rule child in Children)
                            {
                                int pos = tokenPosition;
                                Result childResult = child.Apply(tokens, ref tokenPosition);
                                if (childResult == null)
                                {
                                    tokenPosition = pos;
                                }
                                else
                                {
                                    return new Result() { Rule = this, Children = new List<Result>() { childResult }, Value = null };
                                }
                            }
                            return null;
                        }
                    case Types.OneOrMore:
                        {
                            if (Children.Count != 1)
                            {
                                throw new Exception("OneOrMore rules must have one child");
                            }
                            Result childResult = Children[0].Apply(tokens, ref tokenPosition);
                            if (childResult == null)
                            {
                                return null;
                            }
                            else
                            {
                                List<Result> childResults = new List<Result>() { childResult };
                                while (true)
                                {
                                    int pos = tokenPosition;
                                    childResult = Children[0].Apply(tokens, ref tokenPosition);
                                    if (childResult == null)
                                    {
                                        tokenPosition = pos;
                                        break;
                                    }
                                    else
                                    {
                                        childResults.Add(childResult);
                                    }
                                }
                                return new Result() { Rule = this, Children = childResults, Value = null };
                            }
                        }
                    case Types.ZeroOrMore:
                        {
                            if (Children.Count != 1)
                            {
                                throw new Exception("ZeroOrMore rules must have one child");
                            }
                            List<Result> childResults = new List<Result>();
                            while (true)
                            {
                                int pos = tokenPosition;
                                Result childResult = Children[0].Apply(tokens, ref tokenPosition);
                                if (childResult == null)
                                {
                                    tokenPosition = pos;
                                    return new Result() { Rule = this, Children = childResults, Value = null };
                                }
                                else
                                {
                                    childResults.Add(childResult);
                                }
                            }
                        }
                    case Types.Terminal:
                        {
                            if (tokenPosition < tokens.Count && tokens[tokenPosition].Rule.Name == Name)
                            {
                                return new Result() { Rule = this, Children = new List<Result>(), Value = tokens[tokenPosition++].Value };
                            }
                            else
                            {
                                return null;
                            }
                        }
                    default:
                        throw new NotImplementedException();
                }
            }
        }

        public Parser(Lexer lexer, Rule root)
        {
            Lexer = lexer;
            Root = root;
        }

        public Result Parse(string input)
        {
            List<Lexer.Result> tokens = Lexer.Lex(input);
            int tokenPosition = 0;
            Result result = Root.Apply(tokens, ref tokenPosition);
            if (tokenPosition != tokens.Count)
            {
                throw new Exception("parse underflow");
            }
            else
            {
                return result;
            }
        }

        private class UnresolvedRule
        {
            public string Name { get; set; }

            public Rule.Types Type { get; set; }

            public List<string> Children { get; set; }
        }

        public static Parser Load(string grammar)
        {
            /*
             * equals = "=";
             * star = "\*";
             * bar = "\|";
             * plus = "\+";
             * gt = ">";
             * semicolon = ";";
             * str = "\"((\\.)|[^\\\\\"])*\"";
             * num = "\d+";
             * id = "[a-zA-Z][\da-zA-Z\-_]*";
             * ws *= "\s+";
             * 
             * lines => line+;
             * line => lexerLine | negLexerLine | ruleLine;
             * lexerLine => id equals str semicolon;
             * negLexerLine => id star equals str semicolon;
             * ruleLine => id equals gt rule semicolon;
             * rule => andRule | orRule | oneOrMoreRule | zeroOrMoreRule;
             * andRule => id andRuleTail;
             * andRuleTail => id+;
             * orRule => id orRuleTail;
             * orRuleTail => orRuleIter+;
             * orRuleIter => bar id;
             * oneOrMoreRule => id plus;
             * zeroOrMoreRule => id star;
             */
            Parser grammarParser = new Parser(
                new Lexer(
                    new Lexer.Rule("equals", @"=", true),
                    new Lexer.Rule("star", @"\*", true),
                    new Lexer.Rule("bar", @"\|", true),
                    new Lexer.Rule("plus", @"\+", true),
                    new Lexer.Rule("gt", @">", true),
                    new Lexer.Rule("semicolon", @";", true),
                    new Lexer.Rule("str", @""".*""", true),
                    new Lexer.Rule("num", @"\d+", true),
                    new Lexer.Rule("id", @"[a-zA-Z][\da-zA-Z\-_]*", true),
                    new Lexer.Rule("ws", @"\s+", false)
                ),
                new Rule("lines", Rule.Types.OneOrMore,
                    new Rule("line", Rule.Types.Or,
                        new Rule("lexerLine", Rule.Types.And,
                            new Rule("id", Rule.Types.Terminal),
                            new Rule("equals", Rule.Types.Terminal),
                            new Rule("str", Rule.Types.Terminal),
                            new Rule("semicolon", Rule.Types.Terminal)
                        ),
                        new Rule("negLexerLine", Rule.Types.And,
                            new Rule("id", Rule.Types.Terminal),
                            new Rule("star", Rule.Types.Terminal),
                            new Rule("equals", Rule.Types.Terminal),
                            new Rule("str", Rule.Types.Terminal),
                            new Rule("semicolon", Rule.Types.Terminal)
                        ),
                        new Rule("ruleLine", Rule.Types.And,
                            new Rule("id", Rule.Types.Terminal),
                            new Rule("equals", Rule.Types.Terminal),
                            new Rule("gt", Rule.Types.Terminal),
                            new Rule("rule", Rule.Types.Or,
                                new Rule("andRule", Rule.Types.And,
                                    new Rule("id", Rule.Types.Terminal),
                                    new Rule("andRuleTail", Rule.Types.OneOrMore,
                                        new Rule("id", Rule.Types.Terminal)
                                    )
                                ),
                                new Rule("orRule", Rule.Types.And,
                                    new Rule("id", Rule.Types.Terminal),
                                    new Rule("orRuleTail", Rule.Types.OneOrMore,
                                        new Rule("orRuleIter", Rule.Types.And,
                                            new Rule("bar", Rule.Types.Terminal),
                                            new Rule("id", Rule.Types.Terminal)
                                        )
                                    )
                                ),
                                new Rule("oneOrMoreRule", Rule.Types.And,
                                    new Rule("id", Rule.Types.Terminal),
                                    new Rule("plus", Rule.Types.Terminal)
                                ),
                                new Rule("zeroOrMoreRule", Rule.Types.And,
                                    new Rule("id", Rule.Types.Terminal),
                                    new Rule("star", Rule.Types.Terminal)
                                )
                            ),
                            new Rule("semicolon", Rule.Types.Terminal)
                        )
                    )
                )
            );
            /*
             * equals = "=";
             * star = "\*";
             * bar = "\|";
             * plus = "\+";
             * gt = ">";
             * semicolon = ";";
             * str = "\"((\\.)|[^\\\\\"])*\"";
             * num = "\d+";
             * id = "[a-zA-Z][\da-zA-Z\-_]*";
             * ws *= "\s+";
             * 
             * lines => line+;
             * line => lexerLine | negLexerLine | ruleLine;
             * lexerLine => id equals str semicolon;
             * negLexerLine => id star equals str semicolon;
             * ruleLine => id equals gt rule semicolon;
             * rule => andRule | orRule | oneOrMoreRule | zeroOrMoreRule;
             * andRule => id andRuleTail;
             * andRuleTail => id+;
             * orRule => id orRuleTail;
             * orRuleTail => orRuleIter+;
             * orRuleIter => bar id;
             * oneOrMoreRule => id plus;
             * zeroOrMoreRule => id star;
             */
            Result result = grammarParser.Parse(grammar);
            List<Lexer.Rule> lexerRules = new List<Lexer.Rule>();
            Dictionary<string, UnresolvedRule> unresolvedRules = new Dictionary<string, UnresolvedRule>();
            string rootName = null;
            foreach (Result lineParent in result.Children)
            {
                Result line = lineParent.Children.First();
                string lineType = line.Rule.Name;
                if (lineType == "lexerLine")
                {
                    string name = line.Children[0].Value;
                    string pattern = line.Children[2].Value;
                    lexerRules.Add(new Lexer.Rule(name, pattern.Substring(1, pattern.Length - 2), true));
                    unresolvedRules[name] = new UnresolvedRule()
                    {
                        Name = name,
                        Type = Rule.Types.Terminal,
                        Children = new List<string>(),
                    };
                }
                else if (lineType == "negLexerLine")
                {
                    string name = line.Children[0].Value;
                    string pattern = line.Children[3].Value;
                    lexerRules.Add(new Lexer.Rule(name, pattern.Substring(1, pattern.Length - 2), false));
                    unresolvedRules[name] = new UnresolvedRule()
                    {
                        Name = name,
                        Type = Rule.Types.Terminal,
                        Children = new List<string>(),
                    };
                }
                else if (lineType == "ruleLine")
                {
                    /*
                     * andRule => id andRuleTail;
                     * andRuleTail => id+;
                     * orRule => id orRuleTail;
                     * orRuleTail => orRuleIter+;
                     * orRuleIter => bar id;
                     * oneOrMoreRule => id plus;
                     * zeroOrMoreRule => id star;
                     */
                    string name = line.Children[0].Value;
                    if (rootName == null)
                    {
                        rootName = name;
                    }
                    Result rule = line.Children[3].Children[0];
                    string ruleType = rule.Rule.Name;
                    if (ruleType == "andRule")
                    {
                        List<string> children = new List<string>() { rule.Children[0].Value };
                        children.AddRange(rule.Children[1].Children.Select(child => child.Value));
                        unresolvedRules[name] = new UnresolvedRule()
                        {
                            Name = name,
                            Type = Rule.Types.And,
                            Children = children,
                        };
                    }
                    else if (ruleType == "orRule")
                    {
                        List<string> children = new List<string>() { rule.Children[0].Value };
                        children.AddRange(rule.Children[1].Children
                            .Select(child => child.Children[1].Value));
                        unresolvedRules[name] = new UnresolvedRule()
                        {
                            Name = name,
                            Type = Rule.Types.Or,
                            Children = children,
                        };
                    }
                    else if (ruleType == "oneOrMoreRule")
                    {
                        unresolvedRules[name] = new UnresolvedRule()
                        {
                            Name = name,
                            Type = Rule.Types.OneOrMore,
                            Children = new List<string>() { rule.Children[0].Value },
                        };
                    }
                    else if (ruleType == "zeroOrMoreRule")
                    {
                        unresolvedRules[name] = new UnresolvedRule()
                        {
                            Name = name,
                            Type = Rule.Types.ZeroOrMore,
                            Children = new List<string>() { rule.Children[0].Value },
                        };
                    }
                }
            }

            if (rootName == null)
            {
                throw new Exception("no root rule found");
            }

            Dictionary<string,Rule> resolvedRules = new Dictionary<string,Rule>();
            Rule root = Resolve(resolvedRules, unresolvedRules, rootName);

            return new Parser(new Lexer(lexerRules.ToArray()), root);
        }

        private static Rule Resolve(Dictionary<string, Rule> resolvedRules, Dictionary<string, UnresolvedRule> unresolvedRules, string name)
        {
            if (resolvedRules.ContainsKey(name))
            {
                return resolvedRules[name];
            }
            else if (unresolvedRules.ContainsKey(name))
            {
                return resolvedRules[name] = new Rule(
                    name, 
                    unresolvedRules[name].Type, 
                    unresolvedRules[name].Children
                    .Select(child => Resolve(resolvedRules, unresolvedRules, child))
                    .ToArray());
            }
            else
            {
                throw new Exception("unknown rule " + name);
            }
        }

        public Lexer Lexer { get; set; }

        public Rule Root { get; set; }
    }
}
