using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.ComponentModel;

namespace libterp
{
    public class Lexer
    {
        public class Rule
        {
            public Regex Regex { get; private set; }

            public string Name { get; private set; }

            public bool Delimiter { get; private set; }

            public Rule(string name, string regex)
                : this(name, regex, false)
            {
            }

            public Rule(string name, string regex, bool delimiter)
            {
                Name = name;
                Regex = new Regex(@"^" + regex);
                Delimiter = delimiter;
            }
        };

        public class Result
        {
            public string Type { get; private set; }

            public string Value { get; private set; }

            public Result(string type, string value)
            {
                Type = type;
                Value = value;
            }

            public override string ToString()
            {
                return string.Format("{0}({1})", Type, Value);
            }
        };

        public List<Rule> Rules { get; private set; }

        public Lexer(params Rule[] rules)
        {
            Rules = new List<Rule>(rules);
        }

        private class Context
        {
            public List<Result> Results { get; set; }

            public string Str { get; set; }

            public int Position { get; set; }

            public Context()
            {
                Results = new List<Result>();
                Position = 0;
            }

            public List<Context> Expand(List<Rule> rules)
            {
                List<Context> contexts = new List<Context>();
                foreach (Rule rule in rules)
                {
                    Match match = rule.Regex.Match(Str.Substring(Position));
                    if (match.Success && match.Length > 0)
                    {
                        Context context = new Context()
                        {
                            Str = Str,
                            Position = Position + match.Length,
                            Results = new List<Result>(Results)
                        };
                        if (!rule.Delimiter)
                        {
                            context.Results.Add(new Result(rule.Name, Str.Substring(Position, match.Length)));
                        }
                        contexts.Add(context);
                    }
                }
                return contexts;
            }
        };

        public List<Result> Lex(string str)
        {
            Queue<Context> contexts = new Queue<Context>();
            contexts.Enqueue(new Context() { Str = str });
            while (contexts.Any())
            {
                Context context = contexts.Dequeue();
                if (context.Position == str.Length)
                {
                    return context.Results;
                }
                else
                {
                    context.Expand(Rules).ForEach(c => contexts.Enqueue(c));
                }
            }
            return null;
        }
    }
}
/**
 * regex = ""\S+""
 * newline = "\n"
 * equals = "="
 * pipe = "|"
 * plus = "+"
 * lparen = "("
 * rparen = ")"
 * star = "*"
 * id = "[a-zA-Z]\w+"
 * ws *= "\s+"
 * 
 * grammar = line*
 * line = decl newline | newline
 * decl = rule | token | delimToken
 * token = id equals regex
 * delimToken = id star equals regex
 * rule = id equals ruleDecl
 * ruleDecl = id
 *             | ruleDeclStar
 *             | ruleDeclPlus
 *             | ruleDeclAnd
 *             | ruleDeclOr
 *             | ruleDeclParen
 * ruleDeclOr = ruleDecl ruleDeclOr_repeat
 * ruleDeclOr_repeat = ruleDecl_repeatContent+
 * ruleDeclOr_repeatContent = pipe ruleDecl
 * ruleDeclAnd = ruleDecl ruleDeclAnd_repeat
 * ruleDeclAnd_repeat = ruleDecl+
 * ruleDeclParen = lparen ruleDecl rparen
 * ruleDeclPlus = ruleDecl plus
 * ruleDeclStar = ruleDecl star
 **/
