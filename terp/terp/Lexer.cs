using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;

namespace terp
{
    public class Lexer
    {
        public class Result
        {
            public String Type { get; private set; }

            public String Value { get; private set; }

            public Result(string type, string value)
            {
                Type = type;
                Value = value;
            }
        };

        public class Rule
        {
            public string Name { get; private set; }

            private Regex Regex { get; set; }

            public bool Delimiter { get; private set; }

            public Rule(string name, string regex, bool delimiter)
            {
                Name = name;
                Regex = new Regex(@"^" + regex);
                Delimiter = delimiter;
            }

            public int Lex(string input, int index)
            {
                Match match = Regex.Match(input.Substring(index));
                if (match.Success && match.Index == 0 && match.Length > 0)
                {
                    return match.Length;
                }
                else
                {
                    return 0;
                }
            }
        };

        public List<Rule> Rules { get; private set; }

        public bool HasRule(string name)
        {
            return Rules.Any(rule => rule.Name == name);
        }

        public Lexer(params Rule[] rules)
        {
            Rules = new List<Rule>(rules);
        }

        public List<Result> Lex(string input)
        {
            List<Result> results = new List<Result>();
            int position = 0;
            while (position < input.Length)
            {
                int len = 0;
                string matchName = null;
                foreach (Rule rule in Rules)
                {
                    int ruleLen = rule.Lex(input, position);
                    if (ruleLen > len)
                    {
                        len = ruleLen;
                        if (!rule.Delimiter)
                        {
                            matchName = rule.Name;
                        }
                    }
                }
                if (len == 0)
                {
                    throw new Exception("lex error at " + position + " " + input.Substring(position));
                }
                if (matchName != null)
                {
                    results.Add(new Result(matchName, input.Substring(position, len)));
                }
                position += len;
            }
            return results;
        }
    };
}
