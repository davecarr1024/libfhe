using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;

namespace Derp
{
    public class Lexer
    {
        public class Rule
        {
            public string Name { get; set; }

            public string Pattern { get; set; }

            public bool Include { get; set; }

            public Rule(string name, string pattern, bool include)
            {
                Name = name;
                Pattern = pattern;
                Include = include;
            }

            public string Apply(string input)
            {
                Match match = Regex.Match(input, Pattern);
                if (match.Success && match.Index == 0 && match.Length > 0)
                {
                    return input.Substring(0, match.Length);
                }
                else
                {
                    return null;
                }
            }

            public override string ToString()
            {
                return string.Format("<Lexer.Rule Name=\"{0}\" Pattern=\"{1}\" Include=\"{2}\">", Name, Pattern, Include);
            }
        }

        public class Result
        {
            public Rule Rule { get; set; }

            public string Value { get; set; }

            public Result(Rule rule, string value)
            {
                Rule = rule;
                Value = value;
            }

            public override string ToString()
            {
                return string.Format("<Lexer.Result Rule=\"{0}\" Value=\"{1}\">", Rule.Name, Value);
            }
        }

        public List<Rule> Rules { get; set; }

        public Lexer(params Rule[] rules)
        {
            Rules = rules.ToList();
        }

        public List<Result> Lex(string input)
        {
            List<Result> results = new List<Result>();
            for (int pos = 0; pos < input.Length; )
            {
                Result result = Rules
                    .Select(rule => new Result(rule, rule.Apply(input.Substring(pos))))
                    .FirstOrDefault(r => !string.IsNullOrEmpty(r.Value));
                if (result != null)
                {
                    if (result.Rule.Include)
                    {
                        results.Add(result);
                    }
                    pos += result.Value.Length;
                }
                else
                {
                    throw new Exception("lex error at " + pos + " " + input.Substring(pos, Math.Min(25, input.Length - pos)));
                }
            }
            return results;
        }
    }
}
