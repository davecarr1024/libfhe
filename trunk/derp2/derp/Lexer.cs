using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Text.RegularExpressions;

namespace derp
{
    public class Lexer
    {
        public class Result
        {
            public Rule Rule { get; set; }

            public string Value { get; set; }

            public override string ToString()
            {
                return "Lexer.Result( " + Rule.Name + ", " + Value + " )";
            }
        }

        public class Rule
        {
            public string Name { get; set; }

            public string Pattern { get; set; }

            public bool Include { get; set; }

            private Regex regex = null;

            public Rule(string name, string pattern, bool include)
            {
                Name = name;
                Pattern = pattern;
                Include = include;
            }

            public Result Apply(string input, int pos)
            {
                if (regex == null)
                {
                    regex = new Regex(Pattern);
                }
                Match match = regex.Match(input, pos);
                if (match.Success && match.Index == pos && match.Length > 0)
                {
                    return new Result() { Rule = this, Value = input.Substring(pos, match.Length) };
                }
                else
                {
                    return null;
                }
            }

            public override string ToString()
            {
                return "Lexer.Rule( " + Name + ", " + Pattern + ", " + Include + " )";
            }
        }

        private List<Rule> rules = new List<Rule>();

        public List<Rule> Rules
        {
            get { return rules; }
            set { rules = value; }
        }

        public Lexer(params Rule[] rules)
        {
            Rules = rules.ToList();
        }

        public List<Result> Lex(string input)
        {
            List<Result> results = new List<Result>();
            for (int pos = 0; pos < input.Length; )
            {
                Result result = rules
                    .Select(rule => rule.Apply(input, pos))
                    .Where( r => r != null )
                    .FirstOrDefault();
                if (result == null)
                {
                    throw new Exception("lex error " + input.Substring(pos));
                }
                else
                {
                    if (result.Rule.Include)
                    {
                        results.Add(result);
                    }
                    pos += result.Value.Length;
                }
            }
            return results;
        }
    }
}
