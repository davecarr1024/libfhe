using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Lexer
{
    public class Lexer
    {
        public List<Rule> Rules { get; private set; }

        public Lexer(params Rule[] rules)
        {
            Rules = rules.ToList();
        }

        public List<Result> Apply(string input)
        {
            List<Result> results = new List<Result>();
            int pos = 0;
            while (pos < input.Length)
            {
                Result result = Rules
                    .Select(rule => new Result(rule, rule.Apply(input.Substring(pos))))
                    .Where(r => !string.IsNullOrEmpty(r.Value))
                    .OrderByDescending(r => r.Value.Length)
                    .FirstOrDefault();
                if (result == null)
                {
                    throw new Exception("lex error at " + new string(input.Skip(pos).Take(100).ToArray()));
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
