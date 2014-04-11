using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Lexer
{
    public class Lexer
    {
        public List<Rule> Rules { get; private set; }

        public Lexer(params Rule[] rules)
        {
            Rules = rules.ToList();
        }

        public List<Result> Lex(string input)
        {
            int pos = 0;
            List<Result> results = new List<Result>();
            while (pos < input.Length)
            {
                Result result = Rules.Select(rule => rule.Apply(input.Substring(pos))).FirstOrDefault(r => r != null);
                if (result == null)
                {
                    throw new Exception("lex error at " + string.Concat(input.Skip(pos).Take(50)));
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
