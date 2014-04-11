using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Text.RegularExpressions;

namespace Sherp.Lexer
{
    public class Rule
    {
        public string Name { get; private set; }

        public string Pattern { get; private set; }

        public bool Include { get; private set; }

        public Rule(string name, string pattern, bool include)
        {
            Name = name;
            Pattern = pattern;
            Include = include;
        }

        public Result Apply(string input)
        {
            Match match = Regex.Match(input, Pattern);
            if (match.Success && match.Index == 0 && match.Length > 0)
            {
                return new Result(this, input.Substring(0, match.Length));
            }
            else
            {
                return null;
            }
        }
    }
}
