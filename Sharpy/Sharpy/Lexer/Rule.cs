using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Text.RegularExpressions;

namespace Sharpy.Lexer
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

        public string Apply(string input)
        {
            Match match = Regex.Match(input, Pattern);
            if (match.Success && match.Index == 0 && match.Length > 0)
            {
                return match.Value;
            }
            else
            {
                return null;
            }
        }

        public override string ToString()
        {
            return string.Format("Lexer.Rule({0},{1},{2})", Name, Pattern, Include);
        }
    }
}
