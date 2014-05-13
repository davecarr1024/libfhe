using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Lexer
{
    public class Result
    {
        public Rule Rule { get; private set; }

        public string Value { get; private set; }

        public Result(Rule rule, string value)
        {
            Rule = rule;
            Value = value;
        }

        public override string ToString()
        {
            return string.Format("Lexer.Result({0},{1})", Rule.Name, Value);
        }
    }
}
