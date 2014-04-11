using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Lexer
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
            return string.Format("<Lexer.Result Rule=\"{0}\" Value=\"{1}\"/>", Rule.Name, Value);
        }
    }
}
