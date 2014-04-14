using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Parser
{
    public class RuleRef : RuleExpr
    {
        public string Name { get; set; }

        public string Value { get; set; }

        public RuleRef(string value)
        {
            Value = value;
        }

        public Rule Bind(Dictionary<string, Rule> boundRules, Dictionary<string, RuleExpr> unboundRules, Lexer.Lexer lexer)
        {
            Rule boundRule;
            RuleExpr unboundRule;
            if (boundRules.TryGetValue(Value, out boundRule))
            {
                return boundRule;
            }
            else if (unboundRules.TryGetValue(Value, out unboundRule))
            {
                return boundRules[Value] = unboundRule.Bind(boundRules, unboundRules, lexer);
            }
            else if (lexer.Rules.Any(rule => rule.Name == Value))
            {
                return boundRules[Value] = new Rule(Value, Rule.Types.Terminal);
            }
            else
            {
                throw new Exception("unknown rule " + Value);
            }
        }

        public override string ToString()
        {
            return string.Format("<Parser.RuleRef Name=\"{0}\" Value=\"{1}\"/>", Name, Value);
        }
    }
}
