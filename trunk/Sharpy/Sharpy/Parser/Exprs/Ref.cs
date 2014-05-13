using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Parser.Exprs
{
    public class Ref : Expr
    {
        public string Name { get; set; }

        public string Value { get; private set; }

        public Ref(string value)
        {
            Name = value;
            Value = value;
        }

        public Rule Resolve(Dictionary<string, Rule> rules, Dictionary<string, Expr> exprs, Dictionary<string, Lexer.Rule> lexerRules)
        {
            Rule rule;
            Expr expr;
            Lexer.Rule lexerRule;
            if (rules.TryGetValue(Value, out rule))
            {
                return rule;
            }
            else if (exprs.TryGetValue(Value, out expr))
            {
                return rules[Value] = expr.Resolve(rules, exprs, lexerRules);
            }
            else if (lexerRules.TryGetValue(Value, out lexerRule))
            {
                return rules[Value] = new Rule(Rule.Types.Terminal, Value);
            }
            else
            {
                throw new Exception("unknown ref " + Value);
            }
        }

        public override string ToString()
        {
            return Value;
        }
    }
}
