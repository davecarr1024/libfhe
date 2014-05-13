using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Parser.Exprs
{
    public class Token : Expr
    {
        public string Name { get; set; }

        public string Value { get; private set; }

        public Token(string value)
        {
            Name = Value;
            Value = value;
        }

        public Rule Resolve(Dictionary<string, Rule> rules, Dictionary<string, Expr> exprs, Dictionary<string, Lexer.Rule> lexerRules)
        {
            Rule rule;
            Lexer.Rule lexerRule;
            if (rules.TryGetValue(Value, out rule))
            {
                return rule;
            }
            else if (lexerRules.TryGetValue(Value, out lexerRule))
            {
                return rules[Value] = new Rule(Rule.Types.Terminal, Value);
            }
            else
            {
                lexerRules[Value] = new Lexer.Rule(Value, Value, true);
                return rules[Value] = new Rule(Rule.Types.Terminal, Value);
            }
        }

        public override string ToString()
        {
            return Value;
        }
    }
}
