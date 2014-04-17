using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Parser.RuleExprs
{
    public class LexerRule : RuleExpr
    {
        public string Name { get; set; }

        public LexerRule(string name)
        {
            Name = name;
        }

        public Rule Bind(Dictionary<string, Rule> boundRules, Dictionary<string, RuleExpr> unboundRules, Lexer.Lexer lexer)
        {
            Rule rule;
            if (boundRules.TryGetValue(Name, out rule))
            {
                return rule;
            }
            else
            {
                if (!lexer.Rules.Any(r => r.Name == Name))
                {
                    lexer.Rules.Insert(0, new Lexer.Rule(Name, Name, true));
                }
                return boundRules[Name] = new Rule(Name, Rule.Types.Terminal);
            }
        }
    }
}
