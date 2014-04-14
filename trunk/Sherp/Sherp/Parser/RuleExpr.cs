using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Parser
{
    public interface RuleExpr
    {
        string Name { get; set; }

        Rule Bind(Dictionary<string, Rule> boundRules, Dictionary<string, RuleExpr> unboundRules, Lexer.Lexer lexer);
    }
}
