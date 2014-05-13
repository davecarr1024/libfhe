using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Parser.Exprs
{
    public interface Expr
    {
        string Name { get; set; }

        Rule Resolve(Dictionary<string, Rule> rules, Dictionary<string, Expr> exprs, Dictionary<string, Lexer.Rule> lexerRules);
    }
}
