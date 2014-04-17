using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Parser.RuleExprs
{
    public class ParserRule : RuleExpr
    {
        public string Name { get; set; }
        
        public Rule.Types Type { get; private set; }

        public List<RuleExpr> Children { get; private set; }

        public ParserRule(Rule.Types type, params RuleExpr[] children)
        {
            Type = type;
            Children = children.ToList();
        }

        public Rule Bind(Dictionary<string, Rule> boundRules, Dictionary<string, RuleExpr> unboundRules, Lexer.Lexer lexer)
        {
            Rule rule = new Rule(Name, Type);
            if ( Name != null)
            {
                boundRules[Name] = rule;
            }
            foreach (RuleExpr child in Children)
            {
                rule.Children.Add(child.Bind(boundRules, unboundRules, lexer));
            }
            return rule;
        }

        public override string ToString()
        {
            return string.Format("<Parser.ParserRule Name=\"{0}\" Type=\"{1}\"/>", Name, Type);
        }
    }
}
