using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Parser.Exprs
{
    public class Def : Expr
    {
        public Rule.Types Type { get; private set; }

        public string Name { get; set; }

        public List<Expr> Children { get; private set; }

        public Def(Rule.Types type, string name, params Expr[] children)
        {
            Type = type;
            Name = name;
            Children = children.ToList();
        }

        public Rule Resolve(Dictionary<string, Rule> rules, Dictionary<string, Expr> exprs, Dictionary<string, Lexer.Rule> lexerRules)
        {
            Rule rule;
            if (!string.IsNullOrEmpty(Name) && rules.TryGetValue(Name, out rule))
            {
                return rule;
            }
            else
            {
                rule = new Rule(Type, Name);
                if (!string.IsNullOrEmpty(Name))
                {
                    rules[Name] = rule;
                }
                foreach (Expr child in Children)
                {
                    rule.Children.Add(child.Resolve(rules, exprs, lexerRules));
                }
                return rule;
            }
        }

        public override string ToString()
        {
            return string.Format("Parser.Exprs.Def({0},{1})", Type, Name);
        }
    }
}
