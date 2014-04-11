using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Parser
{
    public class RuleDecl
    {
        public string Name { get; private set; }

        public Rule.Types Type { get; private set; }

        public List<string> Children { get; private set; }

        public RuleDecl(string name, Rule.Types type, params string[] children)
        {
            Name = name;
            Type = type;
            Children = children.ToList();
        }

        public RuleDecl(Result result)
        {
            Init(result);
        }

        private void Init(Result result)
        {
            switch (result.Rule.Name)
            {
                case "parserDecl":
                    Name = result.Children[0].Value;
                    Init(result.Children[3]);
                    break;
                case "parserRule":
                    Init(result.Children[0]);
                    break;
                case "andRule":
                    Type = Rule.Types.And;
                    Children = new List<string>() { result.Children[0].Value };
                    Init(result.Children[1]);
                    break;
                case "andRuleTail":
                    Children.AddRange(result.Children.Select(child => child.Value));
                    break;
                case "orRule":
                    Type = Rule.Types.Or;
                    Children = new List<string>() { result.Children[0].Value };
                    Init(result.Children[1]);
                    break;
                case "orRuleTail":
                    foreach (Result child in result.Children)
                    {
                        Init(child);
                    }
                    break;
                case "orRuleIter":
                    Children.Add(result.Children[1].Value);
                    break;
                case "oneOrMoreRule":
                    Type = Rule.Types.OneOrMore;
                    Children = new List<string>() { result.Children[0].Value };
                    break;
                case "zeroOrMoreRule":
                    Type = Rule.Types.ZeroOrMore;
                    Children = new List<string>() { result.Children[0].Value };
                    break;
                case "zeroOrOneRule":
                    Type = Rule.Types.ZeroOrOne;
                    Children = new List<string>() { result.Children[0].Value };
                    break;
                default:
                    throw new Exception("invalid ruledecl " + result.Rule.Name);
            }
        }

        public static Rule Bind(Lexer.Lexer lexer, RuleDecl root, params RuleDecl[] unboundRuleList)
        {
            Dictionary<string, RuleDecl> unboundRules = new Dictionary<string, RuleDecl>() { { root.Name, root } };
            foreach (RuleDecl unboundRule in unboundRuleList)
            {
                unboundRules[unboundRule.Name] = unboundRule;
            }
            return Bind(root.Name, lexer, unboundRules);
        }

        public static Rule Bind(string rootName, Lexer.Lexer lexer, Dictionary<string, RuleDecl> unboundRules)
        {
            Dictionary<string, Rule> boundRules = new Dictionary<string, Rule>();
            Dictionary<string, Lexer.Rule> lexerRules = new Dictionary<string, Lexer.Rule>();
            foreach (Lexer.Rule lexerRule in lexer.Rules)
            {
                lexerRules[lexerRule.Name] = lexerRule;
            }
            return Bind(rootName, boundRules, unboundRules, lexerRules);
        }

        private static Rule Bind(string name, Dictionary<string, Rule> boundRules, Dictionary<string, RuleDecl> unboundRules, Dictionary<string, Lexer.Rule> lexerRules)
        {
            Rule boundRule;
            RuleDecl unboundRule;
            Lexer.Rule lexerRule;
            if (boundRules.TryGetValue(name, out boundRule))
            {
                return boundRule;
            }
            else if (unboundRules.TryGetValue(name, out unboundRule))
            {
                boundRule = boundRules[name] = new Rule(name, unboundRule.Type);
                foreach (string childName in unboundRule.Children)
                {
                    boundRule.Children.Add(Bind(childName, boundRules, unboundRules, lexerRules));
                }
                return boundRule;
            }
            else if (lexerRules.TryGetValue(name, out lexerRule))
            {
                return new Rule(name, Rule.Types.Terminal);
            }
            else
            {
                throw new Exception("unknown rule " + name);
            }
        }
    }
}
