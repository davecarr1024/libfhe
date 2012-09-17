using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.ComponentModel;

namespace libterp
{
  public class Lexer
  {
    public class Rule
    {
      public Regex Regex { get; private set; }

      public string Name { get; private set; }

      public bool Delimiter { get; private set; }

      public Rule(string name, string regex)
        : this(name, regex, false)
      {
      }

      public Rule(string name, string regex, bool delimiter)
      {
        Name = name;
        Regex = new Regex(@"^" + regex);
        Delimiter = delimiter;
      }
    };

    public class Token
    {
      public string Type { get; private set; }

      public string Value { get; private set; }

      public Token(string type, string value)
      {
        Type = type;
        Value = value;
      }

      public override string ToString()
      {
        return string.Format("{0}({1})", Type, Value);
      }
    };

    public List<Rule> Rules { get; private set; }

    public Lexer(params Rule[] rules)
    {
      Rules = new List<Rule>(rules);
    }

    public List<Token> Lex(string s)
    {
      List<Token> tokens = new List<Token>();
      for (int i = 0; i < s.Length; )
      {
        bool matched = false;
        foreach (Rule rule in Rules)
        {
          if (!matched)
          {
            Match match = rule.Regex.Match(s.Substring(i));
            if (match.Success)
            {
              if (!rule.Delimiter)
              {
                tokens.Add(new Token(rule.Name, s.Substring(i, match.Length)));
              }
              i += match.Length;
              matched = true;
            }
          }
        }
        if (!matched)
        {
          throw new Exception("lex error at position " + i + " " + s.Substring(i));
        }
      }
      return tokens;
    }
  }
}
/**
 * enddecl = ;;
 * tokenop = =;
 * ruleop = =>;
 * orop = |;
 * andop = +;
 * lparen = (;
 * rparen = );
 * oneormoreop = *;
 * delimop = (delim);
 * id = \w+;
 * ws = \W+ (delim);
 * 
 * grammar => decl*
 * decl => ( rule | token ) enddecl;
 * decl2 => rule | token
 * token => id tokenop ( id | id delimop );
 * rule => id rulop ruledecl;
 * ruledecl => id | 
 *             ruledecl orop ruledecl | 
 *             ruledecl andop ruledecl | 
 *             lparen ruledecl rparen | 
 *             ruledecl oneormoreop;
 *
 * grammar => decl*
 * decl => decl2 enddecl;
 * decl2 => rule | token;
 * token => id tokenop token2;
 * token2 => ( id | delimtoken );
 * delimtoken => id delimop;
 * rule => id rulop ruledecl;
 * ruledecl => id | ruledeclor | ruledecland | ruledeclparen | ruledecloneormore;
 * ruledeclor => ruledecl orop ruledecl;
 * ruledecland => ruledecl andop ruledecl;
 * ruledeclparen => lparen ruledecl rparen;
 * ruledecloneormore => ruledecl oneormoreop;
 **/
