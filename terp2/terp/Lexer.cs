using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;

namespace terp
{
  public class Lexer
  {
    public class Rule
    {
      public string Name { get; set; }

      public Regex Regex { get; set; }

      public bool Delimiter { get; set; }

      public int Match(string input, int position)
      {
        Match match = Regex.Match(input.Substring(position));
        if (match.Success && match.Index == 0 && match.Length > 0)
        {
          return match.Length;
        }
        else
        {
          return 0;
        }
      }
    }

    public class Result
    {
      public string Type { get; set; }

      public string Value { get; set; }
    }

    public List<Rule> Rules { get; set; }

    public List<Result> Lex(string input)
    {
      List<Result> results = new List<Result>();
      int position = 0;
      while (position < input.Length)
      {
        bool found = false;
        foreach (Rule rule in Rules)
        {
          if (!found)
          {
            int length = rule.Match(input, position);
            if (length > 0)
            {
              found = true;
              if (!rule.Delimiter)
              {
                results.Add(new Result() { Type = rule.Name, Value = input.Substring(position, length) });
              }
              position += length;
            }
          }
        }
        if (!found)
        {
          throw new Exception("lex error at " + position + ": " + input.Substring(position));
        }
      }
      return results;
    }

    public Lexer()
    {
      Rules = new List<Rule>();
    }
  }
}
