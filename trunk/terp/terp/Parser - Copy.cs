using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace terp
{
  public class Parser
  {
    public class Result
    {
      public string Type { get; private set; }

      public string Value { get; set; }

      public List<Result> Children { get; private set; }

      public Result(string type, params Result[] children)
      {
        Type = type;
        Children = new List<Result>(children);
      }
    };

    public class Rule
    {
      public enum Type
      {
        And,
        Or,
        Terminal,
        OneOrMore,
        ZeroOrMore,
      };

      public Parser Parser { get; private set; }

      public Type type { get; private set; }

      public string Name { get; private set; }

      public List<Rule> Children { get; private set; }

      public List<string> ChildNames { get; private set; }

      public Rule(Parser parser, Type type, string name, params string[] childNames)
      {
        Parser = parser;
        this.type = type;
        Name = name;
        Children = new List<Rule>();
        ChildNames = new List<string>(childNames);
      }

      private void AddChild(Rule child)
      {
        if (!Children.Contains(child))
        {
          Children.Add(child);
        }
      }

      public void Connect()
      {
        foreach (string childName in ChildNames)
        {
          if (Parser.Rules.ContainsKey(childName))
          {
            AddChild(Parser.Rules[childName]);
          }
          else
          {
            throw new Exception("Rule " + Name + " failed to find child " + childName);
          }
        }
      }

      public Result Parse(Context context)
      {
        switch (type)
        {
          case Type.And:
            return ParseAnd(context);
          case Type.Or:
            return ParseOr(context);
          case Type.Terminal:
            return ParseTerminal(context);
          case Type.OneOrMore:
            return ParseOneOrMore(context);
          case Type.ZeroOrMore:
            return ParseZeroOrMore(context);
          default:
            throw new Exception("Rule " + Name + " has unknown type " + type);
        }
      }

      private string Tab(string input)
      {
        return string.Join("", input.Split('\n').Select(line => "  " + line).ToArray());
      }

      private Result ParseZeroOrMore(Context context)
      {
        if (Children.Count != 1)
        {
          throw new Exception("ZeroOrMore Rule " + Name + " failed: must have one child");
        }
        else
        {
          Context childContext = new Context(context);
          Result result = new Result(Name);
          Result childResult;
          do
          {
            try
            {
              childResult = Children[0].Parse(childContext);
              result.Children.Add(childResult);
              context.Position = childContext.Position;
            }
            catch (Exception)
            {
              childResult = null;
            }
          }
          while (childResult != null);
          return result;
        }
      }

      private Result ParseOneOrMore(Context context)
      {
        if (Children.Count != 1)
        {
          throw new Exception("OneOrMore Rule " + Name + " failed: must have one child");
        }
        else
        {
          Context childContext = new Context(context);
          Result result = new Result(Name);
          Result childResult;
          try
          {
            childResult = Children[0].Parse(childContext);
          }
          catch (Exception ex)
          {
            throw new Exception("OneOrMore Rule " + Name + " failed: failed to parse first child:\n" + Tab(ex.Message));
          }
          while (childResult != null)
          {
            result.Children.Add(childResult);
            context.Position = childContext.Position;
            try
            {
              childResult = Children[0].Parse(childContext);
            }
            catch (Exception)
            {
              childResult = null;
            }
          }
          return result;
        }
      }

      private Result ParseTerminal(Context context)
      {
        if (context.Position >= context.Input.Count)
        {
          throw new Exception("Terminal Rule " + Name + " failed: out of input");
        }
        else if (context.Input[context.Position].Type != Name)
        {
          throw new Exception("Terminal Rule " + Name + " token type mismatch: " + context.Input[context.Position].Type);
        }
        else
        {
          return new Result(Name) { Value = context.Input[context.Position++].Value };
        }
      }

      private Result ParseOr(Context context)
      {
        List<Exception> childExceptions = new List<Exception>();
        foreach (Rule rule in Children)
        {
          try
          {
            Context childContext = new Context(context);
            Result result = new Result(Name, rule.Parse(childContext));
            context.Position = childContext.Position;
            return result;
          }
          catch (Exception ex)
          {
            childExceptions.Add(ex);
          }
        }
        throw new Exception("Or Rule " + Name + " failed: to parse any children:\n"
          + Tab(string.Join("\n", childExceptions.Select(ex => ex.Message).ToArray())));
      }

      private Result ParseAnd(Context context)
      {
        Result result = new Result(Name);
        foreach (Rule childRule in Children)
        {
          try
          {
            result.Children.Add(childRule.Parse(context));
          }
          catch (Exception ex)
          {
            throw new Exception("And Rule " + Name + " failed: failed to parse child " + childRule.Name + ":\n" + Tab(ex.Message));
          }
        }
        return result;
      }
    };

    public class Context
    {
      public Queue<Lexer.Result> Input { get; set; }

      public Context(List<Lexer.Result> input)
      {
        Input = new Queue<Lexer.Result>(input);
      }

      public Context(Context context)
      {
        Input = new Queue<Lexer.Result>(context.Input);
      }

      public List<Context> Expand()
      {
        List<Context> newContexts = new List<Context>();
        return newContexts;
      }
    };

    public Lexer Lexer { get; set; }

    public Dictionary<string, Rule> Rules { get; private set; }

    private Rule RootRule { get; set; }

    public Rule AddRule(Rule.Type type, string name, params string[] childNames)
    {
      Rule rule = new Rule(this, type, name, childNames);
      if (!Rules.Any())
      {
        RootRule = rule;
      }
      Rules[name] = rule;
      return rule;
    }

    private void ConnectRules()
    {
      List<string> childNames = new List<string>();
      foreach (Rule rule in Rules.Values)
      {
        childNames.AddRange(rule.ChildNames);
      }

      foreach (string childName in childNames)
      {
        if (!Rules.ContainsKey(childName) && Lexer.HasRule(childName))
        {
          AddRule(Rule.Type.Terminal, childName);
        }
      }

      foreach (Rule rule in Rules.Values)
      {
        rule.Connect();
      }
    }

    public Parser(Lexer lexer)
    {
      Lexer = lexer;
      Rules = new Dictionary<string, Rule>();
    }

    public Parser(string grammar)
    {
      /**
       * lparen = "\("
       * rparen = "\)"
       * regex = "\".+\""
       * equals = "="
       * pipe = "|"
       * plus = "+"
       * star = "*"
       * id = "[a-zA-Z]\w*"
       * whitespace *= "\s+"
       * 
       * grammar = decl*
       * decl = token | delimToken | rule
       * token = id equals regex
       * delimToken = id star equals regex
       * rule = id equals ruleDecl
       * ruleDecl = id | ruleDeclOr | ruleDeclAnd | ruleDeclParen | ruleDeclStar | ruleDeclPlus
       * ruleDeclOr = ruleDecl pipe ruleDecl
       * ruleDeclAnd = ruleDecl ruleDecl
       * ruleDeclParen = lparen ruleDecl rparen
       * ruleDeclStar = ruleDecl star
       * ruleDeclPlus = ruleDecl plus
       */

      Lexer lexer = new Lexer(
        new Lexer.Rule("lparen", @"\(", false),
        new Lexer.Rule("rparen", @"\)", false),
        new Lexer.Rule("regex", "\".+\"", false),
        new Lexer.Rule("equals", @"\=", false),
        new Lexer.Rule("pipe", @"\|", false),
        new Lexer.Rule("plus", @"\+", false),
        new Lexer.Rule("star", @"\*", false),
        new Lexer.Rule("id", @"[a-zA-Z]\w*", false),
        new Lexer.Rule("whitespace", @"\s+", true)
        );

      Parser parser = new Parser(lexer);
      parser.AddRule(Rule.Type.ZeroOrMore, "grammar", "decl");
      parser.AddRule(Rule.Type.Or, "decl", "token", "delimToken", "rule");
      parser.AddRule(Rule.Type.And, "token", "id", "equals", "regex");
      parser.AddRule(Rule.Type.And, "delimToken", "id", "star", "equals", "regex");
      parser.AddRule(Rule.Type.And, "rule", "id", "equals", "ruleDecl");
      parser.AddRule(Rule.Type.Or, "ruleDecl", "id", "ruleDeclOr", "ruleDeclAnd", "ruleDeclParen", "ruleDeclStar", "ruleDeclPlus");
      parser.AddRule(Rule.Type.And, "ruleDeclOr", "ruleDecl", "pipe", "ruleDecl");
      parser.AddRule(Rule.Type.And, "ruleDeclAnd", "ruleDecl", "ruleDecl");
      parser.AddRule(Rule.Type.And, "ruleDeclParen", "lparen", "ruleDecl", "rparen");
      parser.AddRule(Rule.Type.And, "ruleDeclStar", "ruleDecl", "star");
      parser.AddRule(Rule.Type.And, "ruleDeclPlus", "ruleDecl", "plus");

      Result result = parser.Parse(grammar);

      Lexer = new Lexer();

      foreach (Result declOr in result.Children)
      {
        Result decl = declOr.Children[0];
        if (decl.Type == "token")
        {
          string name = decl.Children[0].Value;
          string regex = decl.Children[2].Value;
          Lexer.Rules.Add(new Lexer.Rule(name, regex, false));
        }
        else if (decl.Type == "delimToken")
        {
          string name = decl.Children[0].Value;
          string regex = decl.Children[3].Value;
          Lexer.Rules.Add(new Lexer.Rule(name, regex, true));
        }
        else if (decl.Type == "rule")
        {
          string name = decl.Children[0].Value;
          Result rule = decl.Children[2].Children[0];
        }
      }
    }

    public Result Parse(string input)
    {
      ConnectRules();
      Context context = new Context(Lexer.Lex(input));
      Result result = RootRule.Parse(context);
      if (context.Input.Any())
      {
        throw new Exception("Parse error: leftover tokens");
      }
      return result;
    }
  }
}
