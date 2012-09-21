using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace terp
{
  public class LispInterpreter
  {
    private Parser Parser { get; set; }

    public LispInterpreter(string grammarFilename)
    {
      Parser = new Parser(File.ReadAllText(grammarFilename));
    }

  }
}
