using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using terp;

namespace interp
{
    class Program
    {
        static void Main(string[] args)
        {
            VirtualMachine.Assembly assembly = new VirtualMachine.Assembly()
            {
                Namespaces =
                {
                    new VirtualMachine.Namespace()
                    {
                        Name = "test",
                        Classes =
                        {
                            new VirtualMachine.Class()
                            {
                                Name = "Program",
                                Methods = 
                                {
                                    new VirtualMachine.Method()
                                    {
                                        Name = "Main",
                                        ArgNames = {"this"},
                                        Statements = 
                                        {
                                            new VirtualMachine.Assignment()
                                            {
                                                Name = "a",
                                                Value = new VirtualMachine.Literal()
                                                {
                                                    Value = new VirtualMachine.Int() { Value = 12 }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            };
            VirtualMachine.Value value = VirtualMachine.Execute(assembly);
        }
    }
}
