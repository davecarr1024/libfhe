using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Sherp
{
    [TestClass]
    public class SherpTest
    {
        [TestMethod]
        public void TestLiterals()
        {
            Assert.AreEqual(new Vals.NoneType(), Sherp.Eval(@"None;"));
            Assert.AreEqual(new Vals.NoneType(), Sherp.Eval(@"NoneType();"));
            Assert.AreEqual(new Vals.Bool(false), Sherp.Eval(@"False;"));
            Assert.AreEqual(new Vals.Bool(true), Sherp.Eval(@"True;"));
            Assert.AreEqual(new Vals.Bool(false), Sherp.Eval(@"Bool(False);"));
            Assert.AreEqual(new Vals.Bool(true), Sherp.Eval(@"Bool(True);"));
        }
    }
}
