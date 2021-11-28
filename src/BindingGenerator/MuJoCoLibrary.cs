using System.Collections.Generic;
using System.IO;
using System.Linq;
using System;
using System.Text.RegularExpressions;
using CppAst;

namespace BindingGenerator
{
    public class MuJoCoLibrary
    {
        private string libRoot;

        private string headerFile;

        public MuJoCoLibrary(string libRoot)
        {
            this.libRoot = Path.GetFullPath(libRoot);
            var includePath = Path.Combine(this.libRoot, "include");
            var libPath = Path.Combine(this.libRoot, "bin");

            this.headerFile = new DirectoryInfo(includePath).GetFiles().Where(x => x.Name=="mujoco.h").Select(x => x.FullName).FirstOrDefault();
        }
        public void ConvertToCSharp(string outputPath)
        {
            var options = new CppParserOptions
            {
                ParseMacros = true,
                Defines =
                {
                    "_WIN32",
                }
            };

            var csCompilation = CppParser.ParseFile(headerFile, options);
            if (csCompilation.HasErrors)
            {
                foreach (var message in csCompilation.Diagnostics.Messages)
                {
                    Console.WriteLine(message);
                }
                return;
            }

            CsCodeGenerator.Generate(csCompilation, outputPath);
        }
    }
}
