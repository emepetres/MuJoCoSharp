using System.Collections.Generic;
using System.IO;
using System.Linq;
using CppAst.CodeGen.Common;
using CppAst.CodeGen.CSharp;
using Zio;
using Zio.FileSystems;
using System;
using System.Text.RegularExpressions;
using CppAst;

namespace BindingGenerator
{
    public class MuJoCoLibrary
    {
        private string libRoot;

        private List<string> includeFiles;

        public MuJoCoLibrary(string libRoot)
        {
            this.libRoot = Path.GetFullPath(libRoot);
            var includePath = Path.Combine(this.libRoot, "include");
            var libPath = Path.Combine(this.libRoot, "bin");

            //this.includeFiles = new DirectoryInfo(includePath).GetFiles().Select(x => x.FullName).ToList();
            this.includeFiles = new DirectoryInfo(includePath).GetFiles().Where(x => x.Name=="mujoco.h").Select(x => x.FullName).ToList();
        }

        public void ConvertToCSharp(string outputFilePath)
        {
            var options = new MuJoCoConverterOptions(outputFilePath)
            {
                Defines = {
                    "_WIN32"
                },
                ////TypedefWrapWhiteList =
                ////{
                ////    "mjtNum"
                ////}
                ////MappingRules =
                ////{
                ////    ////e => e.MapMacroToConst("mjtNum", "double"),
                ////    ////e => e.MapMacroToConst("mjtByte", "byte"),
                ////    ////e => e.Map<CppElement>("mjtNum", "double"),
                ////    e => e.Map("mjtNum").Name("double"),
                ////}
            };

            var csCompilation = CSharpConverter.Convert(includeFiles, options);

            if (csCompilation.HasErrors)
            {
                foreach (var message in csCompilation.Diagnostics.Messages)
                {
                    Console.WriteLine(message);
                }
                return;
            }

            var fs = new PhysicalFileSystem();
            var writer = new CodeWriter(new CodeWriterOptions(fs));

            csCompilation.DumpTo(writer);

            //var text = fs.ReadAllText(options.DefaultOutputFilePath);
            //File.WriteAllText(@"C:\Users\jcarnero\_mujoco\lib.cs", text);
        }

        public static void Test()
        {
            var options = new CSharpConverterOptions();

            var csCompilation = CSharpConverter.Convert(@"
struct {
    int a;
    int b;
    void (*ptr)(int arg0, int arg1, void (*arg2)(int arg3));
    union
    {
        int c;
        int d;
    } e;
} outer;
            ", options);

            if (csCompilation.HasErrors)
            {
                Console.WriteLine($"ERROR: {csCompilation.Diagnostics}");
                return;
            }

            var fs = new MemoryFileSystem();
            var codeWriter = new CodeWriter(new CodeWriterOptions(fs));
            csCompilation.DumpTo(codeWriter);

            var text = fs.ReadAllText(options.DefaultOutputFilePath);
            Console.WriteLine(text);
        }
    }
}
