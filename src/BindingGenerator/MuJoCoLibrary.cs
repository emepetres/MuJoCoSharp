using System.IO;
using CppSharp;
using CppSharp.AST;
using CppSharp.Generators;
using CppSharp.Passes;

namespace BindingGenerator
{
    public class MuJoCoLibrary : ILibrary
    {
        private static string libName = "MuJoCo";
        private string libPath;

        public MuJoCoLibrary(string libPath)
        {
            this.libPath = Path.GetFullPath(libPath);
        }

        public void Setup(Driver driver)
        {
            var includePath = Path.Combine(libPath, "include");
            var includeFiles = Directory.GetFiles(includePath);
            var libDir = Path.Combine(libPath, "bin");
            var libFile = "libmujoco210.so";

            var options = driver.Options;
            options.Verbose = true;
            options.GeneratorKind = GeneratorKind.CSharp;
            //driver.ParserOptions.TargetTriple = "x86_64-linux-gnu";
            var module = options.AddModule(MuJoCoLibrary.libName);

            module.IncludeDirs.Add(includePath);
            module.Headers.AddRange(includeFiles);

            module.LibraryDirs.Add(libDir);
            module.Libraries.Add(libFile);
        }

        public void SetupPasses(Driver driver)
        {
            driver.Context.TranslationUnitPasses.RenameDeclsUpperCase(RenameTargets.Any);
            driver.Context.TranslationUnitPasses.AddPass(new FunctionToInstanceMethodPass());
        }

        public void Preprocess(Driver driver, ASTContext ctx)
        {
        }

        public void Postprocess(Driver driver, ASTContext ctx)
        {
        }
    }
}
