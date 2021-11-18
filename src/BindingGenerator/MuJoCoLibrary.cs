using System.IO;
using System.Linq;
using CppSharp;
using CppSharp.AST;
using CppSharp.Generators;
using CppSharp.Passes;

namespace BindingGenerator
{
    public class MuJoCoLibrary : ILibrary
    {
        private static string libName = "mujoco210";
        private string libRoot;

        public MuJoCoLibrary(string libRoot)
        {
            this.libRoot = Path.GetFullPath(libRoot);
        }

        public void Setup(Driver driver)
        {
            var options = driver.Options;
            var parserOptions = driver.ParserOptions;

            options.Verbose = true;
            parserOptions.Verbose = true;
            options.GeneratorKind = GeneratorKind.CSharp;
            var module = options.AddModule(MuJoCoLibrary.libName);

            var includePath = Path.Combine(this.libRoot, "include");
            var libPath = Path.Combine(this.libRoot, "bin");

            var includeFiles = new DirectoryInfo(includePath).GetFiles().Select(x => x.Name);
            // // var includeFiles = Directory.GetFiles(includePath);

            module.IncludeDirs.Add(includePath);
            module.Headers.AddRange(includeFiles);

#if Windows
            var libExt = "*.dll";
            parserOptions.SetupMSVC(VisualStudioVersion.VS2019);
            module.IncludeDirs.Add(@"C:\Program Files (x86)\Windows Kits\10\Include\10.0.18362.0\ucrt");
            module.IncludeDirs.Add(@"C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Tools\MSVC\14.28.29333\include");
#elif Linux
            var libExt = "*.so";
            //driver.ParserOptions.TargetTriple = "x86_64-linux-gnu";
#endif
            var libFiles = new DirectoryInfo(includePath).GetFiles(libExt).Select(x => x.Name);
            // // var libFiles = Directory.GetFiles(libPath, libExt);

            module.LibraryDirs.Add(libPath);
            // module.Libraries.Add(libFile);
            module.Libraries.AddRange(libFiles);
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
