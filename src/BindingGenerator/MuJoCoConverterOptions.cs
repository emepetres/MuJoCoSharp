using CppAst.CodeGen.CSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace BindingGenerator
{
    public class MuJoCoConverterOptions : CSharpConverterOptions
    {
        public MuJoCoConverterOptions(string outputFilePath) : base()
        {
            DefaultNamespace = "MuJoCoSharp";
            DefaultOutputFilePath = GetUniformPath(outputFilePath);
            DefaultClassLib = "libnative";
            DefaultDllImportNameAndArguments = "\"mujoco210\"";
            GenerateAsInternal = false;
            GenerateEnumItemAsFields = true;
            TypedefCodeGenKind = CppTypedefCodeGenKind.Wrap;
            DefaultCharSet = CharSet.Ansi;
            AllowFixedSizeBuffers = true;
            DefaultMarshalForString = new CSharpMarshalAttribute(CSharpUnmanagedKind.LPStr);
            DefaultMarshalForBool = new CSharpMarshalAttribute(CSharpUnmanagedKind.U1);

            ParseMacros = true;
            
        }

        private static string GetUniformPath(string path)
        {
#if Windows
            var fullPath = Path.GetFullPath(path);
            var match = Regex.Match(fullPath, @"^([A-Z]):\\");
            var driveLetter = match.Groups[1].Value.ToLower();
            var uniformPath = Regex.Replace(Regex.Replace(fullPath, @"\\", "/"), @"^[A-Z]:", $"/mnt/{driveLetter}");
#elif Linux
            var uniformPath = Path.GetFullPath(path);
#endif
            return uniformPath;
        }
    }
}
