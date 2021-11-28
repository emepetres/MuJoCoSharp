using CppAst;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Linq;

namespace BindingGenerator
{
    public static class CsCodeGenerator
    {
        private static string DefaultNamespace = "MuJoCoSharp";
        private static readonly IList<String> unsupportedFunctions = new ReadOnlyCollection<string>
            (new List<String> { });

        public static void Generate(CppCompilation compilation, string basePath)
        {
            string outputPath = basePath;
            Helpers.TypedefList = compilation.Typedefs
                    .Where(t => t.TypeKind == CppTypeKind.Typedef
                           && t.ElementType is CppPointerType
                           && ((CppPointerType)t.ElementType).ElementType.TypeKind != CppTypeKind.Function)
                    .Select(t => t.Name).ToList();

            GenerateDelegates(compilation, outputPath);
            GenerateEnums(compilation, outputPath);
            GenerateStructs(compilation, outputPath);
            GenerateCommmands(compilation, outputPath);
        }

        private static void GenerateDelegates(CppCompilation compilation, string outputPath)
        {
            Debug.WriteLine("Generating Delegates...");

            var delegates = compilation.Typedefs
                .Where(t => t.TypeKind == CppTypeKind.Typedef
                       && t.ElementType is CppPointerType
                       && ((CppPointerType)t.ElementType).ElementType.TypeKind == CppTypeKind.Function)
                .ToList();

            using (StreamWriter file = File.CreateText(Path.Combine(outputPath, "Delegates.cs")))
            {
                file.WriteLine("using System;\r\n");
                file.WriteLine($"namespace {DefaultNamespace}");
                file.WriteLine("{");

                foreach (var funcPointer in delegates)
                {
                    CppFunctionType pointerType = ((CppPointerType)funcPointer.ElementType).ElementType as CppFunctionType;

                    var returnType = Helpers.ConvertToCSharpType(pointerType.ReturnType);
                    file.Write($"\tpublic unsafe delegate {returnType} {funcPointer.Name}(");

                    if (pointerType.Parameters.Count > 0)
                    {
                        file.Write("\r\n");

                        for (int i = 0; i < pointerType.Parameters.Count; i++)
                        {
                            if (i > 0)
                                file.Write(",\r\n");

                            var parameter = pointerType.Parameters[i];
                            var convertedType = Helpers.ConvertToCSharpType(parameter.Type);
                            file.Write($"\t\t {convertedType} {parameter.Name}");
                        }
                    }

                    file.Write(");\r\n\r\n");
                }

                file.WriteLine("}");
            }
        }

        private static void GenerateCommmands(CppCompilation compilation, string outputPath)
        {
            Debug.WriteLine("Generating Commands...");

            string dllName = "mujoco210";

            using (StreamWriter file = File.CreateText(Path.Combine(outputPath, "Commands.cs")))
            {
                file.WriteLine("using System;");
                file.WriteLine("using System.Diagnostics;");
                file.WriteLine("using System.Runtime.InteropServices;\r\n");
                file.WriteLine($"namespace {DefaultNamespace}");
                file.WriteLine("{");
                file.WriteLine("\tpublic static unsafe partial class MuJoCo");
                file.WriteLine("\t{");
                file.WriteLine($"\t\tprivate const string dllName = \"{dllName}\";");

                foreach (var command in compilation.Functions)
                {
                    string convertedType = Helpers.ConvertToCSharpType(command.ReturnType, false);

                    if (!unsupportedFunctions.Contains(command.Name))
                    {
                        file.WriteLine("\r\n\t\t[DllImport(dllName)]");
                        file.WriteLine($"\t\tpublic static extern {convertedType} {command.Name}({Helpers.GetParametersSignature(command)});");
                    }
                }

                file.WriteLine("\t}");
                file.WriteLine("}");
            }
        }

        private static void GenerateStructs(CppCompilation compilation, string outputPath)
        {
            Debug.WriteLine("Generating Structs...");

            using (StreamWriter file = File.CreateText(Path.Combine(outputPath, "Structs.cs")))
            {
                file.WriteLine("using System;");
                file.WriteLine("using System.Runtime.InteropServices;\r\n");
                file.WriteLine($"namespace {DefaultNamespace}");
                file.WriteLine("{");

                var structs = compilation.Classes.Where(c => c.ClassKind == CppClassKind.Struct && c.IsDefinition == true);

                foreach (var structure in structs)
                {
                    file.WriteLine("\t[StructLayout(LayoutKind.Sequential)]");
                    file.WriteLine($"\tpublic unsafe struct {structure.Name}");
                    file.WriteLine("\t{");
                    foreach (var member in structure.Fields)
                    {
                        string type = "";
                        if (member.Type is CppClass complexMember)
                        {
                            if (complexMember.ClassKind == CppClassKind.Class)
                            {
                                Console.WriteLine($"WARN: Subclass {member.Name} treated as a struct.");
                            }
                            type = $"{structure.Name}_{member.Name}";
                            file.WriteLine("\t\t[StructLayout(LayoutKind.Sequential)]");
                            file.WriteLine($"\t\tpublic unsafe struct {type}");
                            file.WriteLine("\t\t{");

                            foreach (var field in complexMember.Fields)
                            {
                                var subtype = Helpers.ConvertToCSharpType(field.Type);

                                if (subtype == "bool")
                                {
                                    // This is to marshal bool as one byte
                                    // https://stackoverflow.com/questions/28514373/what-is-the-size-of-a-boolean-in-c-does-it-really-take-4-bytes/28515361
                                    // https://stackoverflow.com/questions/11416433/marshalling-non-blittable-structs-from-c-sharp-to-c
                                    // https://stackoverflow.com/questions/32110152/c-sharp-marshalling-bool
                                    file.WriteLine($"\t\t\t[MarshalAs(UnmanagedType.I1)]");
                                }

                                file.WriteLine($"\t\t\tpublic {subtype} {field.Name};");
                            }

                            file.WriteLine("\t\t}");
                        }
                        else
                        {
                            type = Helpers.ConvertToCSharpType(member.Type);

                            if (type == "bool")
                            {
                                // This is to marshal bool as one byte
                                // https://stackoverflow.com/questions/28514373/what-is-the-size-of-a-boolean-in-c-does-it-really-take-4-bytes/28515361
                                // https://stackoverflow.com/questions/11416433/marshalling-non-blittable-structs-from-c-sharp-to-c
                                // https://stackoverflow.com/questions/32110152/c-sharp-marshalling-bool
                                file.WriteLine($"\t\t[MarshalAs(UnmanagedType.I1)]");
                            }
                        }
                        file.WriteLine($"\t\tpublic {type} {member.Name};");
                    }

                    file.WriteLine("\t}\r\n");
                }
                file.WriteLine("}\r\n");
            }
        }

        public static void GenerateEnums(CppCompilation compilation, string outputPath)
        {
            Debug.WriteLine("Generating Enums...");

            using (StreamWriter file = File.CreateText(Path.Combine(outputPath, "Enums.cs")))
            {
                file.WriteLine("using System;\r\n");
                file.WriteLine($"namespace {DefaultNamespace}");
                file.WriteLine("{");

                foreach (var cppEnum in compilation.Enums)
                {
                    if (compilation.Typedefs.Any(t => t.Name == cppEnum.Name + "Flags"))
                    {
                        file.WriteLine("\t[Flags]");
                    }

                    file.WriteLine($"\tpublic enum {cppEnum.Name}");
                    file.WriteLine("\t{");

                    foreach (var member in cppEnum.Items)
                    {
                        file.WriteLine($"\t\t{member.Name} = {member.Value},");
                    }

                    file.WriteLine("\t}\r\n");
                }

                file.WriteLine("}");
            }
        }
    }
}
