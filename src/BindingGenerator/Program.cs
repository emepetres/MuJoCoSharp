using System;
using CppSharp;

namespace BindingGenerator
{
    class Program
    {
        static void Main(string[] args)
        {
            var lib = new MuJoCoLibrary(@"./build/mujoco210");

            ConsoleDriver.Run(lib);
        }
    }
}
