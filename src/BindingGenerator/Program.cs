
namespace BindingGenerator
{
    class Program
    {
        static void Main(string[] args)
        {
            var lib = new MuJoCoLibrary(@"./lib/mujoco210");

            lib.ConvertToCSharp("./MuJoCoSharp/MuJoCo.cs");
        }
    }
}
