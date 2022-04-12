
namespace BindingGenerator
{
    class Program
    {
        static void Main(string[] args)
        {
            var lib = new MuJoCoLibrary(@"./lib/mujoco214");

            lib.ConvertToCSharp("./MuJoCoSharp/MuJoCo.cs");
        }
    }
}
