using MuJoCoSharp;
using System.Runtime.InteropServices;

using mjVFS = MuJoCoSharp._mjVFS;
using mjModel = MuJoCoSharp._mjModel;
using mjData = MuJoCoSharp._mjData;
using System.Diagnostics;

unsafe
{
    char* error = stackalloc char[1000];
    IntPtr error_ptr = new IntPtr(error);

    mjModel * m;
    mjData * d;

    // See https://aka.ms/new-console-template for more information
    Console.WriteLine("Hello, World!");


    // load model from file and check for errors
    ////var null_vfs = new mjVFS();
    m = MuJoCo.mj_loadXML("hello.xml", null, error, 1000);

    if (m == null)
    {
        var error_str = Marshal.PtrToStringAnsi(error_ptr);
        Debug.WriteLine(error_str);
        return 1;
    }

    // make data corresponding to model
    d = MuJoCo.mj_makeData(m);

    // run simulation for 10 seconds
    while (d->time < 10)
    {
        Debug.WriteLine(d->time);
        MuJoCo.mj_step(m, d);
    }

    // free model and data
    MuJoCo.mj_deleteData(d);
    MuJoCo.mj_deleteModel(m);

    return 0;
}
