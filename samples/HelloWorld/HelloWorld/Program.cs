using MuJoCoSharp;
using System.Diagnostics;
using System.Runtime.InteropServices;

unsafe
{
    char* error = stackalloc char[1000];
    IntPtr error_ptr = new IntPtr(error);
    libnative.mjModel m;
    libnative.mjData d;

    // See https://aka.ms/new-console-template for more information
    Console.WriteLine("Hello, World!");

    // load model from file and check for errors
    var null_vfs = new libnative.mjVFS();
    m = libnative.mj_loadXML("hello.xml", ref null_vfs, error_ptr, 1000);

    Debug.WriteLine("Error:");
    var error_str = Marshal.PtrToStringAnsi(error_ptr);
    Debug.WriteLine(error_str);

    // make data corresponding to model
    d = libnative.mj_makeData(ref m);

    // run simulation for 10 seconds
    while (d.Value.time < 10)
    {
        libnative.mj_step(ref m, ref d);
    }

    // free model and data
    libnative.mj_deleteData(ref d);
    libnative.mj_deleteModel(ref m);

    return 0;
}
