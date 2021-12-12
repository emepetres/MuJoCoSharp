/* example.i */
%module example
%{
/* Put header files here or function declarations like below */
lib/mujoco210/include/mujoco.h
%}

extern mjModel* mj_loadXML(const char* filename, const mjVFS* vfs,
                          char* error, int error_sz);
extern mjData *mj_makeData(const mjModel *m);
extern void mj_step(const mjModel *m, mjData *d);
extern void mj_deleteData(mjData *d);
extern void mj_deleteModel(mjModel* m);
