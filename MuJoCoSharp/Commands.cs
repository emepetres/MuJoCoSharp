using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace MuJoCoSharp
{
	using mjtNum = Double;
	using mjtByte = Byte;
	using mjOption = _mjOption;
	using mjVisual = _mjVisual;
	using mjStatistic = _mjStatistic;
	using mjWarningStat = _mjWarningStat;
	using mjTimerStat = _mjTimerStat;
	using mjSolverStat = _mjSolverStat;
	using mjContact = _mjContact;
	using mjvGeom = _mjvGeom;
	using mjvLight = _mjvLight;
	using mjvGLCamera = _mjvLight;
	using mjrRect = _mjrRect;
	using mjuiItem = _mjuiItem;
	using mjuiThemeSpacing = _mjuiThemeSpacing;
	using mjuiThemeColor = _mjuiThemeColor;
	using mjuiSection = _mjuiSection;
	using mjVFS = _mjVFS;
	using mjModel = _mjModel;
	using mjData = _mjData;
	using mjLROpt = _mjLROpt;
	using mjvCamera = _mjvCamera;
	using mjvPerturb = _mjvPerturb;
	using mjvScene = _mjvScene;
	using mjvOption = _mjvOption;
	using mjvFigure = _mjvFigure;
	using mjrContext = _mjrContext;
	using mjUI = _mjUI;
	using mjuiDef = _mjuiDef;
	using mjuiState = _mjuiState;

	public static unsafe partial class MuJoCo
	{
		private const string dllName = "mujoco210";

		[DllImport(dllName)]
		public static extern int mj_activate([MarshalAs(UnmanagedType.LPStr)] string filename);

		[DllImport(dllName)]
		public static extern void mj_deactivate();

		[DllImport(dllName)]
		public static extern void mj_defaultVFS(mjVFS* vfs);

		[DllImport(dllName)]
		public static extern int mj_addFileVFS(mjVFS* vfs, [MarshalAs(UnmanagedType.LPStr)] string directory, [MarshalAs(UnmanagedType.LPStr)] string filename);

		[DllImport(dllName)]
		public static extern int mj_makeEmptyFileVFS(mjVFS* vfs, [MarshalAs(UnmanagedType.LPStr)] string filename, int filesize);

		[DllImport(dllName)]
		public static extern int mj_findFileVFS(mjVFS* vfs, [MarshalAs(UnmanagedType.LPStr)] string filename);

		[DllImport(dllName)]
		public static extern int mj_deleteFileVFS(mjVFS* vfs, [MarshalAs(UnmanagedType.LPStr)] string filename);

		[DllImport(dllName)]
		public static extern void mj_deleteVFS(mjVFS* vfs);

		[DllImport(dllName)]
		public static extern mjModel* mj_loadXML([MarshalAs(UnmanagedType.LPStr)] string filename, mjVFS* vfs, char* error, int error_sz);

		[DllImport(dllName)]
		public static extern int mj_saveLastXML([MarshalAs(UnmanagedType.LPStr)] string filename, mjModel* m, [MarshalAs(UnmanagedType.LPStr)] string error, int error_sz);

		[DllImport(dllName)]
		public static extern void mj_freeLastXML();

		[DllImport(dllName)]
		public static extern int mj_printSchema([MarshalAs(UnmanagedType.LPStr)] string filename, [MarshalAs(UnmanagedType.LPStr)] string buffer, int buffer_sz, int flg_html, int flg_pad);

		[DllImport(dllName)]
		public static extern void mj_step(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_step1(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_step2(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_forward(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_inverse(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_forwardSkip(mjModel* m, mjData* d, int skipstage, int skipsensor);

		[DllImport(dllName)]
		public static extern void mj_inverseSkip(mjModel* m, mjData* d, int skipstage, int skipsensor);

		[DllImport(dllName)]
		public static extern void mj_defaultLROpt(mjLROpt* opt);

		[DllImport(dllName)]
		public static extern void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp);

		[DllImport(dllName)]
		public static extern void mj_defaultOption(mjOption* opt);

		[DllImport(dllName)]
		public static extern void mj_defaultVisual(mjVisual* vis);

		[DllImport(dllName)]
		public static extern mjModel* mj_copyModel(mjModel* dest, mjModel* src);

		[DllImport(dllName)]
		public static extern void mj_saveModel(mjModel* m, [MarshalAs(UnmanagedType.LPStr)] string filename, void* buffer, int buffer_sz);

		[DllImport(dllName)]
		public static extern mjModel* mj_loadModel([MarshalAs(UnmanagedType.LPStr)] string filename, mjVFS* vfs);

		[DllImport(dllName)]
		public static extern void mj_deleteModel(mjModel* m);

		[DllImport(dllName)]
		public static extern int mj_sizeModel(mjModel* m);

		[DllImport(dllName)]
		public static extern mjData* mj_makeData(mjModel* m);

		[DllImport(dllName)]
		public static extern mjData* mj_copyData(mjData* dest, mjModel* m, mjData* src);

		[DllImport(dllName)]
		public static extern void mj_resetData(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_resetDataDebug(mjModel* m, mjData* d, byte debug_value);

		[DllImport(dllName)]
		public static extern void mj_resetDataKeyframe(mjModel* m, mjData* d, int key);

		[DllImport(dllName)]
		public static extern mjtNum* mj_stackAlloc(mjData* d, int size);

		[DllImport(dllName)]
		public static extern void mj_deleteData(mjData* d);

		[DllImport(dllName)]
		public static extern void mj_resetCallbacks();

		[DllImport(dllName)]
		public static extern void mj_setConst(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern int mj_setLengthRange(mjModel* m, mjData* d, int index, mjLROpt* opt, [MarshalAs(UnmanagedType.LPStr)] string error, int error_sz);

		[DllImport(dllName)]
		public static extern void mj_printModel(mjModel* m, [MarshalAs(UnmanagedType.LPStr)] string filename);

		[DllImport(dllName)]
		public static extern void mj_printData(mjModel* m, mjData* d, [MarshalAs(UnmanagedType.LPStr)] string filename);

		[DllImport(dllName)]
		public static extern void mju_printMat(mjtNum* mat, int nr, int nc);

		[DllImport(dllName)]
		public static extern void mju_printMatSparse(mjtNum* mat, int nr, int* rownnz, int* rowadr, int* colind);

		[DllImport(dllName)]
		public static extern void mj_fwdPosition(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_fwdVelocity(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_fwdActuation(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_fwdAcceleration(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_fwdConstraint(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_Euler(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_RungeKutta(mjModel* m, mjData* d, int N);

		[DllImport(dllName)]
		public static extern void mj_invPosition(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_invVelocity(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_invConstraint(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_compareFwdInv(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_sensorPos(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_sensorVel(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_sensorAcc(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_energyPos(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_energyVel(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_checkPos(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_checkVel(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_checkAcc(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_kinematics(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_comPos(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_camlight(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_tendon(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_transmission(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_crb(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_factorM(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_solveM(mjModel* m, mjData* d, mjtNum* x, mjtNum* y, int n);

		[DllImport(dllName)]
		public static extern void mj_solveM2(mjModel* m, mjData* d, mjtNum* x, mjtNum* y, int n);

		[DllImport(dllName)]
		public static extern void mj_comVel(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_passive(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_subtreeVel(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_rne(mjModel* m, mjData* d, int flg_acc, mjtNum* result);

		[DllImport(dllName)]
		public static extern void mj_rnePostConstraint(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_collision(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_makeConstraint(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_projectConstraint(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_referenceConstraint(mjModel* m, mjData* d);

		[DllImport(dllName)]
		public static extern void mj_constraintUpdate(mjModel* m, mjData* d, mjtNum* jar, mjtNum* cost, int flg_coneHessian);

		[DllImport(dllName)]
		public static extern int mj_addContact(mjModel* m, mjData* d, mjContact* con);

		[DllImport(dllName)]
		public static extern int mj_isPyramidal(mjModel* m);

		[DllImport(dllName)]
		public static extern int mj_isSparse(mjModel* m);

		[DllImport(dllName)]
		public static extern int mj_isDual(mjModel* m);

		[DllImport(dllName)]
		public static extern void mj_mulJacVec(mjModel* m, mjData* d, mjtNum* res, mjtNum* vec);

		[DllImport(dllName)]
		public static extern void mj_mulJacTVec(mjModel* m, mjData* d, mjtNum* res, mjtNum* vec);

		[DllImport(dllName)]
		public static extern void mj_jac(mjModel* m, mjData* d, mjtNum* jacp, mjtNum* jacr, mjtNum point, int body);

		[DllImport(dllName)]
		public static extern void mj_jacBody(mjModel* m, mjData* d, mjtNum* jacp, mjtNum* jacr, int body);

		[DllImport(dllName)]
		public static extern void mj_jacBodyCom(mjModel* m, mjData* d, mjtNum* jacp, mjtNum* jacr, int body);

		[DllImport(dllName)]
		public static extern void mj_jacGeom(mjModel* m, mjData* d, mjtNum* jacp, mjtNum* jacr, int geom);

		[DllImport(dllName)]
		public static extern void mj_jacSite(mjModel* m, mjData* d, mjtNum* jacp, mjtNum* jacr, int site);

		[DllImport(dllName)]
		public static extern void mj_jacPointAxis(mjModel* m, mjData* d, mjtNum* jacPoint, mjtNum* jacAxis, mjtNum point, mjtNum axis, int body);

		[DllImport(dllName)]
		public static extern int mj_name2id(mjModel* m, int type, [MarshalAs(UnmanagedType.LPStr)] string name);

		[DllImport(dllName)]
		public static extern char* mj_id2name(mjModel* m, int type, int id);

		[DllImport(dllName)]
		public static extern void mj_fullM(mjModel* m, mjtNum* dst, mjtNum* M);

		[DllImport(dllName)]
		public static extern void mj_mulM(mjModel* m, mjData* d, mjtNum* res, mjtNum* vec);

		[DllImport(dllName)]
		public static extern void mj_mulM2(mjModel* m, mjData* d, mjtNum* res, mjtNum* vec);

		[DllImport(dllName)]
		public static extern void mj_addM(mjModel* m, mjData* d, mjtNum* dst, int* rownnz, int* rowadr, int* colind);

		[DllImport(dllName)]
		public static extern void mj_applyFT(mjModel* m, mjData* d, mjtNum* force, mjtNum* torque, mjtNum* point, int body, mjtNum* qfrc_target);

		[DllImport(dllName)]
		public static extern void mj_objectVelocity(mjModel* m, mjData* d, int objtype, int objid, mjtNum* res, int flg_local);

		[DllImport(dllName)]
		public static extern void mj_objectAcceleration(mjModel* m, mjData* d, int objtype, int objid, mjtNum* res, int flg_local);

		[DllImport(dllName)]
		public static extern void mj_contactForce(mjModel* m, mjData* d, int id, mjtNum* result);

		[DllImport(dllName)]
		public static extern void mj_differentiatePos(mjModel* m, mjtNum* qvel, mjtNum dt, mjtNum* qpos1, mjtNum* qpos2);

		[DllImport(dllName)]
		public static extern void mj_integratePos(mjModel* m, mjtNum* qpos, mjtNum* qvel, mjtNum dt);

		[DllImport(dllName)]
		public static extern void mj_normalizeQuat(mjModel* m, mjtNum* qpos);

		[DllImport(dllName)]
		public static extern void mj_local2Global(mjData* d, mjtNum* xpos, mjtNum* xmat, mjtNum* pos, mjtNum* quat, int body, mjtByte sameframe);

		[DllImport(dllName)]
		public static extern mjtNum mj_getTotalmass(mjModel* m);

		[DllImport(dllName)]
		public static extern void mj_setTotalmass(mjModel* m, mjtNum newmass);

		[DllImport(dllName)]
		public static extern int mj_version();

		[DllImport(dllName)]
		public static extern mjtNum mj_ray(mjModel* m, mjData* d, mjtNum* pnt, mjtNum* vec, mjtByte* geomgroup, mjtByte flg_static, int bodyexclude, int* geomid);

		[DllImport(dllName)]
		public static extern mjtNum mj_rayHfield(mjModel* m, mjData* d, int geomid, mjtNum* pnt, mjtNum* vec);

		[DllImport(dllName)]
		public static extern mjtNum mj_rayMesh(mjModel* m, mjData* d, int geomid, mjtNum* pnt, mjtNum* vec);

		[DllImport(dllName)]
		public static extern mjtNum mju_rayGeom(mjtNum* pos, mjtNum* mat, mjtNum* size, mjtNum* pnt, mjtNum* vec, int geomtype);

		[DllImport(dllName)]
		public static extern mjtNum mju_raySkin(int nface, int nvert, int* face, float* vert, mjtNum* pnt, mjtNum* vec, int* vertid);

		[DllImport(dllName)]
		public static extern void mjv_defaultCamera(mjvCamera* cam);

		[DllImport(dllName)]
		public static extern void mjv_defaultPerturb(mjvPerturb* pert);

		[DllImport(dllName)]
		public static extern void mjv_room2model(mjtNum* modelpos, mjtNum* modelquat, mjtNum* roompos, mjtNum* roomquat, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_model2room(mjtNum* roompos, mjtNum* roomquat, mjtNum* modelpos, mjtNum* modelquat, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_cameraInModel(mjtNum* headpos, mjtNum* forward, mjtNum* up, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_cameraInRoom(mjtNum* headpos, mjtNum* forward, mjtNum* up, mjvScene* scn);

		[DllImport(dllName)]
		public static extern mjtNum mjv_frustumHeight(mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_alignToCamera(mjtNum* res, mjtNum* vec, mjtNum* forward);

		[DllImport(dllName)]
		public static extern void mjv_moveCamera(mjModel* m, int action, mjtNum reldx, mjtNum reldy, mjvScene* scn, mjvCamera* cam);

		[DllImport(dllName)]
		public static extern void mjv_movePerturb(mjModel* m, mjData* d, int action, mjtNum reldx, mjtNum reldy, mjvScene* scn, mjvPerturb* pert);

		[DllImport(dllName)]
		public static extern void mjv_moveModel(mjModel* m, int action, mjtNum reldx, mjtNum reldy, mjtNum* roomup, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_initPerturb(mjModel* m, mjData* d, mjvScene* scn, mjvPerturb* pert);

		[DllImport(dllName)]
		public static extern void mjv_applyPerturbPose(mjModel* m, mjData* d, mjvPerturb* pert, int flg_paused);

		[DllImport(dllName)]
		public static extern void mjv_applyPerturbForce(mjModel* m, mjData* d, mjvPerturb* pert);

		[DllImport(dllName)]
		public static extern mjvGLCamera mjv_averageCamera(mjvGLCamera* cam1, mjvGLCamera* cam2);

		[DllImport(dllName)]
		public static extern int mjv_select(mjModel* m, mjData* d, mjvOption* vopt, mjtNum aspectratio, mjtNum relx, mjtNum rely, mjvScene* scn, mjtNum* selpnt, int* geomid, int* skinid);

		[DllImport(dllName)]
		public static extern void mjv_defaultOption(mjvOption* opt);

		[DllImport(dllName)]
		public static extern void mjv_defaultFigure(mjvFigure* fig);

		[DllImport(dllName)]
		public static extern void mjv_initGeom(mjvGeom* geom, int type, mjtNum* size, mjtNum* pos, mjtNum* mat, float* rgba);

		[DllImport(dllName)]
		public static extern void mjv_makeConnector(mjvGeom* geom, int type, mjtNum width, mjtNum a0, mjtNum a1, mjtNum a2, mjtNum b0, mjtNum b1, mjtNum b2);

		[DllImport(dllName)]
		public static extern void mjv_defaultScene(mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_makeScene(mjModel* m, mjvScene* scn, int maxgeom);

		[DllImport(dllName)]
		public static extern void mjv_freeScene(mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_updateScene(mjModel* m, mjData* d, mjvOption* opt, mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_addGeoms(mjModel* m, mjData* d, mjvOption* opt, mjvPerturb* pert, int catmask, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_makeLights(mjModel* m, mjData* d, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_updateCamera(mjModel* m, mjData* d, mjvCamera* cam, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjv_updateSkin(mjModel* m, mjData* d, mjvScene* scn);

		[DllImport(dllName)]
		public static extern void mjr_defaultContext(mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_makeContext(mjModel* m, mjrContext* con, int fontscale);

		[DllImport(dllName)]
		public static extern void mjr_changeFont(int fontscale, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_addAux(int index, int width, int height, int samples, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_freeContext(mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_uploadTexture(mjModel* m, mjrContext* con, int texid);

		[DllImport(dllName)]
		public static extern void mjr_uploadMesh(mjModel* m, mjrContext* con, int meshid);

		[DllImport(dllName)]
		public static extern void mjr_uploadHField(mjModel* m, mjrContext* con, int hfieldid);

		[DllImport(dllName)]
		public static extern void mjr_restoreBuffer(mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_setBuffer(int framebuffer, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_readPixels(byte* rgb, float* depth, mjrRect viewport, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_drawPixels(byte* rgb, float* depth, mjrRect viewport, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_blitBuffer(mjrRect src, mjrRect dst, int flg_color, int flg_depth, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_setAux(int index, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_blitAux(int index, mjrRect src, int left, int bottom, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_text(int font, [MarshalAs(UnmanagedType.LPStr)] string txt, mjrContext* con, float x, float y, float r, float g, float b);

		[DllImport(dllName)]
		public static extern void mjr_overlay(int font, int gridpos, mjrRect viewport, [MarshalAs(UnmanagedType.LPStr)] string overlay, [MarshalAs(UnmanagedType.LPStr)] string overlay2, mjrContext* con);

		[DllImport(dllName)]
		public static extern mjrRect mjr_maxViewport(mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a);

		[DllImport(dllName)]
		public static extern void mjr_label(mjrRect viewport, int font, [MarshalAs(UnmanagedType.LPStr)] string txt, float r, float g, float b, float a, float rt, float gt, float bt, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_figure(mjrRect viewport, mjvFigure* fig, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_render(mjrRect viewport, mjvScene* scn, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjr_finish();

		[DllImport(dllName)]
		public static extern int mjr_getError();

		[DllImport(dllName)]
		public static extern int mjr_findRect(int x, int y, int nrect, mjrRect* rect);

		[DllImport(dllName)]
		public static extern mjuiThemeSpacing mjui_themeSpacing(int ind);

		[DllImport(dllName)]
		public static extern mjuiThemeColor mjui_themeColor(int ind);

		[DllImport(dllName)]
		public static extern void mjui_add(ref mjUI ui, mjuiDef* def);

		[DllImport(dllName)]
		public static extern void mjui_addToSection(ref mjUI ui, int sect, mjuiDef* def);

		[DllImport(dllName)]
		public static extern void mjui_resize(ref mjUI ui, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjui_update(int section, int item, ref mjUI ui, mjuiState* state, mjrContext* con);

		[DllImport(dllName)]
		public static extern mjuiItem* mjui_event(ref mjUI ui, mjuiState* state, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mjui_render(ref mjUI ui, mjuiState* state, mjrContext* con);

		[DllImport(dllName)]
		public static extern void mju_error([MarshalAs(UnmanagedType.LPStr)] string msg);

		[DllImport(dllName)]
		public static extern void mju_error_i([MarshalAs(UnmanagedType.LPStr)] string msg, int i);

		[DllImport(dllName)]
		public static extern void mju_error_s([MarshalAs(UnmanagedType.LPStr)] string msg, [MarshalAs(UnmanagedType.LPStr)] string text);

		[DllImport(dllName)]
		public static extern void mju_warning([MarshalAs(UnmanagedType.LPStr)] string msg);

		[DllImport(dllName)]
		public static extern void mju_warning_i([MarshalAs(UnmanagedType.LPStr)] string msg, int i);

		[DllImport(dllName)]
		public static extern void mju_warning_s([MarshalAs(UnmanagedType.LPStr)] string msg, [MarshalAs(UnmanagedType.LPStr)] string text);

		[DllImport(dllName)]
		public static extern void mju_clearHandlers();

		[DllImport(dllName)]
		public static extern void* mju_malloc(ulong size);

		[DllImport(dllName)]
		public static extern void mju_free(void* ptr);

		[DllImport(dllName)]
		public static extern void mj_warning(mjData* d, int warning, int info);

		[DllImport(dllName)]
		public static extern void mju_writeLog([MarshalAs(UnmanagedType.LPStr)] string type, [MarshalAs(UnmanagedType.LPStr)] string msg);

		[DllImport(dllName)]
		public static extern void mju_zero3(mjtNum res);

		[DllImport(dllName)]
		public static extern void mju_copy3(mjtNum res, mjtNum data);

		[DllImport(dllName)]
		public static extern void mju_scl3(mjtNum res, mjtNum vec, mjtNum scl);

		[DllImport(dllName)]
		public static extern void mju_add3(mjtNum res, mjtNum vec1, mjtNum vec2);

		[DllImport(dllName)]
		public static extern void mju_sub3(mjtNum res, mjtNum vec1, mjtNum vec2);

		[DllImport(dllName)]
		public static extern void mju_addTo3(mjtNum res, mjtNum vec);

		[DllImport(dllName)]
		public static extern void mju_subFrom3(mjtNum res, mjtNum vec);

		[DllImport(dllName)]
		public static extern void mju_addToScl3(mjtNum res, mjtNum vec, mjtNum scl);

		[DllImport(dllName)]
		public static extern void mju_addScl3(mjtNum res, mjtNum vec1, mjtNum vec2, mjtNum scl);

		[DllImport(dllName)]
		public static extern mjtNum mju_normalize3(mjtNum res);

		[DllImport(dllName)]
		public static extern mjtNum mju_norm3(mjtNum vec);

		[DllImport(dllName)]
		public static extern mjtNum mju_dot3(mjtNum vec1, mjtNum vec2);

		[DllImport(dllName)]
		public static extern mjtNum mju_dist3(mjtNum pos1, mjtNum pos2);

		[DllImport(dllName)]
		public static extern void mju_rotVecMat(mjtNum res, mjtNum vec, mjtNum mat);

		[DllImport(dllName)]
		public static extern void mju_rotVecMatT(mjtNum res, mjtNum vec, mjtNum mat);

		[DllImport(dllName)]
		public static extern void mju_cross(mjtNum res, mjtNum a, mjtNum b);

		[DllImport(dllName)]
		public static extern void mju_zero4(mjtNum res);

		[DllImport(dllName)]
		public static extern void mju_unit4(mjtNum res);

		[DllImport(dllName)]
		public static extern void mju_copy4(mjtNum res, mjtNum data);

		[DllImport(dllName)]
		public static extern mjtNum mju_normalize4(mjtNum res);

		[DllImport(dllName)]
		public static extern void mju_zero(mjtNum* res, int n);

		[DllImport(dllName)]
		public static extern void mju_copy(mjtNum* res, mjtNum* data, int n);

		[DllImport(dllName)]
		public static extern mjtNum mju_sum(mjtNum* vec, int n);

		[DllImport(dllName)]
		public static extern mjtNum mju_L1(mjtNum* vec, int n);

		[DllImport(dllName)]
		public static extern void mju_scl(mjtNum* res, mjtNum* vec, mjtNum scl, int n);

		[DllImport(dllName)]
		public static extern void mju_add(mjtNum* res, mjtNum* vec1, mjtNum* vec2, int n);

		[DllImport(dllName)]
		public static extern void mju_sub(mjtNum* res, mjtNum* vec1, mjtNum* vec2, int n);

		[DllImport(dllName)]
		public static extern void mju_addTo(mjtNum* res, mjtNum* vec, int n);

		[DllImport(dllName)]
		public static extern void mju_subFrom(mjtNum* res, mjtNum* vec, int n);

		[DllImport(dllName)]
		public static extern void mju_addToScl(mjtNum* res, mjtNum* vec, mjtNum scl, int n);

		[DllImport(dllName)]
		public static extern void mju_addScl(mjtNum* res, mjtNum* vec1, mjtNum* vec2, mjtNum scl, int n);

		[DllImport(dllName)]
		public static extern mjtNum mju_normalize(mjtNum* res, int n);

		[DllImport(dllName)]
		public static extern mjtNum mju_norm(mjtNum* res, int n);

		[DllImport(dllName)]
		public static extern mjtNum mju_dot(mjtNum* vec1, mjtNum* vec2, int n);

		[DllImport(dllName)]
		public static extern void mju_mulMatVec(mjtNum* res, mjtNum* mat, mjtNum* vec, int nr, int nc);

		[DllImport(dllName)]
		public static extern void mju_mulMatTVec(mjtNum* res, mjtNum* mat, mjtNum* vec, int nr, int nc);

		[DllImport(dllName)]
		public static extern void mju_transpose(mjtNum* res, mjtNum* mat, int nr, int nc);

		[DllImport(dllName)]
		public static extern void mju_mulMatMat(mjtNum* res, mjtNum* mat1, mjtNum* mat2, int r1, int c1, int c2);

		[DllImport(dllName)]
		public static extern void mju_mulMatMatT(mjtNum* res, mjtNum* mat1, mjtNum* mat2, int r1, int c1, int r2);

		[DllImport(dllName)]
		public static extern void mju_mulMatTMat(mjtNum* res, mjtNum* mat1, mjtNum* mat2, int r1, int c1, int c2);

		[DllImport(dllName)]
		public static extern void mju_sqrMatTD(mjtNum* res, mjtNum* mat, mjtNum* diag, int nr, int nc);

		[DllImport(dllName)]
		public static extern void mju_transformSpatial(mjtNum res, mjtNum vec, int flg_force, mjtNum newpos, mjtNum oldpos, mjtNum rotnew2old);

		[DllImport(dllName)]
		public static extern void mju_rotVecQuat(mjtNum res, mjtNum vec, mjtNum quat);

		[DllImport(dllName)]
		public static extern void mju_negQuat(mjtNum res, mjtNum quat);

		[DllImport(dllName)]
		public static extern void mju_mulQuat(mjtNum res, mjtNum quat1, mjtNum quat2);

		[DllImport(dllName)]
		public static extern void mju_mulQuatAxis(mjtNum res, mjtNum quat, mjtNum axis);

		[DllImport(dllName)]
		public static extern void mju_axisAngle2Quat(mjtNum res, mjtNum axis, mjtNum angle);

		[DllImport(dllName)]
		public static extern void mju_quat2Vel(mjtNum res, mjtNum quat, mjtNum dt);

		[DllImport(dllName)]
		public static extern void mju_subQuat(mjtNum res, mjtNum qa, mjtNum qb);

		[DllImport(dllName)]
		public static extern void mju_quat2Mat(mjtNum res, mjtNum quat);

		[DllImport(dllName)]
		public static extern void mju_mat2Quat(mjtNum quat, mjtNum mat);

		[DllImport(dllName)]
		public static extern void mju_derivQuat(mjtNum res, mjtNum quat, mjtNum vel);

		[DllImport(dllName)]
		public static extern void mju_quatIntegrate(mjtNum quat, mjtNum vel, mjtNum scale);

		[DllImport(dllName)]
		public static extern void mju_quatZ2Vec(mjtNum quat, mjtNum vec);

		[DllImport(dllName)]
		public static extern void mju_mulPose(mjtNum posres, mjtNum quatres, mjtNum pos1, mjtNum quat1, mjtNum pos2, mjtNum quat2);

		[DllImport(dllName)]
		public static extern void mju_negPose(mjtNum posres, mjtNum quatres, mjtNum pos, mjtNum quat);

		[DllImport(dllName)]
		public static extern void mju_trnVecPose(mjtNum res, mjtNum pos, mjtNum quat, mjtNum vec);

		[DllImport(dllName)]
		public static extern int mju_cholFactor(mjtNum* mat, int n, mjtNum mindiag);

		[DllImport(dllName)]
		public static extern void mju_cholSolve(mjtNum* res, mjtNum* mat, mjtNum* vec, int n);

		[DllImport(dllName)]
		public static extern int mju_cholUpdate(mjtNum* mat, mjtNum* x, int n, int flg_plus);

		[DllImport(dllName)]
		public static extern int mju_eig3(mjtNum* eigval, mjtNum* eigvec, mjtNum* quat, mjtNum* mat);

		[DllImport(dllName)]
		public static extern mjtNum mju_muscleGain(mjtNum len, mjtNum vel, mjtNum lengthrange, mjtNum acc0, mjtNum prm);

		[DllImport(dllName)]
		public static extern mjtNum mju_muscleBias(mjtNum len, mjtNum lengthrange, mjtNum acc0, mjtNum prm);

		[DllImport(dllName)]
		public static extern mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, mjtNum prm);

		[DllImport(dllName)]
		public static extern void mju_encodePyramid(mjtNum* pyramid, mjtNum* force, mjtNum* mu, int dim);

		[DllImport(dllName)]
		public static extern void mju_decodePyramid(mjtNum* force, mjtNum* pyramid, mjtNum* mu, int dim);

		[DllImport(dllName)]
		public static extern mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt);

		[DllImport(dllName)]
		public static extern mjtNum mju_min(mjtNum a, mjtNum b);

		[DllImport(dllName)]
		public static extern mjtNum mju_max(mjtNum a, mjtNum b);

		[DllImport(dllName)]
		public static extern mjtNum mju_sign(mjtNum x);

		[DllImport(dllName)]
		public static extern int mju_round(mjtNum x);

		[DllImport(dllName)]
		public static extern char* mju_type2Str(int type);

		[DllImport(dllName)]
		public static extern int mju_str2Type([MarshalAs(UnmanagedType.LPStr)] string str);

		[DllImport(dllName)]
		public static extern char* mju_warningText(int warning, int info);

		[DllImport(dllName)]
		public static extern int mju_isBad(mjtNum x);

		[DllImport(dllName)]
		public static extern int mju_isZero(mjtNum* vec, int n);

		[DllImport(dllName)]
		public static extern mjtNum mju_standardNormal(mjtNum* num2);

		[DllImport(dllName)]
		public static extern void mju_f2n(mjtNum* res, float* vec, int n);

		[DllImport(dllName)]
		public static extern void mju_n2f(float* res, mjtNum* vec, int n);

		[DllImport(dllName)]
		public static extern void mju_d2n(mjtNum* res, double* vec, int n);

		[DllImport(dllName)]
		public static extern void mju_n2d(double* res, mjtNum* vec, int n);

		[DllImport(dllName)]
		public static extern void mju_insertionSort(mjtNum* list, int n);

		[DllImport(dllName)]
		public static extern void mju_insertionSortInt(int* list, int n);

		[DllImport(dllName)]
		public static extern mjtNum mju_Halton(int index, int @base);

		[DllImport(dllName)]
		public static extern char* mju_strncpy([MarshalAs(UnmanagedType.LPStr)] string dst, [MarshalAs(UnmanagedType.LPStr)] string src, int n);

		[DllImport(dllName)]
		public static extern mjtNum mju_sigmoid(mjtNum x);
	}
}
