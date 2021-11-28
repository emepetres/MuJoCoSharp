using System;

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

	public unsafe delegate void mjfGeneric(
		 mjModel* m,
		 mjData* d);

	public unsafe delegate int mjfConFilt(
		 mjModel* m,
		 mjData* d,
		 int geom1,
		 int geom2);

	public unsafe delegate void mjfSensor(
		 mjModel* m,
		 mjData* d,
		 int stage);

	public unsafe delegate mjtNum mjfTime();

	public unsafe delegate mjtNum mjfAct(
		 mjModel* m,
		 mjData* d,
		 int id);

	public unsafe delegate int mjfCollision(
		 mjModel* m,
		 mjData* d,
		 mjContact* con,
		 int g1,
		 int g2,
		 mjtNum margin);

	public unsafe delegate int mjfItemEnable(
		 int category,
		 void* data);

}
