using System;

namespace MuJoCoSharp
{
	public enum _mjtDisableBit
	{
		mjDSBL_CONSTRAINT = 1,
		mjDSBL_EQUALITY = 2,
		mjDSBL_FRICTIONLOSS = 4,
		mjDSBL_LIMIT = 8,
		mjDSBL_CONTACT = 16,
		mjDSBL_PASSIVE = 32,
		mjDSBL_GRAVITY = 64,
		mjDSBL_CLAMPCTRL = 128,
		mjDSBL_WARMSTART = 256,
		mjDSBL_FILTERPARENT = 512,
		mjDSBL_ACTUATION = 1024,
		mjDSBL_REFSAFE = 2048,
		mjNDISABLE = 12,
	}

	public enum _mjtEnableBit
	{
		mjENBL_OVERRIDE = 1,
		mjENBL_ENERGY = 2,
		mjENBL_FWDINV = 4,
		mjENBL_SENSORNOISE = 8,
		mjNENABLE = 4,
	}

	public enum _mjtJoint
	{
		mjJNT_FREE = 0,
		mjJNT_BALL = 1,
		mjJNT_SLIDE = 2,
		mjJNT_HINGE = 3,
	}

	public enum _mjtGeom
	{
		mjGEOM_PLANE = 0,
		mjGEOM_HFIELD = 1,
		mjGEOM_SPHERE = 2,
		mjGEOM_CAPSULE = 3,
		mjGEOM_ELLIPSOID = 4,
		mjGEOM_CYLINDER = 5,
		mjGEOM_BOX = 6,
		mjGEOM_MESH = 7,
		mjNGEOMTYPES = 8,
		mjGEOM_ARROW = 100,
		mjGEOM_ARROW1 = 101,
		mjGEOM_ARROW2 = 102,
		mjGEOM_LINE = 103,
		mjGEOM_SKIN = 104,
		mjGEOM_LABEL = 105,
		mjGEOM_NONE = 1001,
	}

	public enum _mjtCamLight
	{
		mjCAMLIGHT_FIXED = 0,
		mjCAMLIGHT_TRACK = 1,
		mjCAMLIGHT_TRACKCOM = 2,
		mjCAMLIGHT_TARGETBODY = 3,
		mjCAMLIGHT_TARGETBODYCOM = 4,
	}

	public enum _mjtTexture
	{
		mjTEXTURE_2D = 0,
		mjTEXTURE_CUBE = 1,
		mjTEXTURE_SKYBOX = 2,
	}

	public enum _mjtIntegrator
	{
		mjINT_EULER = 0,
		mjINT_RK4 = 1,
	}

	public enum _mjtCollision
	{
		mjCOL_ALL = 0,
		mjCOL_PAIR = 1,
		mjCOL_DYNAMIC = 2,
	}

	public enum _mjtCone
	{
		mjCONE_PYRAMIDAL = 0,
		mjCONE_ELLIPTIC = 1,
	}

	public enum _mjtJacobian
	{
		mjJAC_DENSE = 0,
		mjJAC_SPARSE = 1,
		mjJAC_AUTO = 2,
	}

	public enum _mjtSolver
	{
		mjSOL_PGS = 0,
		mjSOL_CG = 1,
		mjSOL_NEWTON = 2,
	}

	public enum _mjtEq
	{
		mjEQ_CONNECT = 0,
		mjEQ_WELD = 1,
		mjEQ_JOINT = 2,
		mjEQ_TENDON = 3,
		mjEQ_DISTANCE = 4,
	}

	public enum _mjtWrap
	{
		mjWRAP_NONE = 0,
		mjWRAP_JOINT = 1,
		mjWRAP_PULLEY = 2,
		mjWRAP_SITE = 3,
		mjWRAP_SPHERE = 4,
		mjWRAP_CYLINDER = 5,
	}

	public enum _mjtTrn
	{
		mjTRN_JOINT = 0,
		mjTRN_JOINTINPARENT = 1,
		mjTRN_SLIDERCRANK = 2,
		mjTRN_TENDON = 3,
		mjTRN_SITE = 4,
		mjTRN_UNDEFINED = 1000,
	}

	public enum _mjtDyn
	{
		mjDYN_NONE = 0,
		mjDYN_INTEGRATOR = 1,
		mjDYN_FILTER = 2,
		mjDYN_MUSCLE = 3,
		mjDYN_USER = 4,
	}

	public enum _mjtGain
	{
		mjGAIN_FIXED = 0,
		mjGAIN_MUSCLE = 1,
		mjGAIN_USER = 2,
	}

	public enum _mjtBias
	{
		mjBIAS_NONE = 0,
		mjBIAS_AFFINE = 1,
		mjBIAS_MUSCLE = 2,
		mjBIAS_USER = 3,
	}

	public enum _mjtObj
	{
		mjOBJ_UNKNOWN = 0,
		mjOBJ_BODY = 1,
		mjOBJ_XBODY = 2,
		mjOBJ_JOINT = 3,
		mjOBJ_DOF = 4,
		mjOBJ_GEOM = 5,
		mjOBJ_SITE = 6,
		mjOBJ_CAMERA = 7,
		mjOBJ_LIGHT = 8,
		mjOBJ_MESH = 9,
		mjOBJ_SKIN = 10,
		mjOBJ_HFIELD = 11,
		mjOBJ_TEXTURE = 12,
		mjOBJ_MATERIAL = 13,
		mjOBJ_PAIR = 14,
		mjOBJ_EXCLUDE = 15,
		mjOBJ_EQUALITY = 16,
		mjOBJ_TENDON = 17,
		mjOBJ_ACTUATOR = 18,
		mjOBJ_SENSOR = 19,
		mjOBJ_NUMERIC = 20,
		mjOBJ_TEXT = 21,
		mjOBJ_TUPLE = 22,
		mjOBJ_KEY = 23,
	}

	public enum _mjtConstraint
	{
		mjCNSTR_EQUALITY = 0,
		mjCNSTR_FRICTION_DOF = 1,
		mjCNSTR_FRICTION_TENDON = 2,
		mjCNSTR_LIMIT_JOINT = 3,
		mjCNSTR_LIMIT_TENDON = 4,
		mjCNSTR_CONTACT_FRICTIONLESS = 5,
		mjCNSTR_CONTACT_PYRAMIDAL = 6,
		mjCNSTR_CONTACT_ELLIPTIC = 7,
	}

	public enum _mjtConstraintState
	{
		mjCNSTRSTATE_SATISFIED = 0,
		mjCNSTRSTATE_QUADRATIC = 1,
		mjCNSTRSTATE_LINEARNEG = 2,
		mjCNSTRSTATE_LINEARPOS = 3,
		mjCNSTRSTATE_CONE = 4,
	}

	public enum _mjtSensor
	{
		mjSENS_TOUCH = 0,
		mjSENS_ACCELEROMETER = 1,
		mjSENS_VELOCIMETER = 2,
		mjSENS_GYRO = 3,
		mjSENS_FORCE = 4,
		mjSENS_TORQUE = 5,
		mjSENS_MAGNETOMETER = 6,
		mjSENS_RANGEFINDER = 7,
		mjSENS_JOINTPOS = 8,
		mjSENS_JOINTVEL = 9,
		mjSENS_TENDONPOS = 10,
		mjSENS_TENDONVEL = 11,
		mjSENS_ACTUATORPOS = 12,
		mjSENS_ACTUATORVEL = 13,
		mjSENS_ACTUATORFRC = 14,
		mjSENS_BALLQUAT = 15,
		mjSENS_BALLANGVEL = 16,
		mjSENS_JOINTLIMITPOS = 17,
		mjSENS_JOINTLIMITVEL = 18,
		mjSENS_JOINTLIMITFRC = 19,
		mjSENS_TENDONLIMITPOS = 20,
		mjSENS_TENDONLIMITVEL = 21,
		mjSENS_TENDONLIMITFRC = 22,
		mjSENS_FRAMEPOS = 23,
		mjSENS_FRAMEQUAT = 24,
		mjSENS_FRAMEXAXIS = 25,
		mjSENS_FRAMEYAXIS = 26,
		mjSENS_FRAMEZAXIS = 27,
		mjSENS_FRAMELINVEL = 28,
		mjSENS_FRAMEANGVEL = 29,
		mjSENS_FRAMELINACC = 30,
		mjSENS_FRAMEANGACC = 31,
		mjSENS_SUBTREECOM = 32,
		mjSENS_SUBTREELINVEL = 33,
		mjSENS_SUBTREEANGMOM = 34,
		mjSENS_USER = 35,
	}

	public enum _mjtStage
	{
		mjSTAGE_NONE = 0,
		mjSTAGE_POS = 1,
		mjSTAGE_VEL = 2,
		mjSTAGE_ACC = 3,
	}

	public enum _mjtDataType
	{
		mjDATATYPE_REAL = 0,
		mjDATATYPE_POSITIVE = 1,
		mjDATATYPE_AXIS = 2,
		mjDATATYPE_QUATERNION = 3,
	}

	public enum _mjtLRMode
	{
		mjLRMODE_NONE = 0,
		mjLRMODE_MUSCLE = 1,
		mjLRMODE_MUSCLEUSER = 2,
		mjLRMODE_ALL = 3,
	}

	public enum _mjtWarning
	{
		mjWARN_INERTIA = 0,
		mjWARN_CONTACTFULL = 1,
		mjWARN_CNSTRFULL = 2,
		mjWARN_VGEOMFULL = 3,
		mjWARN_BADQPOS = 4,
		mjWARN_BADQVEL = 5,
		mjWARN_BADQACC = 6,
		mjWARN_BADCTRL = 7,
		mjNWARNING = 8,
	}

	public enum _mjtTimer
	{
		mjTIMER_STEP = 0,
		mjTIMER_FORWARD = 1,
		mjTIMER_INVERSE = 2,
		mjTIMER_POSITION = 3,
		mjTIMER_VELOCITY = 4,
		mjTIMER_ACTUATION = 5,
		mjTIMER_ACCELERATION = 6,
		mjTIMER_CONSTRAINT = 7,
		mjTIMER_POS_KINEMATICS = 8,
		mjTIMER_POS_INERTIA = 9,
		mjTIMER_POS_COLLISION = 10,
		mjTIMER_POS_MAKE = 11,
		mjTIMER_POS_PROJECT = 12,
		mjNTIMER = 13,
	}

	public enum _mjtCatBit
	{
		mjCAT_STATIC = 1,
		mjCAT_DYNAMIC = 2,
		mjCAT_DECOR = 4,
		mjCAT_ALL = 7,
	}

	public enum _mjtMouse
	{
		mjMOUSE_NONE = 0,
		mjMOUSE_ROTATE_V = 1,
		mjMOUSE_ROTATE_H = 2,
		mjMOUSE_MOVE_V = 3,
		mjMOUSE_MOVE_H = 4,
		mjMOUSE_ZOOM = 5,
		mjMOUSE_SELECT = 6,
	}

	public enum _mjtPertBit
	{
		mjPERT_TRANSLATE = 1,
		mjPERT_ROTATE = 2,
	}

	public enum _mjtCamera
	{
		mjCAMERA_FREE = 0,
		mjCAMERA_TRACKING = 1,
		mjCAMERA_FIXED = 2,
		mjCAMERA_USER = 3,
	}

	public enum _mjtLabel
	{
		mjLABEL_NONE = 0,
		mjLABEL_BODY = 1,
		mjLABEL_JOINT = 2,
		mjLABEL_GEOM = 3,
		mjLABEL_SITE = 4,
		mjLABEL_CAMERA = 5,
		mjLABEL_LIGHT = 6,
		mjLABEL_TENDON = 7,
		mjLABEL_ACTUATOR = 8,
		mjLABEL_CONSTRAINT = 9,
		mjLABEL_SKIN = 10,
		mjLABEL_SELECTION = 11,
		mjLABEL_SELPNT = 12,
		mjLABEL_CONTACTFORCE = 13,
		mjNLABEL = 14,
	}

	public enum _mjtFrame
	{
		mjFRAME_NONE = 0,
		mjFRAME_BODY = 1,
		mjFRAME_GEOM = 2,
		mjFRAME_SITE = 3,
		mjFRAME_CAMERA = 4,
		mjFRAME_LIGHT = 5,
		mjFRAME_WORLD = 6,
		mjNFRAME = 7,
	}

	public enum _mjtVisFlag
	{
		mjVIS_CONVEXHULL = 0,
		mjVIS_TEXTURE = 1,
		mjVIS_JOINT = 2,
		mjVIS_ACTUATOR = 3,
		mjVIS_CAMERA = 4,
		mjVIS_LIGHT = 5,
		mjVIS_TENDON = 6,
		mjVIS_RANGEFINDER = 7,
		mjVIS_CONSTRAINT = 8,
		mjVIS_INERTIA = 9,
		mjVIS_SCLINERTIA = 10,
		mjVIS_PERTFORCE = 11,
		mjVIS_PERTOBJ = 12,
		mjVIS_CONTACTPOINT = 13,
		mjVIS_CONTACTFORCE = 14,
		mjVIS_CONTACTSPLIT = 15,
		mjVIS_TRANSPARENT = 16,
		mjVIS_AUTOCONNECT = 17,
		mjVIS_COM = 18,
		mjVIS_SELECT = 19,
		mjVIS_STATIC = 20,
		mjVIS_SKIN = 21,
		mjNVISFLAG = 22,
	}

	public enum _mjtRndFlag
	{
		mjRND_SHADOW = 0,
		mjRND_WIREFRAME = 1,
		mjRND_REFLECTION = 2,
		mjRND_ADDITIVE = 3,
		mjRND_SKYBOX = 4,
		mjRND_FOG = 5,
		mjRND_HAZE = 6,
		mjRND_SEGMENT = 7,
		mjRND_IDCOLOR = 8,
		mjNRNDFLAG = 9,
	}

	public enum _mjtStereo
	{
		mjSTEREO_NONE = 0,
		mjSTEREO_QUADBUFFERED = 1,
		mjSTEREO_SIDEBYSIDE = 2,
	}

	public enum _mjtGridPos
	{
		mjGRID_TOPLEFT = 0,
		mjGRID_TOPRIGHT = 1,
		mjGRID_BOTTOMLEFT = 2,
		mjGRID_BOTTOMRIGHT = 3,
	}

	public enum _mjtFramebuffer
	{
		mjFB_WINDOW = 0,
		mjFB_OFFSCREEN = 1,
	}

	public enum _mjtFontScale
	{
		mjFONTSCALE_50 = 50,
		mjFONTSCALE_100 = 100,
		mjFONTSCALE_150 = 150,
		mjFONTSCALE_200 = 200,
		mjFONTSCALE_250 = 250,
		mjFONTSCALE_300 = 300,
	}

	public enum _mjtFont
	{
		mjFONT_NORMAL = 0,
		mjFONT_SHADOW = 1,
		mjFONT_BIG = 2,
	}

	public enum _mjtButton
	{
		mjBUTTON_NONE = 0,
		mjBUTTON_LEFT = 1,
		mjBUTTON_RIGHT = 2,
		mjBUTTON_MIDDLE = 3,
	}

	public enum _mjtEvent
	{
		mjEVENT_NONE = 0,
		mjEVENT_MOVE = 1,
		mjEVENT_PRESS = 2,
		mjEVENT_RELEASE = 3,
		mjEVENT_SCROLL = 4,
		mjEVENT_KEY = 5,
		mjEVENT_RESIZE = 6,
	}

	public enum _mjtItem
	{
		mjITEM_END = -2,
		mjITEM_SECTION = -1,
		mjITEM_SEPARATOR = 0,
		mjITEM_STATIC = 1,
		mjITEM_BUTTON = 2,
		mjITEM_CHECKINT = 3,
		mjITEM_CHECKBYTE = 4,
		mjITEM_RADIO = 5,
		mjITEM_RADIOLINE = 6,
		mjITEM_SELECT = 7,
		mjITEM_SLIDERINT = 8,
		mjITEM_SLIDERNUM = 9,
		mjITEM_EDITINT = 10,
		mjITEM_EDITNUM = 11,
		mjITEM_EDITTXT = 12,
		mjNITEM = 13,
	}

}
