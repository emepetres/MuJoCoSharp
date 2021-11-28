using System;
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

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjLROpt
	{
		public int mode;
		public int useexisting;
		public int uselimit;
		public mjtNum accel;
		public mjtNum maxforce;
		public mjtNum timeconst;
		public mjtNum timestep;
		public mjtNum inttotal;
		public mjtNum inteval;
		public mjtNum tolrange;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjVFS
	{
		public int nfile;
		public char filename;
		public int filesize;
		public void* filedata;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjOption
	{
		public mjtNum timestep;
		public mjtNum apirate;
		public mjtNum impratio;
		public mjtNum tolerance;
		public mjtNum noslip_tolerance;
		public mjtNum mpr_tolerance;
		public mjtNum gravity;
		public mjtNum wind;
		public mjtNum magnetic;
		public mjtNum density;
		public mjtNum viscosity;
		public mjtNum o_margin;
		public mjtNum o_solref;
		public mjtNum o_solimp;
		public int integrator;
		public int collision;
		public int cone;
		public int jacobian;
		public int solver;
		public int iterations;
		public int noslip_iterations;
		public int mpr_iterations;
		public int disableflags;
		public int enableflags;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjVisual
	{
		[StructLayout(LayoutKind.Sequential)]
		public unsafe struct _mjVisual_global
		{
			public float fovy;
			public float ipd;
			public float linewidth;
			public float glow;
			public int offwidth;
			public int offheight;
		}
		public _mjVisual_global global;
		[StructLayout(LayoutKind.Sequential)]
		public unsafe struct _mjVisual_quality
		{
			public int shadowsize;
			public int offsamples;
			public int numslices;
			public int numstacks;
			public int numquads;
		}
		public _mjVisual_quality quality;
		[StructLayout(LayoutKind.Sequential)]
		public unsafe struct _mjVisual_headlight
		{
			public float ambient;
			public float diffuse;
			public float specular;
			public int active;
		}
		public _mjVisual_headlight headlight;
		[StructLayout(LayoutKind.Sequential)]
		public unsafe struct _mjVisual_map
		{
			public float stiffness;
			public float stiffnessrot;
			public float force;
			public float torque;
			public float alpha;
			public float fogstart;
			public float fogend;
			public float znear;
			public float zfar;
			public float haze;
			public float shadowclip;
			public float shadowscale;
			public float actuatortendon;
		}
		public _mjVisual_map map;
		[StructLayout(LayoutKind.Sequential)]
		public unsafe struct _mjVisual_scale
		{
			public float forcewidth;
			public float contactwidth;
			public float contactheight;
			public float connect;
			public float com;
			public float camera;
			public float light;
			public float selectpoint;
			public float jointlength;
			public float jointwidth;
			public float actuatorlength;
			public float actuatorwidth;
			public float framelength;
			public float framewidth;
			public float constraint;
			public float slidercrank;
		}
		public _mjVisual_scale scale;
		[StructLayout(LayoutKind.Sequential)]
		public unsafe struct _mjVisual_rgba
		{
			public float fog;
			public float haze;
			public float force;
			public float inertia;
			public float joint;
			public float actuator;
			public float actuatornegative;
			public float actuatorpositive;
			public float com;
			public float camera;
			public float light;
			public float selectpoint;
			public float connect;
			public float contactpoint;
			public float contactforce;
			public float contactfriction;
			public float contacttorque;
			public float contactgap;
			public float rangefinder;
			public float constraint;
			public float slidercrank;
			public float crankbroken;
		}
		public _mjVisual_rgba rgba;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjStatistic
	{
		public mjtNum meaninertia;
		public mjtNum meanmass;
		public mjtNum meansize;
		public mjtNum extent;
		public mjtNum center;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjModel
	{
		public int nq;
		public int nv;
		public int nu;
		public int na;
		public int nbody;
		public int njnt;
		public int ngeom;
		public int nsite;
		public int ncam;
		public int nlight;
		public int nmesh;
		public int nmeshvert;
		public int nmeshtexvert;
		public int nmeshface;
		public int nmeshgraph;
		public int nskin;
		public int nskinvert;
		public int nskintexvert;
		public int nskinface;
		public int nskinbone;
		public int nskinbonevert;
		public int nhfield;
		public int nhfielddata;
		public int ntex;
		public int ntexdata;
		public int nmat;
		public int npair;
		public int nexclude;
		public int neq;
		public int ntendon;
		public int nwrap;
		public int nsensor;
		public int nnumeric;
		public int nnumericdata;
		public int ntext;
		public int ntextdata;
		public int ntuple;
		public int ntupledata;
		public int nkey;
		public int nmocap;
		public int nuser_body;
		public int nuser_jnt;
		public int nuser_geom;
		public int nuser_site;
		public int nuser_cam;
		public int nuser_tendon;
		public int nuser_actuator;
		public int nuser_sensor;
		public int nnames;
		public int nM;
		public int nemax;
		public int njmax;
		public int nconmax;
		public int nstack;
		public int nuserdata;
		public int nsensordata;
		public int nbuffer;
		public mjOption opt;
		public mjVisual vis;
		public mjStatistic stat;
		public void* buffer;
		public mjtNum* qpos0;
		public mjtNum* qpos_spring;
		public int* body_parentid;
		public int* body_rootid;
		public int* body_weldid;
		public int* body_mocapid;
		public int* body_jntnum;
		public int* body_jntadr;
		public int* body_dofnum;
		public int* body_dofadr;
		public int* body_geomnum;
		public int* body_geomadr;
		public mjtByte* body_simple;
		public mjtByte* body_sameframe;
		public mjtNum* body_pos;
		public mjtNum* body_quat;
		public mjtNum* body_ipos;
		public mjtNum* body_iquat;
		public mjtNum* body_mass;
		public mjtNum* body_subtreemass;
		public mjtNum* body_inertia;
		public mjtNum* body_invweight0;
		public mjtNum* body_user;
		public int* jnt_type;
		public int* jnt_qposadr;
		public int* jnt_dofadr;
		public int* jnt_bodyid;
		public int* jnt_group;
		public mjtByte* jnt_limited;
		public mjtNum* jnt_solref;
		public mjtNum* jnt_solimp;
		public mjtNum* jnt_pos;
		public mjtNum* jnt_axis;
		public mjtNum* jnt_stiffness;
		public mjtNum* jnt_range;
		public mjtNum* jnt_margin;
		public mjtNum* jnt_user;
		public int* dof_bodyid;
		public int* dof_jntid;
		public int* dof_parentid;
		public int* dof_Madr;
		public int* dof_simplenum;
		public mjtNum* dof_solref;
		public mjtNum* dof_solimp;
		public mjtNum* dof_frictionloss;
		public mjtNum* dof_armature;
		public mjtNum* dof_damping;
		public mjtNum* dof_invweight0;
		public mjtNum* dof_M0;
		public int* geom_type;
		public int* geom_contype;
		public int* geom_conaffinity;
		public int* geom_condim;
		public int* geom_bodyid;
		public int* geom_dataid;
		public int* geom_matid;
		public int* geom_group;
		public int* geom_priority;
		public mjtByte* geom_sameframe;
		public mjtNum* geom_solmix;
		public mjtNum* geom_solref;
		public mjtNum* geom_solimp;
		public mjtNum* geom_size;
		public mjtNum* geom_rbound;
		public mjtNum* geom_pos;
		public mjtNum* geom_quat;
		public mjtNum* geom_friction;
		public mjtNum* geom_margin;
		public mjtNum* geom_gap;
		public mjtNum* geom_user;
		public float* geom_rgba;
		public int* site_type;
		public int* site_bodyid;
		public int* site_matid;
		public int* site_group;
		public mjtByte* site_sameframe;
		public mjtNum* site_size;
		public mjtNum* site_pos;
		public mjtNum* site_quat;
		public mjtNum* site_user;
		public float* site_rgba;
		public int* cam_mode;
		public int* cam_bodyid;
		public int* cam_targetbodyid;
		public mjtNum* cam_pos;
		public mjtNum* cam_quat;
		public mjtNum* cam_poscom0;
		public mjtNum* cam_pos0;
		public mjtNum* cam_mat0;
		public mjtNum* cam_fovy;
		public mjtNum* cam_ipd;
		public mjtNum* cam_user;
		public int* light_mode;
		public int* light_bodyid;
		public int* light_targetbodyid;
		public mjtByte* light_directional;
		public mjtByte* light_castshadow;
		public mjtByte* light_active;
		public mjtNum* light_pos;
		public mjtNum* light_dir;
		public mjtNum* light_poscom0;
		public mjtNum* light_pos0;
		public mjtNum* light_dir0;
		public float* light_attenuation;
		public float* light_cutoff;
		public float* light_exponent;
		public float* light_ambient;
		public float* light_diffuse;
		public float* light_specular;
		public int* mesh_vertadr;
		public int* mesh_vertnum;
		public int* mesh_texcoordadr;
		public int* mesh_faceadr;
		public int* mesh_facenum;
		public int* mesh_graphadr;
		public float* mesh_vert;
		public float* mesh_normal;
		public float* mesh_texcoord;
		public int* mesh_face;
		public int* mesh_graph;
		public int* skin_matid;
		public float* skin_rgba;
		public float* skin_inflate;
		public int* skin_vertadr;
		public int* skin_vertnum;
		public int* skin_texcoordadr;
		public int* skin_faceadr;
		public int* skin_facenum;
		public int* skin_boneadr;
		public int* skin_bonenum;
		public float* skin_vert;
		public float* skin_texcoord;
		public int* skin_face;
		public int* skin_bonevertadr;
		public int* skin_bonevertnum;
		public float* skin_bonebindpos;
		public float* skin_bonebindquat;
		public int* skin_bonebodyid;
		public int* skin_bonevertid;
		public float* skin_bonevertweight;
		public mjtNum* hfield_size;
		public int* hfield_nrow;
		public int* hfield_ncol;
		public int* hfield_adr;
		public float* hfield_data;
		public int* tex_type;
		public int* tex_height;
		public int* tex_width;
		public int* tex_adr;
		public mjtByte* tex_rgb;
		public int* mat_texid;
		public mjtByte* mat_texuniform;
		public float* mat_texrepeat;
		public float* mat_emission;
		public float* mat_specular;
		public float* mat_shininess;
		public float* mat_reflectance;
		public float* mat_rgba;
		public int* pair_dim;
		public int* pair_geom1;
		public int* pair_geom2;
		public int* pair_signature;
		public mjtNum* pair_solref;
		public mjtNum* pair_solimp;
		public mjtNum* pair_margin;
		public mjtNum* pair_gap;
		public mjtNum* pair_friction;
		public int* exclude_signature;
		public int* eq_type;
		public int* eq_obj1id;
		public int* eq_obj2id;
		public mjtByte* eq_active;
		public mjtNum* eq_solref;
		public mjtNum* eq_solimp;
		public mjtNum* eq_data;
		public int* tendon_adr;
		public int* tendon_num;
		public int* tendon_matid;
		public int* tendon_group;
		public mjtByte* tendon_limited;
		public mjtNum* tendon_width;
		public mjtNum* tendon_solref_lim;
		public mjtNum* tendon_solimp_lim;
		public mjtNum* tendon_solref_fri;
		public mjtNum* tendon_solimp_fri;
		public mjtNum* tendon_range;
		public mjtNum* tendon_margin;
		public mjtNum* tendon_stiffness;
		public mjtNum* tendon_damping;
		public mjtNum* tendon_frictionloss;
		public mjtNum* tendon_lengthspring;
		public mjtNum* tendon_length0;
		public mjtNum* tendon_invweight0;
		public mjtNum* tendon_user;
		public float* tendon_rgba;
		public int* wrap_type;
		public int* wrap_objid;
		public mjtNum* wrap_prm;
		public int* actuator_trntype;
		public int* actuator_dyntype;
		public int* actuator_gaintype;
		public int* actuator_biastype;
		public int* actuator_trnid;
		public int* actuator_group;
		public mjtByte* actuator_ctrllimited;
		public mjtByte* actuator_forcelimited;
		public mjtNum* actuator_dynprm;
		public mjtNum* actuator_gainprm;
		public mjtNum* actuator_biasprm;
		public mjtNum* actuator_ctrlrange;
		public mjtNum* actuator_forcerange;
		public mjtNum* actuator_gear;
		public mjtNum* actuator_cranklength;
		public mjtNum* actuator_acc0;
		public mjtNum* actuator_length0;
		public mjtNum* actuator_lengthrange;
		public mjtNum* actuator_user;
		public int* sensor_type;
		public int* sensor_datatype;
		public int* sensor_needstage;
		public int* sensor_objtype;
		public int* sensor_objid;
		public int* sensor_dim;
		public int* sensor_adr;
		public mjtNum* sensor_cutoff;
		public mjtNum* sensor_noise;
		public mjtNum* sensor_user;
		public int* numeric_adr;
		public int* numeric_size;
		public mjtNum* numeric_data;
		public int* text_adr;
		public int* text_size;
		public char* text_data;
		public int* tuple_adr;
		public int* tuple_size;
		public int* tuple_objtype;
		public int* tuple_objid;
		public mjtNum* tuple_objprm;
		public mjtNum* key_time;
		public mjtNum* key_qpos;
		public mjtNum* key_qvel;
		public mjtNum* key_act;
		public mjtNum* key_mpos;
		public mjtNum* key_mquat;
		public int* name_bodyadr;
		public int* name_jntadr;
		public int* name_geomadr;
		public int* name_siteadr;
		public int* name_camadr;
		public int* name_lightadr;
		public int* name_meshadr;
		public int* name_skinadr;
		public int* name_hfieldadr;
		public int* name_texadr;
		public int* name_matadr;
		public int* name_pairadr;
		public int* name_excludeadr;
		public int* name_eqadr;
		public int* name_tendonadr;
		public int* name_actuatoradr;
		public int* name_sensoradr;
		public int* name_numericadr;
		public int* name_textadr;
		public int* name_tupleadr;
		public int* name_keyadr;
		public char* names;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjContact
	{
		public mjtNum dist;
		public mjtNum pos;
		public mjtNum frame;
		public mjtNum includemargin;
		public mjtNum friction;
		public mjtNum solref;
		public mjtNum solimp;
		public mjtNum mu;
		public mjtNum H;
		public int dim;
		public int geom1;
		public int geom2;
		public int exclude;
		public int efc_address;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjWarningStat
	{
		public int lastinfo;
		public int number;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjTimerStat
	{
		public mjtNum duration;
		public int number;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjSolverStat
	{
		public mjtNum improvement;
		public mjtNum gradient;
		public mjtNum lineslope;
		public int nactive;
		public int nchange;
		public int neval;
		public int nupdate;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjData
	{
		public int nstack;
		public int nbuffer;
		public int pstack;
		public int maxuse_stack;
		public int maxuse_con;
		public int maxuse_efc;
		public mjWarningStat warning;
		public mjTimerStat timer;
		public mjSolverStat solver;
		public int solver_iter;
		public int solver_nnz;
		public mjtNum solver_fwdinv;
		public int ne;
		public int nf;
		public int nefc;
		public int ncon;
		public mjtNum time;
		public mjtNum energy;
		public void* buffer;
		public mjtNum* stack;
		public mjtNum* qpos;
		public mjtNum* qvel;
		public mjtNum* act;
		public mjtNum* qacc_warmstart;
		public mjtNum* ctrl;
		public mjtNum* qfrc_applied;
		public mjtNum* xfrc_applied;
		public mjtNum* qacc;
		public mjtNum* act_dot;
		public mjtNum* mocap_pos;
		public mjtNum* mocap_quat;
		public mjtNum* userdata;
		public mjtNum* sensordata;
		public mjtNum* xpos;
		public mjtNum* xquat;
		public mjtNum* xmat;
		public mjtNum* xipos;
		public mjtNum* ximat;
		public mjtNum* xanchor;
		public mjtNum* xaxis;
		public mjtNum* geom_xpos;
		public mjtNum* geom_xmat;
		public mjtNum* site_xpos;
		public mjtNum* site_xmat;
		public mjtNum* cam_xpos;
		public mjtNum* cam_xmat;
		public mjtNum* light_xpos;
		public mjtNum* light_xdir;
		public mjtNum* subtree_com;
		public mjtNum* cdof;
		public mjtNum* cinert;
		public int* ten_wrapadr;
		public int* ten_wrapnum;
		public int* ten_J_rownnz;
		public int* ten_J_rowadr;
		public int* ten_J_colind;
		public mjtNum* ten_length;
		public mjtNum* ten_J;
		public int* wrap_obj;
		public mjtNum* wrap_xpos;
		public mjtNum* actuator_length;
		public mjtNum* actuator_moment;
		public mjtNum* crb;
		public mjtNum* qM;
		public mjtNum* qLD;
		public mjtNum* qLDiagInv;
		public mjtNum* qLDiagSqrtInv;
		public mjContact* contact;
		public int* efc_type;
		public int* efc_id;
		public int* efc_J_rownnz;
		public int* efc_J_rowadr;
		public int* efc_J_rowsuper;
		public int* efc_J_colind;
		public int* efc_JT_rownnz;
		public int* efc_JT_rowadr;
		public int* efc_JT_rowsuper;
		public int* efc_JT_colind;
		public mjtNum* efc_J;
		public mjtNum* efc_JT;
		public mjtNum* efc_pos;
		public mjtNum* efc_margin;
		public mjtNum* efc_frictionloss;
		public mjtNum* efc_diagApprox;
		public mjtNum* efc_KBIP;
		public mjtNum* efc_D;
		public mjtNum* efc_R;
		public int* efc_AR_rownnz;
		public int* efc_AR_rowadr;
		public int* efc_AR_colind;
		public mjtNum* efc_AR;
		public mjtNum* ten_velocity;
		public mjtNum* actuator_velocity;
		public mjtNum* cvel;
		public mjtNum* cdof_dot;
		public mjtNum* qfrc_bias;
		public mjtNum* qfrc_passive;
		public mjtNum* efc_vel;
		public mjtNum* efc_aref;
		public mjtNum* subtree_linvel;
		public mjtNum* subtree_angmom;
		public mjtNum* actuator_force;
		public mjtNum* qfrc_actuator;
		public mjtNum* qfrc_unc;
		public mjtNum* qacc_unc;
		public mjtNum* efc_b;
		public mjtNum* efc_force;
		public int* efc_state;
		public mjtNum* qfrc_constraint;
		public mjtNum* qfrc_inverse;
		public mjtNum* cacc;
		public mjtNum* cfrc_int;
		public mjtNum* cfrc_ext;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjvPerturb
	{
		public int select;
		public int skinselect;
		public int active;
		public int active2;
		public mjtNum refpos;
		public mjtNum refquat;
		public mjtNum localpos;
		public mjtNum scale;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjvCamera
	{
		public int type;
		public int fixedcamid;
		public int trackbodyid;
		public mjtNum lookat;
		public mjtNum distance;
		public mjtNum azimuth;
		public mjtNum elevation;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjvGLCamera
	{
		public float pos;
		public float forward;
		public float up;
		public float frustum_center;
		public float frustum_bottom;
		public float frustum_top;
		public float frustum_near;
		public float frustum_far;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjvGeom
	{
		public int type;
		public int dataid;
		public int objtype;
		public int objid;
		public int category;
		public int texid;
		public int texuniform;
		public int texcoord;
		public int segid;
		public float texrepeat;
		public float size;
		public float pos;
		public float mat;
		public float rgba;
		public float emission;
		public float specular;
		public float shininess;
		public float reflectance;
		public char label;
		public float camdist;
		public float modelrbound;
		public mjtByte transparent;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjvLight
	{
		public float pos;
		public float dir;
		public float attenuation;
		public float cutoff;
		public float exponent;
		public float ambient;
		public float diffuse;
		public float specular;
		public mjtByte headlight;
		public mjtByte directional;
		public mjtByte castshadow;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjvOption
	{
		public int label;
		public int frame;
		public mjtByte geomgroup;
		public mjtByte sitegroup;
		public mjtByte jointgroup;
		public mjtByte tendongroup;
		public mjtByte actuatorgroup;
		public mjtByte flags;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjvScene
	{
		public int maxgeom;
		public int ngeom;
		public mjvGeom* geoms;
		public int* geomorder;
		public int nskin;
		public int* skinfacenum;
		public int* skinvertadr;
		public int* skinvertnum;
		public float* skinvert;
		public float* skinnormal;
		public int nlight;
		public mjvLight lights;
		public mjvGLCamera camera;
		public mjtByte enabletransform;
		public float translate;
		public float rotate;
		public float scale;
		public int stereo;
		public mjtByte flags;
		public int framewidth;
		public float framergb;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjvFigure
	{
		public int flg_legend;
		public int flg_ticklabel;
		public int flg_extend;
		public int flg_barplot;
		public int flg_selection;
		public int flg_symmetric;
		public float linewidth;
		public float gridwidth;
		public int gridsize;
		public float gridrgb;
		public float figurergba;
		public float panergba;
		public float legendrgba;
		public float textrgb;
		public float linergb;
		public float range;
		public char xformat;
		public char yformat;
		public char minwidth;
		public char title;
		public char xlabel;
		public char linename;
		public int legendoffset;
		public int subplot;
		public int highlight;
		public int highlightid;
		public float selection;
		public int linepnt;
		public float linedata;
		public int xaxispixel;
		public int yaxispixel;
		public float xaxisdata;
		public float yaxisdata;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjrRect
	{
		public int left;
		public int bottom;
		public int width;
		public int height;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjrContext
	{
		public float lineWidth;
		public float shadowClip;
		public float shadowScale;
		public float fogStart;
		public float fogEnd;
		public float fogRGBA;
		public int shadowSize;
		public int offWidth;
		public int offHeight;
		public int offSamples;
		public int fontScale;
		public int auxWidth;
		public int auxHeight;
		public int auxSamples;
		public uint offFBO;
		public uint offFBO_r;
		public uint offColor;
		public uint offColor_r;
		public uint offDepthStencil;
		public uint offDepthStencil_r;
		public uint shadowFBO;
		public uint shadowTex;
		public uint auxFBO;
		public uint auxFBO_r;
		public uint auxColor;
		public uint auxColor_r;
		public int ntexture;
		public int textureType;
		public uint texture;
		public uint basePlane;
		public uint baseMesh;
		public uint baseHField;
		public uint baseBuiltin;
		public uint baseFontNormal;
		public uint baseFontShadow;
		public uint baseFontBig;
		public int rangePlane;
		public int rangeMesh;
		public int rangeHField;
		public int rangeBuiltin;
		public int rangeFont;
		public int nskin;
		public uint* skinvertVBO;
		public uint* skinnormalVBO;
		public uint* skintexcoordVBO;
		public uint* skinfaceVBO;
		public int charWidth;
		public int charWidthBig;
		public int charHeight;
		public int charHeightBig;
		public int glewInitialized;
		public int windowAvailable;
		public int windowSamples;
		public int windowStereo;
		public int windowDoublebuffer;
		public int currentBuffer;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiState
	{
		public int nrect;
		public mjrRect rect;
		public void* userdata;
		public int type;
		public int left;
		public int right;
		public int middle;
		public int doubleclick;
		public int button;
		public double buttontime;
		public double x;
		public double y;
		public double dx;
		public double dy;
		public double sx;
		public double sy;
		public int control;
		public int shift;
		public int alt;
		public int key;
		public double keytime;
		public int mouserect;
		public int dragrect;
		public int dragbutton;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiThemeSpacing
	{
		public int total;
		public int scroll;
		public int label;
		public int section;
		public int itemside;
		public int itemmid;
		public int itemver;
		public int texthor;
		public int textver;
		public int linescroll;
		public int samples;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiThemeColor
	{
		public float master;
		public float thumb;
		public float secttitle;
		public float sectfont;
		public float sectsymbol;
		public float sectpane;
		public float shortcut;
		public float fontactive;
		public float fontinactive;
		public float decorinactive;
		public float decorinactive2;
		public float button;
		public float check;
		public float radio;
		public float select;
		public float select2;
		public float slider;
		public float slider2;
		public float edit;
		public float edit2;
		public float cursor;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiItemSingle
	{
		public int modifier;
		public int shortcut;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiItemMulti
	{
		public int nelem;
		public char name;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiItemSlider
	{
		public double range;
		public double divisions;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiItemEdit
	{
		public int nelem;
		public double range;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiItem
	{
		public int type;
		public char name;
		public int state;
		public void* pdata;
		public int sectionid;
		public int itemid;
		public mjrRect rect;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiSection
	{
		public char name;
		public int state;
		public int modifier;
		public int shortcut;
		public int nitem;
		public mjuiItem item;
		public mjrRect rtitle;
		public mjrRect rcontent;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjUI
	{
		public mjuiThemeSpacing spacing;
		public mjuiThemeColor color;
		public mjfItemEnable predicate;
		public void* userdata;
		public int rectid;
		public int auxid;
		public int radiocol;
		public int width;
		public int height;
		public int maxheight;
		public int scroll;
		public int mousesect;
		public int mouseitem;
		public int mousehelp;
		public int editsect;
		public int edititem;
		public int editcursor;
		public int editscroll;
		public char edittext;
		public mjuiItem* editchanged;
		public int nsect;
		public mjuiSection sect;
	}

	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct _mjuiDef
	{
		public int type;
		public char name;
		public int state;
		public void* pdata;
		public char other;
	}

}

