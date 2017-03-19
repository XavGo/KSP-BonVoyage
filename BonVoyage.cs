﻿using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.UI.Screens;
using KSP.IO;

namespace BonVoyage
{
	/*
	 * Addon start at KSC screen
	 * Loads rovers with BonVoyage module onboard
	 * Keeps all rovers in memory until scene switch
	 * Every scene switch rebuilds rover list
	*/
	[KSPAddon(KSPAddon.Startup.SpaceCentre, true)]
	public class BonVoyage : MonoBehaviour
	{
		public static BonVoyage Instance;

		private List<ActiveRover> activeRovers;
		private ApplicationLauncherButton appLauncherButton;
		private IButton toolbarButton;
		private DateTime lastUpdated;
		private bool gamePaused;
		private BonVoyageModule currentModule;
		private List<Vector3d> wayPoints;

		// Config stuff
		private PluginConfiguration config;
		private bool autoDewarp;
		public bool AutoDewarp { get { return autoDewarp; } }
		private bool useKSPSkin;

		// Input locking stuff
		private ControlTypes lockMask = (
			ControlTypes.YAW |
			ControlTypes.PITCH |
			ControlTypes.ROLL |
			ControlTypes.THROTTLE |
			ControlTypes.STAGING |
			ControlTypes.CUSTOM_ACTION_GROUPS |
			ControlTypes.GROUPS_ALL |
			ControlTypes.RCS |
			ControlTypes.SAS |
			ControlTypes.WHEEL_STEER |
			ControlTypes.WHEEL_THROTTLE
		);
		private Rect labelRect;
		private GUIStyle labelStyle;

		//GUI variables
		private bool useToolbar;
		private bool guiVisible;
		private bool rcVisible;
		private bool globalHidden;
		private Rect guiRect;
		private int guiId;
		Vector2 mainWindowScrollPosition;
		private Rect rcRect;
		private int rcGuiId;

		/// <summary>
		/// Instead of constructor.
		/// </summary>
		public void Awake()
		{
			if (Instance != null)
			{
				Destroy(this);
				return;
			}
			Instance = this;

			toolbarButton = null;
			guiVisible = false;
			rcVisible = false;
			globalHidden = false;
			gamePaused = false;

			guiId = GUIUtility.GetControlID(FocusType.Passive);
			rcGuiId = GUIUtility.GetControlID (FocusType.Passive);
			config = PluginConfiguration.CreateForType<BonVoyage>();
			config.load();
			autoDewarp = config.GetValue<bool>("autoDewarp", false);

			activeRovers = new List<ActiveRover>();
			wayPoints = new List<Vector3d> ();

			Rect sample = new Rect();
			sample.width = 700;
			sample.height = 500;
			sample.center = new Vector2(Screen.width / 2, Screen.height / 2);
			guiRect = config.GetValue<Rect>("guiRect", new Rect(sample));
			sample.width = 400;
			sample.height = 500;
			sample.center = new Vector2(Screen.width / 2, Screen.height / 2);
			rcRect = config.GetValue<Rect> ("rcRect", new Rect(sample));
			useKSPSkin = config.GetValue<bool> ("useKSPSkin", false);
			useToolbar = config.GetValue<bool> ("useToolbar", false);
			config.save();
			lastUpdated = DateTime.Now;
			mainWindowScrollPosition = new Vector2(0, 0);

			labelRect = new Rect(0, 0, Screen.width, Screen.height / 2);
			labelStyle = new GUIStyle();
			labelStyle.stretchWidth = true;
			labelStyle.stretchHeight = true;
			labelStyle.alignment = TextAnchor.MiddleCenter;
			labelStyle.fontSize = Screen.height / 20;
			labelStyle.fontStyle = FontStyle.Bold;
			labelStyle.normal.textColor = Palette.red;

//			mapMarker = GameDatabase.Instance.GetTexture("BonVoyage/Textures/map-marker", false);
		}

		/// <summary>
		/// Initial instance start.
		/// </summary>
		public void Start()
		{
			DontDestroyOnLoad(this);
			GameEvents.onGUIApplicationLauncherReady.Add(onGUIApplicationLauncherReady);
			GameEvents.onGameSceneSwitchRequested.Add(onGameSceneSwitchRequested);
			GameEvents.onLevelWasLoaded.Add(onLevelWasLoaded);
			GameEvents.onVesselChange.Add(onVesselChange);
			GameEvents.onHideUI.Add(onHideUI);
			GameEvents.onShowUI.Add(onShowUI);
			GameEvents.onGamePause.Add (onGamePause);
			GameEvents.onGameUnpause.Add (onGameUnpause);
			LoadRovers();
		}

		/// <summary>
		/// Cleanup on destruction.
		/// </summary>
		public void OnDestroy()
		{
			GameEvents.onGUIApplicationLauncherReady.Remove(onGUIApplicationLauncherReady);
			GameEvents.onGameSceneSwitchRequested.Remove(onGameSceneSwitchRequested);
			GameEvents.onLevelWasLoaded.Remove(onLevelWasLoaded);
			GameEvents.onVesselChange.Remove(onVesselChange);
			GameEvents.onHideUI.Remove(onHideUI);
			GameEvents.onShowUI.Remove(onShowUI);
			GameEvents.onGamePause.Remove (onGamePause);
			GameEvents.onGameUnpause.Remove (onGameUnpause);

			DestroyLauncher ();

			config.SetValue("autoDewarp", autoDewarp);
			config.SetValue("guiRect", guiRect);
			config.SetValue ("rcRect", rcRect);
			config.SetValue ("useKSPSkin", useKSPSkin);
			config.SetValue ("useToolbar", useToolbar);
			config.save();
			InputLockManager.RemoveControlLock("BonVoyageInputLock");
		}

		public void onGamePause() {
			gamePaused = true;
		}

		public void onGameUnpause() {
			gamePaused = false;
		}

		/// <summary>
		/// Hide UI and remove input lock on scene switch
		/// </summary>
		/// <param name="ev">Ev.</param>
		public void onGameSceneSwitchRequested(GameEvents.FromToAction<GameScenes, GameScenes> ev)
		{
			if (appLauncherButton != null)
				appLauncherButton.SetFalse();
			guiVisible = false;
			rcVisible = false;
			InputLockManager.RemoveControlLock("BonVoyageInputLock");
		}

		/// <summary>
		/// Active vessel changed, deal with it.
		/// </summary>
		/// <param name="vessel">Vessel.</param>
		public void onVesselChange(Vessel vessel)
		{
			if (!HighLogic.LoadedSceneIsFlight)
				return;
			
			currentModule = vessel.FindPartModuleImplementing<BonVoyageModule> ();
//			if (wayPoints != null)
//				wayPoints.Clear ();
			if (currentModule != null) {
				currentModule.SystemCheck ();
				UpdateWayPoints ();
//				wayPoints = PathUtils.DecodePath (currentModule.pathEncoded, currentModule.vessel.mainBody);
				if (currentModule.isActive) {
					InputLockManager.SetControlLock (lockMask, "BonVoyageInputLock");
					return;
				}
			}

//			currentModule = null;
//			foreach (var rover in activeRovers)
//			{
//				if (rover.vessel == vessel)
////					ControlThis (rover.vessel.FindPartModuleImplementing<BonVoyageModule> (), false);
//					currentModule = rover.vessel.FindPartModuleImplementing<BonVoyageModule>();
//				
//				if (rover.vessel == vessel && rover.bvActive)
//				{
//					InputLockManager.SetControlLock(lockMask, "BonVoyageInputLock");
//					return;
//				}
//			}
			InputLockManager.RemoveControlLock("BonVoyageInputLock");
		}

		/// <summary>
		/// Hide UI.
		/// </summary>
		private void onHideUI()
		{
			globalHidden = true;
		}

		/// <summary>
		/// Show UI.
		/// </summary>
		private void onShowUI()
		{
			globalHidden = false;
		}

		/// <summary>
		/// Scene was loaded, update controlled rover list.
		/// </summary>
		/// <param name="scene">Scene.</param>
		public void onLevelWasLoaded(GameScenes scene)
		{
			gamePaused = false;
			LoadRovers();
			onVesselChange(FlightGlobals.ActiveVessel);
		}

		/// <summary>
		/// Update controlled rovers once a second.
		/// </summary>
		public void Update()
		{
			if (gamePaused)
				return;

			if (lastUpdated.AddMilliseconds(200) > DateTime.Now)
				return;

			lastUpdated = DateTime.Now;

			double currentTime = Planetarium.GetUniversalTime();
			
			for (int i = 0; i < activeRovers.Count; ++i)
			{
				ConfigNode vesselConfigNode = new ConfigNode();
				activeRovers[i].vessel.protoVessel.Save(vesselConfigNode);
				ConfigNode BVModule = new ConfigNode();
				foreach (ConfigNode part in vesselConfigNode.GetNodes("PART"))
				{
					BVModule = part.GetNode("MODULE", "name", "BonVoyageModule");
					if (BVModule != null)
						break;
				}

				activeRovers[i].Update(currentTime, BVModule, vesselConfigNode);
			}
		}

		/// <summary>
		/// Deal with launcher button.
		/// </summary>
		private void CreateLauncher() {
			if (ToolbarManager.ToolbarAvailable && useToolbar && !HighLogic.LoadedSceneIsEditor) {
				toolbarButton = ToolbarManager.Instance.add ("BonVoyage", "AppLaunch");
				toolbarButton.TexturePath = "BonVoyage/Textures/bon-voyage-icon-toolbar";
				toolbarButton.ToolTip = "Bon Voyage Controller";
				toolbarButton.Visible = true;
				toolbarButton.OnClick += (ClickEvent e) => {
					onToggle();
				};
			}
			else if (appLauncherButton == null)
			{
				appLauncherButton = ApplicationLauncher.Instance.AddModApplication(
					onAppTrue,
					onAppFalse,
					null,
					null,
					null,
					null,
					ApplicationLauncher.AppScenes.SPACECENTER |
					ApplicationLauncher.AppScenes.TRACKSTATION |
					ApplicationLauncher.AppScenes.FLIGHT |
					ApplicationLauncher.AppScenes.MAPVIEW,
					GameDatabase.Instance.GetTexture("BonVoyage/Textures/bon-voyage-icon", false)
				);
			}
		}

		/// <summary>
		/// Clear launcher button
		/// </summary>
		private void DestroyLauncher() {
			if (appLauncherButton != null) {
				ApplicationLauncher.Instance.RemoveModApplication (appLauncherButton);
				appLauncherButton = null;
			}

			if (toolbarButton != null) {
				toolbarButton.Destroy ();
				toolbarButton = null;
			}
		}

		/// <summary>
		/// Laucnher ready to be initialized
		/// </summary>
		public void onGUIApplicationLauncherReady()
		{
			CreateLauncher ();
		}

		/// <summary>
		/// Draw UI
		/// </summary>
		public void OnGUI()
		{
			if (gamePaused || globalHidden) return;

			if (MapView.MapIsEnabled && HighLogic.LoadedSceneIsFlight && wayPoints.Count > 0)
				GLUtils.DrawCurve (wayPoints);

			if (InputLockManager.GetControlLock("BonVoyageInputLock") != 0)
			{
				GUILayout.BeginArea(labelRect);
				GUILayout.Label("Bon Voyage control lock active", labelStyle);
				GUILayout.EndArea();
			}

			if (!guiVisible && !rcVisible) return;

			if (useKSPSkin)
				GUI.skin = HighLogic.Skin;
			else
				GUI.skin = UnityEngine.GUI.skin;

			if (guiVisible) {
				guiRect = Layout.Window (
					guiId,
					guiRect,
					DrawGUI,
					"Bon Voyage powered rovers"
				);
			}

			if (rcVisible) {
				rcRect = Layout.Window (
					rcGuiId,
					rcRect,
					DrawRcGUI,
					"Bon Voyage control"
				);
			}
		}

		public void onAppTrue()
		{
			guiVisible = true;
		}

		public void onAppFalse()
		{
			guiVisible = false;
		}

		public void onToggle() {
			guiVisible = !guiVisible;
		}


		/// <summary>
		/// Draws main UI
		/// </summary>
		/// <param name="guiId">GUI identifier.</param>
		public void DrawGUI(int guiId)
		{
			double currentTime = Planetarium.GetUniversalTime();
			GUILayout.BeginVertical();
			mainWindowScrollPosition = GUILayout.BeginScrollView(mainWindowScrollPosition);
			for (int i = 0; i < activeRovers.Count; i++) {
				var rover = activeRovers [i];
				switch (rover.status) {
					case "current":
						GUI.contentColor = Palette.white;
						break;
					case "roving":
						GUI.contentColor = Palette.green;
						break;
					case "idle":
						GUI.contentColor = Palette.yellow;
						break;
					case "awaiting sunlight":
						GUI.contentColor = Palette.red;
						break;
					case "fuel cells resource depleted":
						GUI.contentColor = Palette.red;
						break;
					case "not enough power":
						GUI.contentColor = Palette.red;
						break;
				}
				GUILayout.BeginHorizontal ();
				if (Layout.Button (rover.vessel.vesselName, GUILayout.Width(200))) {
					if (HighLogic.LoadedScene == GameScenes.TRACKSTATION) {
						PlanetariumCamera.fetch.SetTarget (rover.vessel.mapObject);
					}
					if (HighLogic.LoadedSceneIsFlight) {
						MapView.EnterMapView ();
						PlanetariumCamera.fetch.SetTarget (rover.vessel.mapObject);
					}
				}
				if (Layout.Button ("Switch to", GUILayout.Width(100))) {
					if (rover.vessel.loaded)
						FlightGlobals.SetActiveVessel (rover.vessel);
					else {
						GamePersistence.SaveGame ("persistent", HighLogic.SaveFolder, SaveMode.OVERWRITE);
						FlightDriver.StartAndFocusVessel ("persistent", FlightGlobals.Vessels.IndexOf (rover.vessel));
					}
				}

//				Layout.LabelAndText (rover.vessel.mainBody.theName, rover.status);
				Layout.Label (rover.vessel.mainBody.bodyName, GUILayout.Width(75));
				Layout.Label (rover.status, GUILayout.Width (125));

				if (rover.status == "roving" || rover.status == "awaiting sunlight" || rover.status == "fuel cells resource depleted" || rover.status == "not enough power") {
					Layout.Label (
						"v̅ = " + rover.AverageSpeed.ToString ("N") + ", yet to travel " +
						rover.yetToTravel.ToString ("N0") + " meters"
					);
				}

				if (rover.status == "idle") {
					TimeSpan t = TimeSpan.FromSeconds (currentTime - rover.LastTime);

					string idlePeriod = string.Format (
						                    "{0:D2}h:{1:D2}m:{2:D2}s",
						                    t.Hours,
						                    t.Minutes,
						                    t.Seconds
					                    );

					Layout.Label (idlePeriod);
				}
				GUILayout.EndHorizontal ();
			}
			GUILayout.EndScrollView();
			GUI.contentColor = Palette.white;
			autoDewarp = Layout.Toggle(autoDewarp, "Automagic Dewarp");
//			useKSPSkin = Layout.Toggle (useKSPSkin, "Use KSP Skin");
			GUILayout.BeginHorizontal ();
			if (Layout.Button ("Close", GUILayout.Height(25))) {
				if (appLauncherButton != null)
					appLauncherButton.SetFalse ();
				else
					onToggle ();
			}
			if (Layout.Button("Switch Toolbar", GUILayout.Height(25), GUILayout.Width(150))) {
				useToolbar = !useToolbar;
				DestroyLauncher ();
				CreateLauncher ();
			}
			if (HighLogic.LoadedSceneIsFlight)
			if (Layout.Button ("BV Control Panel", GUILayout.Height (25), GUILayout.Width (200))) {
				if (currentModule != null) {
					currentModule.SystemCheck ();
					rcVisible = true;
					if (appLauncherButton != null)
						appLauncherButton.SetFalse ();
					else
						onToggle ();
				}
			}
			GUILayout.EndHorizontal ();
			GUILayout.EndVertical();
			GUI.DragWindow();
		}


		/// <summary>
		/// Draws UI for module control
		/// </summary>
		/// <param name="rcGuiId">Rc GUI identifier.</param>
		public void DrawRcGUI(int rcGuiId) {
			if (currentModule == null) {
				rcVisible = false;
				return;
			}
			double currentTime = Planetarium.GetUniversalTime();
			GUILayout.BeginVertical ();
			TimeSpan t = TimeSpan.FromSeconds (currentTime - currentModule.lastTime);
			Layout.Label (
				string.Format (
					"Idle for {0:D2}h:{1:D2}m:{2:D2}s",
					t.Hours,
					t.Minutes,
					t.Seconds
				)
			);
			Layout.LabelAndText ("Target latitude", currentModule.targetLatitude.ToString());
			Layout.LabelAndText ("Target longitude", currentModule.targetLatitude.ToString());
			Layout.LabelAndText ("Distance to target", currentModule.distanceToTarget.ToString("N0"));
			Layout.LabelAndText ("Distance travelled", currentModule.distanceTravelled.ToString("N0"));
			Layout.LabelAndText ("Average speed", currentModule.averageSpeed.ToString("F"));
			Layout.LabelAndText ("Solar power", currentModule.solarProd.ToString("F"));
			Layout.LabelAndText ("Other power", currentModule.otherProd.ToString("F"));
			Layout.LabelAndText ("Fuel cell power", currentModule.fuelCellProd.ToString("F"));
			Layout.LabelAndText ("Power required", currentModule.powerRequired.ToString("F"));
			Layout.LabelAndText ("Is manned", currentModule.isManned.ToString ());

//			Layout.TextField ("lat");
//			Layout.TextField ("lon");
			if (Layout.Button ("Pick target on map", Palette.yellow)) {
				currentModule.PickTarget ();
			}
			if (Layout.Button ("Set to active target", Palette.yellow)) {
				currentModule.SetToActive ();
			}
			if (Layout.Button ("Set to active waypoint", Palette.yellow)) {
				currentModule.SetToWaypoint ();
			}
			if (!currentModule.isActive) {
				if (Layout.Button ("Poehali!!!", Palette.green)) {
					currentModule.Activate ();
				}
			} else {
				if (Layout.Button ("Deactivate", Palette.red)) {
					currentModule.Deactivate ();
				}
			}
			if (Layout.Button ("System Check", Palette.yellow)) {
				currentModule.SystemCheck ();
			}

			if (Layout.Button ("Close", Palette.red)) {
				rcVisible = false;
			}

//			if (Layout.Button ("LZString test")) {
//				currentModule.TestLZString ();
//			}

			GUILayout.EndVertical ();
			GUI.DragWindow ();
		}

		public void UpdateWayPoints() {
//			if (wayPoints != null) {
//				wayPoints.Clear ();
				wayPoints = PathUtils.DecodePath (currentModule.pathEncoded, currentModule.vessel.mainBody);
//			}
		}

		/// <summary>
		/// Enable module control UI, for use from BonVoyageModule
		/// </summary>
		public void ShowModuleControl() {
			rcVisible = true;
		}

		public void UpdateRoverState(Vessel vessel, bool stateActive) {
			for (int i = 0; i < activeRovers.Count; i++) {
				var rover = activeRovers [i];
				if (rover.vessel == vessel) {
					rover.bvActive = stateActive;
					if (stateActive)
						InputLockManager.SetControlLock (lockMask, "BonVoyageInputLock");
					else
						InputLockManager.RemoveControlLock ("BonVoyageInputLock");	
					return;
				}
			}
		}

		public void LoadRovers()
		{
			activeRovers.Clear();
			foreach (var vessel in FlightGlobals.Vessels)
			{
				ConfigNode vesselConfigNode = new ConfigNode();
				vessel.protoVessel.Save(vesselConfigNode);

				foreach(ConfigNode part in vesselConfigNode.GetNodes("PART")) {
					ConfigNode BVModule = part.GetNode("MODULE", "name", "BonVoyageModule");
					if (BVModule != null) {
						activeRovers.Add (new ActiveRover (vessel, BVModule, vesselConfigNode));
						break;
					}
				}
			}
		}
	}
}
