using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.UI.Screens;

namespace BonVoyage
{
	public class ActiveRover
	{
		public string status;
		public double toTravel;

		public Vessel vessel;
		private ConfigNode vesselConfigNode;
		private double lastTime;
		public double LastTime { get { return lastTime; } }
		private double targetLatitude;
		private double targetLongitude;

		private double averageSpeed;
		private double speedMultiplier;
		public double AverageSpeed { get { return averageSpeed * speedMultiplier; } }

		private double distanceTravelled;
		private double distanceToTarget;
		public double yetToTravel { get { return distanceToTarget - distanceTravelled; } }

		private double solarPower;
		private double fuelCellPower;
		private double otherPower;
		private double requiredPower;
		public bool kerbalismFuelCell;
		public double stockFuelCellECMax;
		public double stockFuelCellECRemain;
		public bool bvActive;
		private bool isManned;
		private ConfigNode BVModule;
		private List<PathUtils.WayPoint> path;

		/// <summary>
		/// Initializes a new instance of the <see cref="BonVoyage.ActiveRover"/> class.
		/// </summary>
		/// <param name="v">Vessel.</param>
		/// <param name="module">Bon Voyage Module.</param>
		/// <param name="vcf">Vessel Config Node.</param>
		public ActiveRover(Vessel v, ConfigNode module, ConfigNode vcf)
		{
			vessel = v;
			vesselConfigNode = vcf;

			BVModule = module;

			bvActive = bool.Parse(BVModule.GetValue("isActive"));

			// Workaround for update from versions prior to 1.0
			try
			{
				isManned = bool.Parse(BVModule.GetValue("isManned"));
			}
			catch
			{
				isManned = true;
			}

			solarPower = double.Parse(BVModule.GetValue("solarProd"));
			fuelCellPower = double.Parse(BVModule.GetValue("fuelCellProd"));
			otherPower = double.Parse(BVModule.GetValue("otherProd"));
			requiredPower = double.Parse(BVModule.GetValue("powerRequired"));
			kerbalismFuelCell = bool.Parse(BVModule.GetValue("kerbalismFuelCell"));
			stockFuelCellECMax = double.Parse(BVModule.GetValue("stockFuelCellECMax"));
			stockFuelCellECRemain = double.Parse(BVModule.GetValue("stockFuelCellECRemain"));

			lastTime = double.Parse(BVModule.GetValue("lastTime"));
			distanceTravelled = double.Parse(BVModule.GetValue("distanceTravelled"));
			distanceToTarget = double.Parse(BVModule.GetValue("distanceToTarget"));
			targetLatitude = double.Parse(BVModule.GetValue("targetLatitude"));
			targetLongitude = double.Parse(BVModule.GetValue("targetLongitude"));
			averageSpeed = double.Parse(BVModule.GetValue("averageSpeed"));

			path = PathUtils.DecodePath(BVModule.GetValue("pathEncoded"));
			speedMultiplier = 1.0;
		}

		/// <summary>
		/// Update rover.
		/// </summary>
		/// <param name="currentTime">Current time.</param>
		/// <param name="module">BVModule.</param>
		/// <param name="vcf">Vessel Config Node.</param>
		public void Update(double currentTime, ConfigNode module, ConfigNode vcf)
		{
			if (vessel.isActiveVessel)
			{
				status = "current";
				return;
			}

			if (!bvActive || vessel.loaded)
			{
				status = "idle";
				return;
			}

			Vector3d vesselPos = vessel.mainBody.position - vessel.GetWorldPos3D();
			Vector3d toKerbol = vessel.mainBody.position - FlightGlobals.Bodies[0].position;
			double angle = Vector3d.Angle(vesselPos, toKerbol);
			vesselConfigNode = vcf;
			BVModule = module;
			// Speed penalties at twighlight and at night
			if (angle > 90 && isManned)
				speedMultiplier = 0.25;
			else if (angle > 85 && isManned)
				speedMultiplier = 0.5;
			else if (angle > 80 && isManned)
				speedMultiplier = 0.75;
			else
				speedMultiplier = 1.0;

			double deltaT = currentTime - lastTime;
			double deltaEC = 0.0;
			if (kerbalismFuelCell)
				deltaEC = deltaT * requiredPower;
			else if (angle > 90 && requiredPower > otherPower)
				deltaEC = deltaT * (requiredPower - otherPower);
			else if (requiredPower > otherPower + solarPower)
				deltaEC = deltaT * (requiredPower - otherPower - solarPower);

			// max power
			if (solarPower + fuelCellPower + otherPower < requiredPower || !EnoughEC(deltaEC))
			{
				status = "not enough power";
				lastTime = currentTime;
				BVModule.SetValue("lastTime", currentTime.ToString());
				vessel.protoVessel = new ProtoVessel(vesselConfigNode, HighLogic.CurrentGame);
				return;
			}

			// At night, don't move if other OR otherANDfuelcellnotdepleted can't provide enough power
			if (angle > 90 && !(otherPower > requiredPower || (otherPower + fuelCellPower > requiredPower && !FuelDepleted())))
			{
				status = "awaiting sunlight";
				lastTime = currentTime;
				BVModule.SetValue("lastTime", currentTime.ToString());
				vessel.protoVessel = new ProtoVessel(vesselConfigNode, HighLogic.CurrentGame);
				return;
			}

			// At day, don't move if other OR otherANDsolar ...
			if (angle <= 90 && !(otherPower > requiredPower || (otherPower + solarPower > requiredPower) || !FuelDepleted()))
			{
				status = "fuel cells resource depleted";
				lastTime = currentTime;
				BVModule.SetValue("lastTime", currentTime.ToString());
				vessel.protoVessel = new ProtoVessel(vesselConfigNode, HighLogic.CurrentGame);
				return;
			}

			ECdrain(deltaEC);

			double deltaS = AverageSpeed * deltaT;
			double bearing = GeoUtils.InitialBearing(
				vessel.latitude,
				vessel.longitude,
				targetLatitude,
				targetLongitude
			);
			distanceTravelled += deltaS;
			if (distanceTravelled >= distanceToTarget)
			{
				// vessel.latitude = targetLatitude;
				// vessel.longitude = targetLongitude;
				if (!MoveSafe(targetLatitude, targetLongitude))
					distanceTravelled -= deltaS;
				else
				{
					distanceTravelled = distanceToTarget;

					bvActive = false;
					BVModule.SetValue("isActive", "False");
					BVModule.SetValue("distanceTravelled", distanceToTarget.ToString());
					BVModule.SetValue("pathEncoded", "");

					LqdFuelOxiDrain(stockFuelCellECMax - stockFuelCellECRemain);

					// BVModule.GetNode ("EVENTS").GetNode ("Activate").SetValue ("active", "True");
					// BVModule.GetNode ("EVENTS").GetNode ("Deactivate").SetValue ("active", "False");

					if (BonVoyage.Instance.AutoDewarp)
					{
						if (TimeWarp.CurrentRate > 3)
							TimeWarp.SetRate(3, true);
						if (TimeWarp.CurrentRate > 0)
							TimeWarp.SetRate(0, false);
						ScreenMessages.PostScreenMessage(vessel.vesselName + " has arrived to destination at " + vessel.mainBody.name);
					}
					HoneyImHome();
				}
				status = "idle";
			}
			else
			{
				int step = Convert.ToInt32(Math.Floor(distanceTravelled / PathFinder.StepSize));
				double remainder = distanceTravelled % PathFinder.StepSize;

				if (step < path.Count - 1)
					bearing = GeoUtils.InitialBearing(
						path[step].latitude,
						path[step].longitude,
						path[step + 1].latitude,
						path[step + 1].longitude
					);
				else
					bearing = GeoUtils.InitialBearing(
						path[step].latitude,
						path[step].longitude,
						targetLatitude,
						targetLongitude
					);

				double[] newCoordinates = GeoUtils.GetLatitudeLongitude(
					path[step].latitude,
					path[step].longitude,
					bearing,
					remainder,
					vessel.mainBody.Radius
				);

				// vessel.latitude = newCoordinates[0];
				// vessel.longitude = newCoordinates[1];
				if (!MoveSafe(newCoordinates[0], newCoordinates[1]))
				{
					distanceTravelled -= deltaS;
					status = "idle";
				}
				else
					status = "roving";
			}
			// vessel.altitude = GeoUtils.TerrainHeightAt(vessel.latitude, vessel.longitude, vessel.mainBody);
			Save(currentTime);
		}

		/// <summary>
		/// Save data to ProtoVessel.
		/// </summary>
		public void Save(double currentTime)
		{
			lastTime = currentTime;
			vesselConfigNode.SetValue("lat", vessel.latitude.ToString());
			vesselConfigNode.SetValue("lon", vessel.longitude.ToString());
			vesselConfigNode.SetValue("alt", vessel.altitude.ToString());
			vesselConfigNode.SetValue("landedAt", vessel.mainBody.theName);
			BVModule.SetValue("distanceTravelled", (distanceTravelled).ToString());
			BVModule.SetValue("lastTime", currentTime.ToString());
			vessel.protoVessel = new ProtoVessel(vesselConfigNode, HighLogic.CurrentGame);
		}

		/// <summary>
		/// Test if hydrogen or oxygen tank are empty.
		/// </summary>
		public bool FuelDepleted()
		{
			var partList = vesselConfigNode.GetNodes("PART");

			if (kerbalismFuelCell)
			{
				bool hydrogenDepleted = true;
				bool oxygenDepleted = true;

				for (int i = 0; i < partList.Length; i++)
				{
					if (hydrogenDepleted)
					{
						ConfigNode hydrogenNode = partList[i].GetNode("RESOURCE", "name", "Hydrogen");
						if (hydrogenNode != null && double.Parse(hydrogenNode.GetValue("amount")) > 0.0)
						{
							hydrogenDepleted = false;
							if (!oxygenDepleted)
								break;
						}
					}
					if (oxygenDepleted)
					{
						ConfigNode oxygenNode = partList[i].GetNode("RESOURCE", "name", "Oxygen");
						if (oxygenNode != null && double.Parse(oxygenNode.GetValue("amount")) > 0.0)
						{
							oxygenDepleted = false;
							if (!hydrogenDepleted)
								break;
						}
					}
				}

				return (hydrogenDepleted || oxygenDepleted);
			}
			else
				return (stockFuelCellECRemain == 0.0);
		}

		/// <summary>
		/// Test if EC > 50% and if ecToDrain smaller than ecAmount
		/// </summary>
		public bool EnoughEC(double ecToDrain)
		{
			if (!kerbalismFuelCell)
				return true;

			double ecAmount = 0.0;
			double ecMaxAmount = 0.0;
			var partList = vesselConfigNode.GetNodes("PART");

			for (int i = 0; i < partList.Length; ++i)
			{
				ConfigNode ecNode = partList[i].GetNode("RESOURCE", "name", "ElectricCharge");
				if (ecNode != null)
				{
					ecAmount += double.Parse(ecNode.GetValue("amount"));
					ecMaxAmount += double.Parse(ecNode.GetValue("maxAmount"));
				}
			}
			return (ecAmount > ecMaxAmount / 2.0 && ecToDrain < 0.2 * ecMaxAmount);
		}

		/// <summary>
		/// Drain EC, returns true if all EC asked has been drained
		/// </summary>
		public bool ECdrain(double ecToDrain)
		{
			if (kerbalismFuelCell)
			{
				double ecRemainder = ecToDrain;
				var partList = vesselConfigNode.GetNodes("PART");

				for (int i = 0; i < partList.Length; i++)
				{
					ConfigNode ecNode = partList[i].GetNode("RESOURCE", "name", "ElectricCharge");
					if (ecNode != null)
					{
						double ecAmount = double.Parse(ecNode.GetValue("amount"));
						if (ecAmount >= ecRemainder)
						{
							ecNode.SetValue("amount", ecAmount - ecRemainder);
							ecRemainder = 0.0;
							break;
						}
						else
						{
							ecNode.SetValue("amount", 0.0);
							ecRemainder -= ecAmount;
						}
					}
				}

				return (ecRemainder == 0.0);
			}
			else
			{
				if (stockFuelCellECRemain >= ecToDrain)
				{
					stockFuelCellECRemain -= ecToDrain;
					BVModule.SetValue("stockFuelCellECRemain", stockFuelCellECRemain.ToString());
					return true;
				}
				else
				{
					stockFuelCellECRemain = 0.0;
					BVModule.SetValue("stockFuelCellECRemain", stockFuelCellECRemain.ToString());
					return false;
				}
			}
		}

		/// <summary>
		/// Drain stock Fuel cell input resources, returns true if all EC asked has been drained
		/// </summary>
		public bool LqdFuelOxiDrain(double ecToDrain)
		{
			if (kerbalismFuelCell)
				return true;
			
			double lqdFuelRemainder = ecToDrain * 0.001125;
			double oxiRemainder = ecToDrain * 0.001375;
			var partList = vesselConfigNode.GetNodes("PART");

			for (int i = 0; i < partList.Length; i++)
			{
				ConfigNode lqdFuelNode = partList[i].GetNode("RESOURCE", "name", "LiquidFuel");
				if (lqdFuelNode != null)
				{
					double lqdFuelAmount = double.Parse(lqdFuelNode.GetValue("amount"));
					if (lqdFuelAmount >= lqdFuelRemainder)
					{
						lqdFuelNode.SetValue("amount", lqdFuelAmount - lqdFuelRemainder);
						lqdFuelRemainder = 0.0;
						if(oxiRemainder == 0.0)
							break;
					}
					else
					{
						lqdFuelNode.SetValue("amount", 0.0);
						lqdFuelRemainder -= lqdFuelAmount;
					}
				}

				ConfigNode oxiNode = partList[i].GetNode("RESOURCE", "name", "Oxidizer");
				if (oxiNode != null)
				{
					double oxiAmount = double.Parse(oxiNode.GetValue("amount"));
					if (oxiAmount >= oxiRemainder)
					{
						oxiNode.SetValue("amount", oxiAmount - oxiRemainder);
						oxiRemainder = 0.0;
						if (lqdFuelRemainder == 0.0)
							break;
					}
					else
					{
						oxiNode.SetValue("amount", 0.0);
						oxiRemainder -= oxiAmount;
					}
				}
			}
			return (lqdFuelRemainder == 0.0 && oxiRemainder == 0.0);
		}

		/// <summary>
		/// Prevent crazy torpedoing active vessel :D
		/// </summary>
		/// <returns><c>true</c>, if rover was moved, <c>false</c> otherwise.</returns>
		/// <param name="latitude">Latitude.</param>
		/// <param name="longitude">Longitude.</param>
		private bool MoveSafe(double latitude, double longitude)
		{
			double altitude = GeoUtils.TerrainHeightAt(latitude, longitude, vessel.mainBody);
			if (FlightGlobals.ActiveVessel != null)
			{
				Vector3d newPos = vessel.mainBody.GetWorldSurfacePosition(latitude, longitude, altitude);
				Vector3d actPos = FlightGlobals.ActiveVessel.GetWorldPos3D();
				double distance = Vector3d.Distance(newPos, actPos);
				if (distance <= 2400)
				{
					return false;
				}
				// VesselRanges ranges = active.vesselRanges.GetSituationRanges(Vessel.Situations.LANDED || Vessel.Situations.FLYING);
				// vessel.GoOffRails ();
				// vessel.Load ();
			}

			vessel.latitude = latitude;
			vessel.longitude = longitude;
			vessel.altitude = altitude;
			return true;
		}

		/// <summary>
		/// Notify that rover has arrived
		/// </summary>
		private void HoneyImHome()
		{
			MessageSystem.Message message = new MessageSystem.Message(
				"Rover arrived",
				//------------------------------------------
				"<color=#74B4E2>" + vessel.vesselName + "</color>" +
				" has arrived to destination\n<color=#AED6EE>LAT:" +
				targetLatitude.ToString("F2") + "</color>\n<color=#AED6EE>LON:" +
				targetLongitude.ToString("F2") +
				"</color>\n<color=#82BCE5>At " + vessel.mainBody.name + ".</color>\n" +
				"Distance travelled: " +
				"<color=#74B4E2>" + distanceTravelled.ToString("N") + "</color> meters",
				//------------------------------------------
				MessageSystemButton.MessageButtonColor.GREEN,
				MessageSystemButton.ButtonIcons.COMPLETE
			);
			MessageSystem.Instance.AddMessage(message);
		}
	}
}
