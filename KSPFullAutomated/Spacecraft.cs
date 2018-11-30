using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using static System.Diagnostics.Debug;

namespace KSPFullAutomated
{
    class Spacecraft
    {
        static Stream<double> UniversalTime { get; set; }
        static Connection Client { get; set; }
        const int StandardDelay = 500;
        public static void Initialize(IPAddress address)
        {
            Client = new Connection("ksp autopilot", address);
            UniversalTime = Client.AddStream(() => Client.SpaceCenter().UT);
        }
        public static void SendVehicleToLaunchPad(string name)
        {
            Client.SpaceCenter().LaunchVesselFromVAB(name);
        }
        public static Spacecraft LaunchVehicleFromVAB(string name)
        {
            SendVehicleToLaunchPad(name);
            return new Spacecraft(name);
        }
        public Spacecraft(string name)
        {
            Ship = Client.SpaceCenter().Vessels.First(p => p.Name == name);
            var flight = Ship.Flight();
            Altitude = Client.AddStream(() => flight.MeanAltitude);
        }
        public void Engage() => Ship.AutoPilot.Engage();
        public void Disengage() => Ship.AutoPilot.Disengage();
        private Vessel Ship { get; set; }
        private double ErrorMarigin { get; set; } = 0.1;
        private Stream<double> Altitude { get; set; }
        /// <summary>
        /// this will launch a landed vehicle to orbit
        /// </summary>
        /// <param name="desiredAltitude"></param>
        public void LaunchToOrbit(int desiredAltitude)
        {
            
            //ascent code
            double maxVelocity = 400;
            Ship.Control.Throttle = 1;
            while (true)
            {
                if (Ship.Orbit.ApoapsisAltitude >= desiredAltitude)
                {
                    Ship.Control.Throttle = 0;
                    if (Ship.Orbit.Body.AtmosphereDepth < Altitude.Get())
                        break;
                }
                else
                {
                    if (maxVelocity == 400 && Altitude.Get() > 10000)
                    {
                        maxVelocity = 1000;
                        WriteLine("Max Velocity = " + maxVelocity);
                    }
                    else if (maxVelocity == 1000 && Altitude.Get() > 20000)
                    {
                        maxVelocity = 2000;
                        WriteLine("Max Velocity = " + maxVelocity);
                    }

                    Ship.AutoPilot.TargetPitchAndHeading((float)(90 * (1 - Ship.Orbit.ApoapsisAltitude / desiredAltitude)), 90);

                    //System.Diagnostics.Debug.WriteLine("Pitch = " + (float)(90 * (1 - Altitude.Get() / desiredAltitude)) + ", Error = " + _Vessel.AutoPilot.Error);
                    var v = Ship.Velocity(Ship.Orbit.Body.ReferenceFrame);
                    //WriteLine("Speed = " + Math.Sqrt(v.Item1 * v.Item1 + v.Item2 * v.Item2 + v.Item3 * v.Item3));
                    Ship.Control.Throttle += (float)(0.01*(maxVelocity - Math.Sqrt(v.Item1 * v.Item1 + v.Item2 * v.Item2 + v.Item3 * v.Item3)));
                    System.Threading.Thread.Sleep(StandardDelay);
                    CheckStage();
                }
            }
            //bring up the periapsis
            WriteLine("Bringing up the pariapsis");
            Ship.AutoPilot.ReferenceFrame = Ship.OrbitalReferenceFrame;
            var node = CreateNode(Elements.Periapsis, desiredAltitude + Ship.Orbit.Body.EquatorialRadius);
            WriteLine("DeltaV = " + node.Prograde);
            ExecuteManuverNode(node, 10, ErrorMarigin);
            node.Remove();
            //calling circularize method
            WriteLine("Circularizing");
            Circularize(desiredAltitude);
        }
        /// <summary>
        /// Hohmann Transfer
        /// </summary>
        /// <param name="desiredAltitude"></param>
        /// <param name="deltaVErrorMarigin"></param>
        private void ChangeAltitude(double desiredAltitude, double deltaVErrorMarigin = 0.5)
        {
            Node node = CreateNode(Elements.Apoapsis, desiredAltitude);
            ExecuteManuverNode(node, 0.1, deltaVErrorMarigin);
            node = CreateNode(Elements.Periapsis, desiredAltitude);
            ExecuteManuverNode(node, 0.1, deltaVErrorMarigin);
        }
        private void Circularize(double desiredAltitude, int maxManuvers = 3)
        {
            for(int i = 0; i < maxManuvers; i++)
            {
                if (Math.Abs(Ship.Orbit.Apoapsis - desiredAltitude - Ship.Orbit.Body.EquatorialRadius) <= ErrorMarigin && 
                    Math.Abs(Ship.Orbit.Periapsis - desiredAltitude- Ship.Orbit.Body.EquatorialRadius) <= ErrorMarigin)
                    return;
                var node = CreateNode(Elements.Periapsis, desiredAltitude + Ship.Orbit.Body.EquatorialRadius);
                ExecuteManuverNode(node, 3);
                node.Remove();

                node = CreateNode(Elements.Apoapsis, desiredAltitude + Ship.Orbit.Body.EquatorialRadius);
                ExecuteManuverNode(node, 3);
                node.Remove();
            }
        }
        /// <summary>
        /// computes the velocity at the given radius using the "visa-versa equation"
        /// </summary>
        /// <param name="radius"></param>
        /// <returns></returns>
        private double VelocityAtRadius(double radius) =>
            Math.Sqrt(
                Ship.Orbit.Body.GravitationalParameter * 
                (2 / radius - 1 / Ship.Orbit.SemiMajorAxis));
        double VisaVersaEquation(double r, double a) => Math.Sqrt(Ship.Orbit.Body.GravitationalParameter * (2 / r - 1 / a));
        enum Elements
        {
            Apoapsis = 0,
            Periapsis,
        }
        private Node CreateNode(Elements changing, double newValue)
        {
            switch (changing)
            {
                case Elements.Apoapsis:
                    {
                        return Ship.Control.AddNode(
                            Ship.Orbit.TimeToPeriapsis + UniversalTime.Get(),
                            (float)(VisaVersaEquation(Ship.Orbit.Periapsis, (Ship.Orbit.Periapsis + newValue) / 2)
                                - VelocityAtRadius(Ship.Orbit.Periapsis)));
                    }
                case Elements.Periapsis:
                    {
                        return Ship.Control.AddNode(
                            Ship.Orbit.TimeToApoapsis + UniversalTime.Get(),
                            (float)(VisaVersaEquation(Ship.Orbit.Apoapsis, (Ship.Orbit.Apoapsis + newValue) / 2)
                                - VelocityAtRadius(Ship.Orbit.Apoapsis)));
                    }
                default: throw new NotImplementedException();
            }
        }
        public static void WaitUntil(double time)
        {
            while (UniversalTime.Get() < time)
            {
                double remaining = time - UniversalTime.Get();
                int newWarpFacter;
                if (remaining < 10)
                    newWarpFacter = 0;
                else if (remaining < 60)
                    newWarpFacter = 2;
                else if (remaining < 60 * 60)
                    newWarpFacter = 4;
                else
                    newWarpFacter = 7;
                if (Client.SpaceCenter().RailsWarpFactor != newWarpFacter)
                    Client.SpaceCenter().RailsWarpFactor = newWarpFacter;
                System.Threading.Thread.Sleep(StandardDelay);
            }
            Client.SpaceCenter().RailsWarpFactor = 0;
        }
        void CheckStage()
        {
            if (Ship.Thrust == 0 && Ship.Control.Throttle > 0)
                Ship.Control.ActivateNextStage();
        }
        /// <summary>
        /// executes 'node,' does NOT delete 'node'
        /// </summary>
        /// <param name="node"></param>
        /// <param name="burnAngleErrMarigin"></param>
        void ExecuteManuverNode(Node node, double burnAngleErrMarigin, double deltaVErrMarigin = 0.1)
        {
            //telling the autopilot to point the spacecraft at the node
            Ship.AutoPilot.TargetDirection = node.BurnVector();

            //weighting for the vessel to be pointed at the node
            while (Ship.AutoPilot.Error > burnAngleErrMarigin)
                System.Threading.Thread.Sleep(StandardDelay);

            //warping to the manuver
            WaitUntil(UniversalTime.Get() + node.TimeTo);

            //executing the manuver
            while (node.RemainingDeltaV > deltaVErrMarigin)
            {
                Ship.AutoPilot.TargetDirection = node.RemainingBurnVector(Ship.OrbitalReferenceFrame);
                if (Ship.AutoPilot.Error <= burnAngleErrMarigin)
                {
                    if (node.RemainingDeltaV >= 100)
                    {
                        Ship.Control.Throttle = 1;
                    }
                    else
                    {
                        Ship.Control.Throttle = (float)(node.RemainingDeltaV / 100);
                    }
                }
                else
                    Ship.Control.Throttle = 0;
                CheckStage();
                System.Threading.Thread.Sleep(StandardDelay);
                WriteLine("Delta-V = " + node.RemainingDeltaV);
            }
            Ship.Control.Throttle = 0;
        }
    }
}
