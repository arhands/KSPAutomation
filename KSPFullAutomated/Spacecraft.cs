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
        public Spacecraft(string name)
        {
            _Vessel = Client.SpaceCenter().Vessels.First(p => p.Name == name);
            var flight = _Vessel.Flight();
            Altitude = Client.AddStream(() => flight.MeanAltitude);
        }
        public void Engage() => _Vessel.AutoPilot.Engage();
        public void Disengage() => _Vessel.AutoPilot.Disengage();
        private Vessel _Vessel { get; set; }
        private double ErrorMarigin { get; set; }
        private Stream<double> Altitude { get; set; }
        public void LaunchToOrbit(int desiredAltitude)
        {
            
            //ascent code
            double maxVelocity = 400;
            _Vessel.Control.Throttle = 1;
            while (true)
            {
                if (_Vessel.Orbit.ApoapsisAltitude >= desiredAltitude)
                {
                    _Vessel.Control.Throttle = 0;
                    if (_Vessel.Orbit.Body.AtmosphereDepth < Altitude.Get())
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

                    _Vessel.AutoPilot.TargetPitchAndHeading((float)(90 * (1 - _Vessel.Orbit.ApoapsisAltitude / desiredAltitude)), 90);

                    //System.Diagnostics.Debug.WriteLine("Pitch = " + (float)(90 * (1 - Altitude.Get() / desiredAltitude)) + ", Error = " + _Vessel.AutoPilot.Error);
                    var v = _Vessel.Velocity(_Vessel.Orbit.Body.ReferenceFrame);
                    //WriteLine("Speed = " + Math.Sqrt(v.Item1 * v.Item1 + v.Item2 * v.Item2 + v.Item3 * v.Item3));
                    _Vessel.Control.Throttle += (float)(0.01*(maxVelocity - Math.Sqrt(v.Item1 * v.Item1 + v.Item2 * v.Item2 + v.Item3 * v.Item3)));
                    System.Threading.Thread.Sleep(StandardDelay);
                    CheckStage();
                }
            }
            //bring up the periapsis
            WriteLine("Bringing up the pariapsis");
            _Vessel.AutoPilot.ReferenceFrame = _Vessel.OrbitalReferenceFrame;
            var node = CreateNode(Elements.Periapsis, desiredAltitude + _Vessel.Orbit.Body.EquatorialRadius);
            WriteLine("DeltaV = " + node.Prograde);
            ExecuteManuverNode(node, 10, 50);
            node.Remove();
            //calling circularize method
            WriteLine("Circularizing");
            Circularize(desiredAltitude);
        }
        private void Circularize(double desiredAltitude, int maxManuvers = 3)
        {
            for(int i = 0; i < maxManuvers; i++)
            {
                if (Math.Abs(_Vessel.Orbit.Apoapsis - desiredAltitude - _Vessel.Orbit.Body.EquatorialRadius) <= ErrorMarigin && 
                    Math.Abs(_Vessel.Orbit.Periapsis - desiredAltitude- _Vessel.Orbit.Body.EquatorialRadius) <= ErrorMarigin)
                    return;
                var node = CreateNode(Elements.Periapsis, desiredAltitude + _Vessel.Orbit.Body.EquatorialRadius);
                ExecuteManuverNode(node, 3);
                node.Remove();

                node = CreateNode(Elements.Apoapsis, desiredAltitude + _Vessel.Orbit.Body.EquatorialRadius);
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
                _Vessel.Orbit.Body.GravitationalParameter * 
                (2 / radius - 1 / _Vessel.Orbit.SemiMajorAxis));
        double VisaVersaEquation(double r, double a) => Math.Sqrt(_Vessel.Orbit.Body.GravitationalParameter * (2 / r - 1 / a));
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
                        return _Vessel.Control.AddNode(
                            _Vessel.Orbit.TimeToPeriapsis + UniversalTime.Get(),
                            (float)(VisaVersaEquation(_Vessel.Orbit.Periapsis, (_Vessel.Orbit.Periapsis + newValue) / 2)
                                - VelocityAtRadius(_Vessel.Orbit.Periapsis)));
                    }
                case Elements.Periapsis:
                    {
                        return _Vessel.Control.AddNode(
                            _Vessel.Orbit.TimeToApoapsis + UniversalTime.Get(),
                            (float)(VisaVersaEquation(_Vessel.Orbit.Apoapsis, (_Vessel.Orbit.Apoapsis + newValue) / 2)
                                - VelocityAtRadius(_Vessel.Orbit.Apoapsis)));
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
            if (_Vessel.Thrust == 0 && _Vessel.Control.Throttle > 0)
                _Vessel.Control.ActivateNextStage();
        }
        /// <summary>
        /// executes 'node,' does NOT delete 'node'
        /// </summary>
        /// <param name="node"></param>
        /// <param name="burnAngleErrMarigin"></param>
        void ExecuteManuverNode(Node node, double burnAngleErrMarigin, double deltaVErrMarigin = 0.1)
        {
            //telling the autopilot to point the spacecraft at the node
            _Vessel.AutoPilot.TargetDirection = node.BurnVector();

            //weighting for the vessel to be pointed at the node
            while (_Vessel.AutoPilot.Error > burnAngleErrMarigin)
                System.Threading.Thread.Sleep(StandardDelay);

            //warping to the manuver
            WaitUntil(UniversalTime.Get() + node.TimeTo);

            //executing the manuver
            while (node.RemainingDeltaV > deltaVErrMarigin)
            {
                _Vessel.AutoPilot.TargetDirection = node.RemainingBurnVector(_Vessel.OrbitalReferenceFrame);
                if (_Vessel.AutoPilot.Error <= burnAngleErrMarigin)
                {
                    if (node.RemainingDeltaV >= 100)
                    {
                        _Vessel.Control.Throttle = 1;
                    }
                    else
                    {
                        _Vessel.Control.Throttle = (float)(node.RemainingDeltaV / 100);
                    }
                }
                else
                    _Vessel.Control.Throttle = 0;
                CheckStage();
                System.Threading.Thread.Sleep(StandardDelay);
                WriteLine("Delta-V = " + node.RemainingDeltaV);
            }
            _Vessel.Control.Throttle = 0;
        }
    }
}
