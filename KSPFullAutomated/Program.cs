using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace KSPFullAutomated
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Spacecraft.Initialize(new System.Net.IPAddress(new byte[] { 127, 0, 0, 1 }));

            //Spacecraft craft = Spacecraft.LaunchVehicleFromVAB("b");
            Spacecraft craft = new Spacecraft("b");
            craft.Engage();
            
            craft.LaunchToOrbit(200 * 1000);
            craft.Disengage();
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Form1());
        }
    }
}
