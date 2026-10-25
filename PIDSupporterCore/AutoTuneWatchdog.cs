using UnityEngine;

namespace PIDSupporter
{
    // Runs even after the PID/tuner window closes. No background thread accesses Unity.
    internal sealed class AutoTuneWatchdog : MonoBehaviour
    {
        private void Update() { AutoTuneDiagnostics.Tick(); AutoTuneSession.Active?.CheckAlive(); }
        private void OnApplicationQuit() { AutoTuneSession.Active?.Cancel(); AutoTuneWindowLifetime.CloseAll(); }
        private void OnDestroy() { AutoTuneSession.Active?.Cancel(); AutoTuneWindowLifetime.CloseAll(); }
    }
}
