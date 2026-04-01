// app.js — Main React application, SocketIO connection, state management.
(function () {
  const { useState, useEffect, useCallback } = React;
  const html = window.html;

  function App() {
    const [mapData, setMapData]       = useState(null);
    const [robotPose, setRobotPose]   = useState(null);
    const [scanPoints, setScanPoints] = useState([]);
    const [connected, setConnected]   = useState(false);
    const [status, setStatus]         = useState({
      mode: 'teleop',
      linear_vel: 0,
      angular_vel: 0,
      mapping_active: false,
      mapping_paused: false,
      map_name: '',
      save_folder: '~/maps',
    });

    // --- SocketIO connection ---
    useEffect(() => {
      const socket = io({ transports: ['websocket', 'polling'] });

      socket.on('connect',    () => setConnected(true));
      socket.on('disconnect', () => setConnected(false));

      socket.on('map_update',    (d) => setMapData(d));
      socket.on('pose_update',   (d) => setRobotPose(d));
      socket.on('scan_update',   (d) => setScanPoints(d.points || []));
      socket.on('status_update', (d) => setStatus(d));

      return () => socket.disconnect();
    }, []);

    // Only allow goal clicks in autonomous mode when mapping is active
    const canClickGoal = status.mode === 'autonomous'
                      && status.mapping_active
                      && !status.mapping_paused;

    const handleGoalClick = useCallback((x, y, theta) => {
      fetch('/api/send_goal', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x, y, theta }),
      });
    }, []);

    // --- layout ---
    return html`
      <div class="flex h-full">

        <!-- Left: Map Canvas -->
        <div class="flex-1 flex flex-col min-w-0">
          <!-- Top bar -->
          <div class="px-4 py-2 bg-pb-panel/90 flex items-center justify-between
                      border-b border-gray-700/50 backdrop-blur">
            <div class="flex items-center gap-3">
              <span class="text-pb-accent font-extrabold text-lg tracking-wider">Peppermint Robotics</span>
            </div>
            <div class="flex items-center gap-2 text-xs">
              <span class=${connected ? 'text-pb-green' : 'text-pb-red'}>
                ${connected ? 'LIVE' : 'OFFLINE'}
              </span>
              <span class=${
                'inline-block w-2 h-2 rounded-full ' +
                (connected ? 'bg-pb-green pulse-green' : 'bg-pb-red')
              } />
            </div>
          </div>

          <!-- Canvas -->
          <div class="flex-1 relative overflow-hidden">
            <${window.MapCanvas}
              mapData=${mapData}
              robotPose=${robotPose}
              scanPoints=${scanPoints}
              mode=${canClickGoal ? 'autonomous' : 'view'}
              onGoalClick=${canClickGoal ? handleGoalClick : null}
            />
          </div>
        </div>

        <!-- Right: Status + Controls sidebar -->
        <div class="w-80 flex-shrink-0 bg-pb-panel flex flex-col
                    border-l border-gray-700/50 overflow-hidden">
          <${window.StatusBar} status=${status} connected=${connected} />
          <${window.Controls}  status=${status} />
        </div>

      </div>
    `;
  }

  // Mount
  const root = ReactDOM.createRoot(document.getElementById('root'));
  root.render(html`<${App} />`);
})();
