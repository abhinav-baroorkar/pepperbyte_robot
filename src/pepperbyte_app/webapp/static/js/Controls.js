// Controls.js — Simplified mapping controls.
// Pick a mode ONCE before starting. No switching mid-session.
(function () {
  const { useState, useEffect, useCallback } = React;
  const html = window.html;

  function post(url, body) {
    return fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    }).then(r => r.json());
  }

  window.Controls = function Controls({ status }) {
    // --- local state ---
    const [mapName, setMapName]             = useState('');
    const [saveFolder, setSaveFolder]       = useState('~/maps');
    const [selectedMode, setSelectedMode]   = useState('manual'); // manual | autonomous
    const [savedMaps, setSavedMaps]         = useState([]);
    const [message, setMessage]             = useState('');
    const [confirmRestart, setConfirmRestart] = useState(false);

    const isActive  = status.mapping_active && !status.mapping_paused;
    const isPaused  = status.mapping_paused;
    const isRunning = status.mapping_active; // active or paused — session exists

    // Expand ~ to absolute path
    function absFolder(f) { return f.replace('~', '/home/abhinav'); }

    // Load saved maps
    useEffect(() => {
      function load() {
        fetch('/api/maps?folder=' + encodeURIComponent(absFolder(saveFolder)))
          .then(r => r.json())
          .then(d => setSavedMaps(d.maps || []))
          .catch(() => {});
      }
      load();
      const t = setInterval(load, 10000);
      return () => clearInterval(t);
    }, [saveFolder]);

    function flash(msg) {
      setMessage(msg);
      setTimeout(() => setMessage(''), 4000);
    }

    // --- actions ---
    const startMapping = useCallback(() => {
      if (!mapName.trim()) { flash('Enter a map name first'); return; }
      const mode = selectedMode === 'autonomous' ? 'autonomous' : 'mapping';
      post('/api/start_mapping', {
        map_name: mapName.trim(),
        save_folder: absFolder(saveFolder),
        mode: mode,
      }).then(() => {
        post('/api/set_mode', { mode });
        flash('Mapping started — ' + (selectedMode === 'autonomous' ? 'Autonomous' : 'Manual'));
      });
    }, [mapName, saveFolder, selectedMode]);

    const pauseMapping = useCallback(() => {
      post('/api/stop_mapping', {}).then(d => flash(d.message || 'Mapping paused'));
    }, []);

    const resumeMapping = useCallback(() => {
      post('/api/resume_mapping', {}).then(d => flash(d.message || 'Mapping resumed'));
    }, []);

    const saveMap = useCallback(() => {
      post('/api/save_map', {
        map_name: mapName.trim() || 'untitled',
        save_folder: absFolder(saveFolder),
      }).then(d => flash(d.message || 'Map saved'))
        .catch(() => flash('Save failed'));
    }, [mapName, saveFolder]);

    const restartMapping = useCallback(() => {
      if (!confirmRestart) {
        setConfirmRestart(true);
        setTimeout(() => setConfirmRestart(false), 3000);
        return;
      }
      setConfirmRestart(false);
      post('/api/restart_mapping', {
        map_name: mapName.trim() || 'untitled',
        save_folder: absFolder(saveFolder),
      }).then(d => flash(d.message || 'Mapping restarted'))
        .catch(() => flash('Restart failed'));
    }, [confirmRestart, mapName, saveFolder]);

    // =====================================================================
    // RENDER
    // =====================================================================
    return html`
      <div class="p-4 space-y-4 flex-1 overflow-y-auto">

        <!-- ── SETUP (only before mapping starts) ── -->
        ${!isRunning && html`
          <div class="panel-card p-3 space-y-3">
            <h3 class="text-xs font-bold text-gray-400 tracking-widest uppercase">
              New Mapping Session
            </h3>

            <div>
              <label class="text-xs text-gray-500 mb-1 block">Map Name</label>
              <input type="text" placeholder="e.g. room1"
                value=${mapName}
                onInput=${(e) => setMapName(e.target.value)} />
            </div>

            <div>
              <label class="text-xs text-gray-500 mb-1 block">Save Folder</label>
              <input type="text" placeholder="~/maps"
                value=${saveFolder}
                onInput=${(e) => setSaveFolder(e.target.value)} />
            </div>

            <!-- Mode selector — chosen ONCE -->
            <div>
              <label class="text-xs text-gray-500 mb-1 block">Drive Mode</label>
              <div class="flex gap-2">
                <button
                  onClick=${() => setSelectedMode('manual')}
                  class=${
                    'flex-1 py-2 rounded-lg text-sm font-semibold transition-all ' +
                    (selectedMode === 'manual'
                      ? 'bg-pb-accent text-black mode-btn-active'
                      : 'bg-gray-700/50 text-gray-300 hover:bg-gray-600/50')
                  }>
                  Manual
                </button>
                <button
                  onClick=${() => setSelectedMode('autonomous')}
                  class=${
                    'flex-1 py-2 rounded-lg text-sm font-semibold transition-all ' +
                    (selectedMode === 'autonomous'
                      ? 'bg-pb-accent text-black mode-btn-active'
                      : 'bg-gray-700/50 text-gray-300 hover:bg-gray-600/50')
                  }>
                  Autonomous
                </button>
              </div>
            </div>

            <button
              onClick=${startMapping}
              disabled=${!mapName.trim()}
              class=${
                'w-full py-3 rounded-lg text-sm font-bold transition-all ' +
                (mapName.trim()
                  ? 'bg-pb-green/90 text-black hover:bg-pb-green'
                  : 'bg-gray-700 text-gray-500 cursor-not-allowed')
              }>
              START MAPPING
            </button>
          </div>

          <!-- Hint -->
          <div class="text-xs text-gray-600 space-y-1 px-1">
            <p><b class="text-gray-400">Manual</b> — drive with joystick / keyboard while map builds.</p>
            <p><b class="text-gray-400">Autonomous</b> — click on the map to send goals; Nav2 drives.</p>
          </div>
        `}

        <!-- ── ACTIVE SESSION CONTROLS (after mapping starts) ── -->
        ${isRunning && html`
          <div class="panel-card p-3 space-y-3">
            <div class="flex items-center justify-between">
              <h3 class="text-xs font-bold text-gray-400 tracking-widest uppercase">
                Mapping: ${mapName || 'untitled'}
              </h3>
              <span class=${
                'text-xs font-bold px-2 py-0.5 rounded-full ' +
                (status.mode === 'autonomous'
                  ? 'bg-purple-600 text-white'
                  : 'bg-blue-600 text-white')
              }>
                ${status.mode === 'autonomous' ? 'AUTO' : 'MANUAL'}
              </span>
            </div>

            <!-- Pause / Resume -->
            <div class="flex gap-2">
              ${isPaused
                ? html`
                  <button onClick=${resumeMapping}
                    class="flex-1 py-2 rounded-lg text-sm font-bold bg-pb-green/90 text-black hover:bg-pb-green transition-all">
                    RESUME
                  </button>`
                : html`
                  <button onClick=${pauseMapping}
                    class="flex-1 py-2 rounded-lg text-sm font-bold bg-yellow-500/90 text-black hover:bg-yellow-400 transition-all">
                    PAUSE
                  </button>`
              }
            </div>

            <!-- Save / Restart -->
            <div class="flex gap-2">
              <button onClick=${saveMap}
                class="flex-1 py-2 rounded-lg text-sm font-bold bg-blue-600 text-white hover:bg-blue-500 transition-all">
                SAVE MAP
              </button>
              <button onClick=${restartMapping}
                class=${
                  'flex-1 py-2 rounded-lg text-sm font-bold transition-all ' +
                  (confirmRestart
                    ? 'bg-pb-red text-white animate-pulse'
                    : 'bg-pb-red/60 text-white hover:bg-pb-red/80')
                }>
                ${confirmRestart ? 'CONFIRM?' : 'RESTART'}
              </button>
            </div>
          </div>

          <!-- Status hint -->
          <div class="text-xs text-gray-600 space-y-1 px-1">
            ${isActive && status.mode === 'autonomous' && html`
              <p>Click on the map to send a navigation goal.</p>
            `}
            ${isActive && status.mode !== 'autonomous' && html`
              <p>Drive with joystick (hold LB, RB for turbo).</p>
            `}
            ${isPaused && html`
              <p>Map frozen. Click RESUME to continue from current location.</p>
            `}
          </div>
        `}

        <!-- Flash message -->
        ${message && html`
          <div class="text-center text-xs text-pb-accent py-1 animate-pulse">
            ${message}
          </div>
        `}

        <!-- Saved Maps -->
        <div class="panel-card p-3 space-y-2">
          <h3 class="text-xs font-bold text-gray-400 tracking-widest uppercase">Saved Maps</h3>
          ${savedMaps.length === 0
            ? html`<p class="text-xs text-gray-600 italic">No maps saved yet</p>`
            : savedMaps.map(m => html`
                <div key=${m}
                  class="text-sm text-gray-300 py-1 px-2 rounded hover:bg-gray-700/40 flex items-center gap-2">
                  <span class="text-pb-accent">></span>
                  <span class="truncate">${m}</span>
                </div>
              `)
          }
        </div>
      </div>
    `;
  };
})();
