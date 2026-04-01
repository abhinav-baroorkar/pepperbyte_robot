// StatusBar.js — Robot status display: mode badge, speed, mapping state.
(function () {
  const html = window.html;

  const MODE_COLOURS = {
    teleop:     { bg: 'bg-gray-600',    text: 'IDLE' },
    mapping:    { bg: 'bg-blue-600',    text: 'MANUAL' },
    autonomous: { bg: 'bg-purple-600',  text: 'AUTONOMOUS' },
  };

  window.StatusBar = function StatusBar({ status, connected }) {
    const m = MODE_COLOURS[status.mode] || MODE_COLOURS.teleop;

    return html`
      <div class="p-4 space-y-4">

        <!-- Header -->
        <div class="text-center">
          <h1 class="text-xl font-bold tracking-widest text-pb-accent">PEPPERMINT ROBOTICS</h1>
          <p class="text-xs text-gray-400 mt-1">Mapping Control App</p>
        </div>

        <hr class="border-gray-700" />

        <!-- Connection -->
        <div class="flex items-center justify-between text-sm">
          <span class="text-gray-400">Connection</span>
          <span class=${connected
            ? 'text-pb-green font-semibold'
            : 'text-pb-red font-semibold'}>
            ${connected ? 'Connected' : 'Disconnected'}
          </span>
        </div>

        <!-- Mode badge -->
        <div class="panel-card p-3 space-y-2">
          <div class="flex items-center justify-between">
            <span class="text-gray-400 text-sm">Mode</span>
            <span class="${m.bg} px-3 py-0.5 rounded-full text-xs font-bold tracking-wide">
              ${m.text}
            </span>
          </div>

          <!-- Speed -->
          <div class="flex items-center justify-between">
            <span class="text-gray-400 text-sm">Speed</span>
            <span class="font-mono text-sm">
              ${Math.abs(status.linear_vel).toFixed(2)} m/s
            </span>
          </div>

          <!-- Turn rate -->
          <div class="flex items-center justify-between">
            <span class="text-gray-400 text-sm">Turn</span>
            <span class="font-mono text-sm">
              ${Math.abs(status.angular_vel).toFixed(2)} rad/s
            </span>
          </div>

          <!-- Mapping status -->
          <div class="flex items-center justify-between">
            <span class="text-gray-400 text-sm">Mapping</span>
            <span class=${
              status.mapping_active && !status.mapping_paused
                ? 'text-pb-green font-semibold text-sm pulse-green'
                : status.mapping_paused
                  ? 'text-yellow-400 font-semibold text-sm'
                  : 'text-gray-500 text-sm'
            }>
              ${status.mapping_active && !status.mapping_paused
                ? 'ACTIVE'
                : status.mapping_paused
                  ? 'PAUSED'
                  : 'OFF'}
            </span>
          </div>
        </div>
      </div>
    `;
  };
})();
