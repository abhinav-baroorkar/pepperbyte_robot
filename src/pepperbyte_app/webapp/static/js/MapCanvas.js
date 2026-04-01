// MapCanvas.js — HTML5 Canvas occupancy grid renderer with robot + LiDAR overlay.
// Pan with mouse drag, zoom with scroll wheel, click to set nav goal.
(function () {
  const { useRef, useEffect, useCallback } = React;
  const html = window.html;

  // -----------------------------------------------------------------------
  // Colour palette for occupancy cells
  // -----------------------------------------------------------------------
  function cellColour(val) {
    if (val === 255) return [74, 74, 74];       // unknown → dark grey
    if (val === 0)   return [255, 255, 255];     // free    → white
    if (val >= 100)  return [0, 0, 0];           // wall    → black
    const s = 255 - Math.floor(val * 2.55);      // gradient
    return [s, s, s];
  }

  // -----------------------------------------------------------------------
  // Component
  // -----------------------------------------------------------------------
  window.MapCanvas = function MapCanvas(props) {
    const { mapData, robotPose, scanPoints, mode, onGoalClick } = props;
    const canvasRef = useRef(null);

    // Persistent render state (survives re-renders without triggering them)
    const S = useRef({
      panX: 0, panY: 0, zoom: 1,
      dragging: false, dragStartX: 0, dragStartY: 0,
      mapImg: null,                    // off-screen canvas
      mW: 0, mH: 0,                   // map pixel dims
      mRes: 0.05, mOX: 0, mOY: 0,    // resolution, origin
      autoFitted: false,
      goalX: null, goalY: null,        // active navigation goal (world coords)
    });

    // --- rebuild map image when data arrives -----------------------------
    useEffect(() => {
      if (!mapData) return;
      const s = S.current;
      s.mW   = mapData.width;
      s.mH   = mapData.height;
      s.mRes = mapData.resolution;
      s.mOX  = mapData.origin_x;
      s.mOY  = mapData.origin_y;

      // decode base64
      const raw = atob(mapData.data);
      const bytes = new Uint8Array(raw.length);
      for (let i = 0; i < raw.length; i++) bytes[i] = raw.charCodeAt(i);

      // create off-screen image
      const oc = document.createElement('canvas');
      oc.width = s.mW; oc.height = s.mH;
      const octx = oc.getContext('2d');
      const img  = octx.createImageData(s.mW, s.mH);

      for (let row = 0; row < s.mH; row++) {
        for (let col = 0; col < s.mW; col++) {
          const gIdx = row * s.mW + col;
          // flip Y: grid row 0 = bottom of map → image row (H-1)
          const iRow = s.mH - 1 - row;
          const iIdx = (iRow * s.mW + col) * 4;
          const [r, g, b] = cellColour(bytes[gIdx]);
          img.data[iIdx]     = r;
          img.data[iIdx + 1] = g;
          img.data[iIdx + 2] = b;
          img.data[iIdx + 3] = 255;
        }
      }
      octx.putImageData(img, 0, 0);
      s.mapImg = oc;

      // auto-fit on first map
      if (!s.autoFitted) {
        s.autoFitted = true;
        autoFit();
      }
    }, [mapData]);

    // --- auto-fit: centre & scale map to canvas --------------------------
    function autoFit() {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const s = S.current;
      if (!s.mW || !s.mH) return;
      const rect = canvas.getBoundingClientRect();
      const scaleX = rect.width  / s.mW;
      const scaleY = rect.height / s.mH;
      s.zoom = Math.min(scaleX, scaleY) * 0.9;
      s.panX = (rect.width  - s.mW * s.zoom) / 2;
      s.panY = (rect.height - s.mH * s.zoom) / 2;
    }

    // --- render loop (runs via rAF) --------------------------------------
    useEffect(() => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      let animId;

      function draw() {
        const s = S.current;
        const dpr  = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width  = rect.width  * dpr;
        canvas.height = rect.height * dpr;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

        // background
        ctx.fillStyle = '#2d2d44';
        ctx.fillRect(0, 0, rect.width, rect.height);

        ctx.save();
        ctx.translate(s.panX, s.panY);
        ctx.scale(s.zoom, s.zoom);

        // map image
        if (s.mapImg) ctx.drawImage(s.mapImg, 0, 0);

        // helper: world → image-pixel coords
        function w2i(wx, wy) {
          return [
            (wx - s.mOX) / s.mRes,
            s.mH - (wy - s.mOY) / s.mRes,
          ];
        }

        // LiDAR dots (red)
        if (scanPoints && scanPoints.length && robotPose) {
          ctx.fillStyle = '#ff4444';
          const ct = Math.cos(robotPose.theta);
          const st = Math.sin(robotPose.theta);
          for (const p of scanPoints) {
            const wx = robotPose.x + p[0] * ct - p[1] * st;
            const wy = robotPose.y + p[0] * st + p[1] * ct;
            const [ix, iy] = w2i(wx, wy);
            ctx.beginPath();
            ctx.arc(ix, iy, Math.max(1, 1.5 / s.zoom), 0, Math.PI * 2);
            ctx.fill();
          }
        }

        // robot arrow (green)
        if (robotPose && s.mRes > 0) {
          const [rx, ry] = w2i(robotPose.x, robotPose.y);
          const sz = 0.15 / s.mRes; // ~robot radius in pixels
          ctx.save();
          ctx.translate(rx, ry);
          ctx.rotate(-robotPose.theta); // negate for canvas Y-down
          ctx.fillStyle = '#00ff88';
          ctx.beginPath();
          ctx.moveTo(sz * 1.5, 0);
          ctx.lineTo(-sz, -sz * 0.8);
          ctx.lineTo(-sz * 0.4, 0);
          ctx.lineTo(-sz, sz * 0.8);
          ctx.closePath();
          ctx.fill();
          ctx.strokeStyle = '#fff';
          ctx.lineWidth = Math.max(0.5, 1 / s.zoom);
          ctx.stroke();
          ctx.restore();
        }

        // goal marker (cyan arrow + pulsing ring)
        if (s.goalX !== null && s.goalY !== null && s.mRes > 0) {
          const [gx, gy] = w2i(s.goalX, s.goalY);
          const sz = 0.18 / s.mRes;

          // pulsing ring
          const pulse = 0.6 + 0.4 * Math.sin(Date.now() / 300);
          ctx.save();
          ctx.translate(gx, gy);
          ctx.beginPath();
          ctx.arc(0, 0, sz * 1.8, 0, Math.PI * 2);
          ctx.strokeStyle = 'rgba(0, 212, 255, ' + pulse.toFixed(2) + ')';
          ctx.lineWidth = Math.max(1, 2 / s.zoom);
          ctx.stroke();

          // filled arrow pointing up (north)
          ctx.fillStyle = '#00d4ff';
          ctx.beginPath();
          ctx.moveTo(0, -sz * 1.5);
          ctx.lineTo(-sz * 0.8, sz * 0.5);
          ctx.lineTo(0, 0);
          ctx.lineTo(sz * 0.8, sz * 0.5);
          ctx.closePath();
          ctx.fill();
          ctx.strokeStyle = '#fff';
          ctx.lineWidth = Math.max(0.5, 1 / s.zoom);
          ctx.stroke();

          // "GOAL" label
          ctx.fillStyle = '#00d4ff';
          ctx.font = Math.max(8, 12 / s.zoom) + 'px system-ui';
          ctx.textAlign = 'center';
          ctx.fillText('GOAL', 0, sz * 2.8);
          ctx.textAlign = 'start';

          ctx.restore();
        }

        ctx.restore();

        // Zoom indicator
        ctx.fillStyle = 'rgba(255,255,255,0.4)';
        ctx.font = '12px system-ui';
        ctx.fillText('Zoom: ' + s.zoom.toFixed(1) + 'x', 8, rect.height - 8);

        // Autonomous mode hint
        if (mode === 'autonomous') {
          ctx.fillStyle = 'rgba(0,212,255,0.6)';
          ctx.font = '14px system-ui';
          ctx.textAlign = 'center';
          ctx.fillText('Click on map to set navigation goal', rect.width / 2, 24);
          ctx.textAlign = 'start';
        }

        animId = requestAnimationFrame(draw);
      }

      draw();
      return () => cancelAnimationFrame(animId);
    }, [robotPose, scanPoints, mode]);

    // --- mouse handlers: pan, zoom, click --------------------------------
    const onMouseDown = useCallback((e) => {
      if (e.button !== 0) return;
      const s = S.current;
      s.dragging = true;
      s.dragStartX = e.clientX - s.panX;
      s.dragStartY = e.clientY - s.panY;
    }, []);

    const onMouseMove = useCallback((e) => {
      const s = S.current;
      if (!s.dragging) return;
      s.panX = e.clientX - s.dragStartX;
      s.panY = e.clientY - s.dragStartY;
    }, []);

    const onMouseUp = useCallback((e) => {
      const s = S.current;
      // detect click (not drag) in autonomous mode
      if (s.dragging && mode === 'autonomous') {
        const dx = Math.abs(e.clientX - (s.dragStartX + s.panX));
        const dy = Math.abs(e.clientY - (s.dragStartY + s.panY));
        if (dx < 3 && dy < 3 && onGoalClick && s.mRes > 0) {
          const canvas = canvasRef.current;
          const rect = canvas.getBoundingClientRect();
          const cx = e.clientX - rect.left;
          const cy = e.clientY - rect.top;
          // canvas → image pixel
          const ix = (cx - s.panX) / s.zoom;
          const iy = (cy - s.panY) / s.zoom;
          // image pixel → world
          const wx = ix * s.mRes + s.mOX;
          const wy = (s.mH - iy) * s.mRes + s.mOY;
          s.goalX = wx;
          s.goalY = wy;
          onGoalClick(wx, wy, 0.0);
        }
      }
      s.dragging = false;
    }, [mode, onGoalClick]);

    const onWheel = useCallback((e) => {
      e.preventDefault();
      const s = S.current;
      const canvas = canvasRef.current;
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
      const newZoom = Math.max(0.1, Math.min(50, s.zoom * factor));
      // zoom towards mouse position
      s.panX = mx - (mx - s.panX) * (newZoom / s.zoom);
      s.panY = my - (my - s.panY) * (newZoom / s.zoom);
      s.zoom = newZoom;
    }, []);

    // fit button handler exposed via ref
    const handleFit = useCallback(() => autoFit(), []);

    // --- zoom buttons ----------------------------------------------------
    const zoomIn  = useCallback(() => { S.current.zoom = Math.min(50, S.current.zoom * 1.3); }, []);
    const zoomOut = useCallback(() => { S.current.zoom = Math.max(0.1, S.current.zoom / 1.3); }, []);

    return html`
      <div class="relative w-full h-full">
        <canvas
          ref=${canvasRef}
          class=${mode === 'autonomous' ? 'canvas-crosshair' : 'canvas-grab'}
          style=${{ width: '100%', height: '100%', display: 'block' }}
          onMouseDown=${onMouseDown}
          onMouseMove=${onMouseMove}
          onMouseUp=${onMouseUp}
          onMouseLeave=${() => { S.current.dragging = false; }}
          onWheel=${onWheel}
        />
        <div class="absolute bottom-3 right-3 flex gap-1">
          <button onClick=${handleFit}
            class="bg-pb-panel/80 hover:bg-pb-accent/30 px-3 py-1 rounded text-xs border border-gray-600">
            Fit
          </button>
          <button onClick=${zoomIn}
            class="bg-pb-panel/80 hover:bg-pb-accent/30 px-3 py-1 rounded text-xs border border-gray-600">
            +
          </button>
          <button onClick=${zoomOut}
            class="bg-pb-panel/80 hover:bg-pb-accent/30 px-3 py-1 rounded text-xs border border-gray-600">
            -
          </button>
        </div>
      </div>
    `;
  };
})();
