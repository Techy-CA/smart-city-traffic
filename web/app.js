(function () {
  "use strict";

  const D = window.SIM_DATA;
  if (!D) {
    document.getElementById("nodata").hidden = false;
    document.getElementById("app").hidden = true;
    return;
  }

  const N = D.nodes.length;
  const ALGOS = ["dijkstra", "astar", "bfs", "bellman", "dijkstra_array"];
  const ALGO_NAME = {
    dijkstra: "Dijkstra (heap)", astar: "A* search", bfs: "BFS (min hops)",
    bellman: "Bellman-Ford", dijkstra_array: "Dijkstra (array)",
  };
  const LIGHT = { 1: "good", 2: "warning", 3: "critical", 4: "preempt" };

  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue("--" + name).trim();
  const $ = (id) => document.getElementById(id);
  const fmtTime = (s) => Math.floor(s / 60) + ":" + String(Math.floor(s % 60)).padStart(2, "0");
  const fmt = (v, d = 0) => Number(v).toLocaleString(undefined, { minimumFractionDigits: d, maximumFractionDigits: d });

  const tip = $("tooltip");
  function showTip(evt, html) {
    tip.innerHTML = html;
    tip.hidden = false;
    const pad = 14, r = tip.getBoundingClientRect();
    let x = evt.clientX + pad, y = evt.clientY + pad;
    if (x + r.width > innerWidth - 8) x = evt.clientX - r.width - pad;
    if (y + r.height > innerHeight - 8) y = evt.clientY - r.height - pad;
    tip.style.left = x + "px";
    tip.style.top = y + "px";
  }
  const hideTip = () => (tip.hidden = true);

  const xs = D.nodes.map((n) => n.x), ys = D.nodes.map((n) => n.y);
  const W = { x0: Math.min(...xs) - 60, y0: Math.min(...ys) - 50, w: 980, h: 680 };

  class MapView {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.zoom = 1; this.cx = null; this.cy = null;
      new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
      this.resize();
    }
    resize() {
      const r = this.canvas.parentElement.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      this.canvas.width = Math.max(1, r.width * dpr);
      this.canvas.height = Math.max(1, r.height * dpr);
      this.dpr = dpr;
      this.cssW = r.width;
      this.onResize && this.onResize();
    }
    get k() { return (this.cssW / W.w) * this.zoom; }
    sx(x) {
      const cx = this.cx == null ? W.x0 + W.w / 2 : this.cx;
      return (x - cx) * this.k + this.cssW / 2;
    }
    sy(y) {
      const cy = this.cy == null ? W.y0 + W.h / 2 : this.cy;
      return (y - cy) * this.k + (this.cssW * W.h / W.w) / 2;
    }
    begin() {
      const c = this.ctx;
      c.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      c.clearRect(0, 0, this.cssW, this.cssW);
      return c;
    }
    edgeGeom(e, lane = 1) {
      const ed = D.edges[e], a = D.nodes[ed.from], b = D.nodes[ed.to];
      const ax = this.sx(a.x), ay = this.sy(a.y), bx = this.sx(b.x), by = this.sy(b.y);
      const len = Math.hypot(bx - ax, by - ay) || 1;
      const dx = (bx - ax) / len, dy = (by - ay) / len;
      const nx = dy, ny = -dx;
      const off = (ed.lanes * 1.4 + 1.5) * Math.min(1.6, this.k) * 0.9 * lane;
      const trim = 11 * Math.min(1.6, this.k);
      return {
        x1: ax + dx * trim + nx * off, y1: ay + dy * trim + ny * off,
        x2: bx - dx * trim + nx * off, y2: by - dy * trim + ny * off,
        dx, dy, nx, ny, len: len - 2 * trim,
      };
    }
    nodeAt(px, py) {
      let best = -1, bd = 18;
      D.nodes.forEach((n, i) => {
        const d = Math.hypot(this.sx(n.x) - px, this.sy(n.y) - py);
        if (d < bd) { bd = d; best = i; }
      });
      return best;
    }
    drawRoadBeds() {
      const c = this.ctx, s = Math.min(1.6, this.k);
      c.lineCap = "round";
      D.edges.forEach((ed) => {
        if (ed.rev >= 0 && ed.rev < ed.id) return;
        const a = D.nodes[ed.from], b = D.nodes[ed.to];
        c.strokeStyle = css("road");
        c.lineWidth = (ed.lanes * 2.8 + 5) * s;
        c.beginPath(); c.moveTo(this.sx(a.x), this.sy(a.y)); c.lineTo(this.sx(b.x), this.sy(b.y)); c.stroke();
      });
    }
    drawNodes(opts = {}) {
      const c = this.ctx, s = Math.min(1.6, this.k);
      D.nodes.forEach((n, i) => {
        const x = this.sx(n.x), y = this.sy(n.y);
        const r = (n.type === "junction" ? 6 : 8) * s;
        const fill = opts.fill ? opts.fill(i) : null;
        c.beginPath(); c.arc(x, y, r, 0, Math.PI * 2);
        c.fillStyle = fill || css("node"); c.fill();
        c.lineWidth = opts.ring && opts.ring(i) ? 3 : 1.5;
        c.strokeStyle = opts.ring && opts.ring(i) ? opts.ring(i) : css("axis"); c.stroke();
        if (n.type === "hospital") {
          c.fillStyle = css("critical");
          c.fillRect(x - r * 0.55, y - r * 0.18, r * 1.1, r * 0.36);
          c.fillRect(x - r * 0.18, y - r * 0.55, r * 0.36, r * 1.1);
        }
        if (opts.badge && opts.badge(i) != null) {
          c.fillStyle = "#fff"; c.font = `600 ${Math.round(9 * s)}px system-ui`;
          c.textAlign = "center"; c.textBaseline = "middle";
          c.fillText(opts.badge(i), x, y + 0.5);
        }
        if (this.k > 0.55) {
          c.font = `${n.type === "junction" ? 400 : 600} ${Math.round(10.5 * Math.min(1.3, s))}px system-ui`;
          c.textAlign = "center"; c.textBaseline = "top";
          c.lineWidth = 3; c.strokeStyle = css("surface-2");
          c.strokeText(n.name, x, y + r + 4);
          c.fillStyle = n.type === "junction" ? css("ink-2") : css("ink");
          c.fillText(n.name, x, y + r + 4);
        }
      });
    }
  }

  document.querySelectorAll(".tabs button").forEach((b) =>
    b.addEventListener("click", () => {
      document.querySelectorAll(".tabs button").forEach((x) => x.classList.toggle("active", x === b));
      document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("active", t.id === "tab-" + b.dataset.tab));
      if (b.dataset.tab === "stats") renderAnalytics();
      sim.view.resize(); route.view.resize();
    })
  );
  $("theme").addEventListener("click", () => {
    const root = document.documentElement;
    const dark = root.dataset.theme ? root.dataset.theme === "dark" : matchMedia("(prefers-color-scheme: dark)").matches;
    root.dataset.theme = dark ? "light" : "dark";
    try { localStorage.setItem("sc-theme", root.dataset.theme); } catch (e) {}
    route.draw(); renderAnalytics(true);
  });
  try { const t = localStorage.getItem("sc-theme"); if (t) document.documentElement.dataset.theme = t; } catch (e) {}

  function segmented(id, onPick) {
    const root = $(id);
    root.addEventListener("click", (e) => {
      const b = e.target.closest("button");
      if (!b) return;
      root.querySelectorAll("button").forEach((x) => x.classList.toggle("active", x === b));
      onPick(b.dataset);
    });
  }

  const sim = {
    view: new MapView($("map-sim")),
    run: 1, t: 0, speed: 5, playing: true, last: null,
  };
  const signalSet = new Set(D.signals);
  $("s-signals").textContent = D.signals.length;

  function simData() { return D.sims[sim.run]; }
  function frameIndex(t) {
    const s = simData();
    return Math.min(s.frames.length - 1, Math.max(0, Math.floor(t / s.dt)));
  }

  function vehiclesAt(t) {
    const s = simData(), i = frameIndex(t), f = s.frames[i], g = s.frames[i + 1];
    const alpha = g ? (t - f.t) / (g.t - f.t) : 0;
    const next = new Map();
    if (g) for (let k = 0; k < g.v.length; k += 4) next.set(g.v[k], k);
    const out = [];
    for (let k = 0; k < f.v.length; k += 4) {
      const id = f.v[k], e = f.v[k + 1], flags = f.v[k + 3];
      let p = f.v[k + 2];
      const j = next.get(id);
      if (j !== undefined && g.v[j + 1] === e && g.v[j + 2] >= p) p += (g.v[j + 2] - p) * alpha;
      out.push({ id, e, p: p / 1000, queued: flags & 1, amb: flags & 2 });
    }
    return { frame: f, index: i, vehicles: out };
  }

  sim.view.onResize = () => drawSim();

  function drawSim() {
    const v = sim.view, s = simData();
    if (!s) return;
    const { frame, index, vehicles } = vehiclesAt(sim.t);
    const amb = vehicles.find((x) => x.amb);

    if ($("follow").checked && amb) {
      const ed = D.edges[amb.e], a = D.nodes[ed.from], b = D.nodes[ed.to];
      v.zoom = 2.1; v.cx = a.x + (b.x - a.x) * amb.p; v.cy = a.y + (b.y - a.y) * amb.p;
    } else { v.zoom = 1; v.cx = v.cy = null; }

    const c = v.begin(), s1 = Math.min(1.6, v.k);
    v.drawRoadBeds();

    const route = s.ambulance.route;
    if (sim.t >= s.ambulance.t && route.length) {
      c.save();
      c.strokeStyle = css("critical"); c.globalAlpha = 0.35; c.lineWidth = 16 * s1; c.lineCap = "round"; c.lineJoin = "round";
      c.beginPath();
      route.forEach((n, i) => (i ? c.lineTo : c.moveTo).call(c, v.sx(D.nodes[n].x), v.sy(D.nodes[n].y)));
      c.stroke(); c.restore();
    }

    c.lineCap = "butt";
    D.edges.forEach((ed, e) => {
      const g = v.edgeGeom(e), cong = frame.c[e] / 100;
      c.strokeStyle = cong < 0.5 ? css("good") : cong < 0.9 ? css("warning") : cong < 1.3 ? css("serious") : css("critical");
      c.globalAlpha = cong < 0.01 ? 0.35 : 0.95;
      c.lineWidth = (ed.lanes * 1.1 + 1) * s1;
      c.beginPath(); c.moveTo(g.x1, g.y1); c.lineTo(g.x2, g.y2); c.stroke();
    });
    c.globalAlpha = 1;

    v.drawNodes();

    D.edges.forEach((ed, e) => {
      if (!signalSet.has(ed.to)) return;
      const st = LIGHT[frame.l[e]];
      if (!st) return;
      const g = v.edgeGeom(e, 1.9);
      c.beginPath(); c.arc(g.x2, g.y2, (st === "preempt" ? 4.2 : 3) * s1, 0, Math.PI * 2);
      c.fillStyle = css(st); c.fill();
      if (st === "preempt") { c.lineWidth = 2; c.strokeStyle = "#fff"; c.stroke(); }
    });

    vehicles.forEach((veh) => {
      if (veh.amb) return;
      const g = v.edgeGeom(veh.e);
      const x = g.x1 + (g.x2 - g.x1) * veh.p, y = g.y1 + (g.y2 - g.y1) * veh.p;
      c.beginPath(); c.arc(x, y, 2.3 * s1, 0, Math.PI * 2);
      c.fillStyle = css("car"); c.fill();
    });
    if (amb) {
      const g = v.edgeGeom(amb.e);
      const x = g.x1 + (g.x2 - g.x1) * amb.p, y = g.y1 + (g.y2 - g.y1) * amb.p;
      const pulse = 0.5 + 0.5 * Math.sin(performance.now() / 120);
      c.beginPath(); c.arc(x, y, (9 + 5 * pulse) * s1, 0, Math.PI * 2);
      c.fillStyle = css("critical"); c.globalAlpha = 0.18 + 0.2 * pulse; c.fill(); c.globalAlpha = 1;
      c.beginPath(); c.arc(x, y, 6 * s1, 0, Math.PI * 2);
      c.fillStyle = "#fff"; c.fill(); c.lineWidth = 2 * s1; c.strokeStyle = css("critical"); c.stroke();
      c.fillStyle = css("critical");
      c.fillRect(x - 3.4 * s1, y - 1 * s1, 6.8 * s1, 2 * s1);
      c.fillRect(x - 1 * s1, y - 3.4 * s1, 2 * s1, 6.8 * s1);
    }

    updateSimPanel(frame, index, vehicles, amb);
  }

  function updateSimPanel(frame, index, vehicles, amb) {
    const s = simData();
    $("s-active").textContent = vehicles.length;
    $("s-queued").textContent = vehicles.filter((x) => x.queued).length;
    $("s-cong").textContent = Math.round(s.series.cong[index] * 100) + "%";
    $("clock").textContent = fmtTime(sim.t);
    $("scrubber").value = Math.round((sim.t / maxT()) * 1000);

    const a = s.ambulance, st = s.stats, names = a.route.map((n) => D.nodes[n].name);
    $("amb-route").textContent = "A* route: " + names.join(" → ");
    let status, prog = 0;
    if (sim.t < a.t) status = `Standby at ${D.nodes[a.src].name} — dispatch at ${fmtTime(a.t)}`;
    else if (amb) {
      const elapsed = sim.t - a.t;
      const leg = a.route.indexOf(D.edges[amb.e].from);
      prog = Math.min(1, (leg + amb.p) / (a.route.length - 1));
      status = `${amb.queued ? "⏸ Stopped at red light" : "▶ En route"} · ${fmtTime(elapsed)} elapsed`;
    } else if (st.ambArrived) { status = `✔ Arrived at ${D.nodes[a.dst].name} in ${fmtTime(st.ambTime)}`; prog = 1; }
    else status = "Still en route at end of simulation";
    $("amb-status").textContent = status;
    $("amb-progress").style.width = prog * 100 + "%";
  }

  function renderCompare() {
    const [a, b] = D.sims.map((s) => s.stats);
    const saved = a.ambTime - b.ambTime;
    $("compare").innerHTML = `
      <div class="row"><span>Without corridor</span><span class="v">${fmtTime(a.ambTime)}</span></div>
      <div class="row"><span>With corridor</span><span class="v">${fmtTime(b.ambTime)}</span></div>
      <div class="row"><span>Time saved</span><span class="v good-text">−${Math.round(saved)} s (${Math.round((100 * saved) / a.ambTime)}%)</span></div>
      <div class="row"><span>Time stopped at reds</span><span class="v">${Math.round(a.ambWait)} s → ${Math.round(b.ambWait)} s</span></div>
      <div class="row"><span>Signals pre-empted</span><span class="v">${b.preemptions}</span></div>`;
  }

  const maxT = () => simData().frames[simData().frames.length - 1].t;
  $("clock-end").textContent = fmtTime(maxT());

  function tick(now) {
    if (sim.last != null && sim.playing && $("tab-sim").classList.contains("active")) {
      sim.t += ((now - sim.last) / 1000) * sim.speed;
      if (sim.t > maxT()) sim.t = 0;
    }
    sim.last = now;
    if ($("tab-sim").classList.contains("active")) drawSim();
    requestAnimationFrame(tick);
  }

  segmented("run-select", (d) => { sim.run = +d.run; drawSim(); });
  segmented("speed", (d) => (sim.speed = +d.speed));
  $("play").addEventListener("click", () => {
    sim.playing = !sim.playing;
    $("play").textContent = sim.playing ? "Pause" : "Play";
  });
  $("scrubber").addEventListener("input", (e) => { sim.t = (e.target.value / 1000) * maxT(); drawSim(); });
  renderCompare();
  requestAnimationFrame(tick);

  const route = { view: new MapView($("map-route")), from: 13, to: 4, algo: "dijkstra", step: Infinity, timer: null };
  const planner = (algo, s, d) => D.planner[algo][s * N + d];

  ["r-from", "r-to"].forEach((id) => {
    $(id).innerHTML = D.nodes.map((n) => `<option value="${n.id}">${n.id}. ${n.name}</option>`).join("");
  });
  $("r-from").value = route.from; $("r-to").value = route.to;
  $("r-algo").innerHTML = ["dijkstra", "astar", "bfs", "bellman"]
    .map((k) => `<button data-algo="${k}" class="${k === route.algo ? "active" : ""}">${ALGO_NAME[k]}</button>`).join("");

  route.draw = function () {
    const v = route.view, c = v.begin(), s1 = Math.min(1.6, v.k);
    const r = route.from !== route.to ? planner(route.algo, route.from, route.to) : null;
    const color = css("a-" + route.algo);
    v.drawRoadBeds();

    const visits = r ? r[1] : [];
    const shown = Math.min(visits.length, route.step);
    const order = new Map();
    visits.slice(0, shown).forEach((n, i) => order.set(n, i + 1));
    const done = shown >= visits.length;

    if (r && done) {
      c.strokeStyle = color; c.lineWidth = 7 * s1; c.lineCap = "round"; c.lineJoin = "round";
      c.beginPath();
      r[0].forEach((n, i) => (i ? c.lineTo : c.moveTo).call(c, v.sx(D.nodes[n].x), v.sy(D.nodes[n].y)));
      c.stroke();
    }
    v.drawNodes({
      fill: (i) => (order.has(i) ? color : null),
      ring: (i) => (i === route.from || i === route.to ? css("ink") : null),
      badge: (i) => (order.has(i) ? order.get(i) : null),
    });
  };
  route.view.onResize = () => route.draw();

  function updateRoute() {
    route.draw();
    if (route.from === route.to) return;
    const r = planner(route.algo, route.from, route.to);
    const path = r[0];
    $("r-title").textContent = ALGO_NAME[route.algo];
    $("r-path").innerHTML = path.map((n) => `<span>${D.nodes[n].name}</span>`).join('<span class="arrow">→</span>');
    $("r-dist").textContent = r[2] >= 0 ? fmt(r[2] / 1000, 2) + " km" : "no route";
    $("r-hops").textContent = path.length;
    $("r-visited").textContent = r[3];
    $("r-relax").textContent = r[4];

    let rows = `<tr><th>Algorithm</th><th>km</th><th>Explored</th><th>Relaxed</th></tr>`;
    ["dijkstra", "astar", "bfs", "bellman"].forEach((k) => {
      const q = planner(k, route.from, route.to);
      rows += `<tr class="${k === route.algo ? "sel" : ""}"><td><i class="key" style="background:var(--a-${k})"></i>${ALGO_NAME[k]}</td>
        <td>${fmt(q[2] / 1000, 2)}</td><td>${q[3]}</td><td>${q[4]}</td></tr>`;
    });
    $("r-table").innerHTML = rows;
  }

  function animateSearch() {
    clearInterval(route.timer);
    if (route.from === route.to) return;
    const total = planner(route.algo, route.from, route.to)[1].length;
    route.step = 0;
    route.timer = setInterval(() => {
      route.step++;
      route.draw();
      if (route.step >= total) clearInterval(route.timer);
    }, 260);
  }

  function setRoute(from, to) {
    clearInterval(route.timer);
    route.from = from; route.to = to; route.step = Infinity;
    $("r-from").value = from; $("r-to").value = to;
    updateRoute();
  }
  $("r-from").addEventListener("change", (e) => setRoute(+e.target.value, route.to));
  $("r-to").addEventListener("change", (e) => setRoute(route.from, +e.target.value));
  segmented("r-algo", (d) => { route.algo = d.algo; setRoute(route.from, route.to); });
  $("r-animate").addEventListener("click", animateSearch);

  let pickFrom = true;
  $("map-route").addEventListener("click", (e) => {
    const r = e.target.getBoundingClientRect();
    const n = route.view.nodeAt(e.clientX - r.left, e.clientY - r.top);
    if (n < 0) return;
    if (pickFrom) setRoute(n, route.to === n ? route.from : route.to);
    else if (n !== route.from) setRoute(route.from, n);
    pickFrom = !pickFrom;
  });
  $("map-route").addEventListener("mousemove", (e) => {
    const r = e.target.getBoundingClientRect();
    const n = route.view.nodeAt(e.clientX - r.left, e.clientY - r.top);
    e.target.style.cursor = n >= 0 ? "pointer" : "default";
    if (n >= 0) showTip(e, `<b>${D.nodes[n].name}</b>Node ${n} · ${D.nodes[n].type}<br>Click to set ${pickFrom ? "From" : "To"}`);
    else hideTip();
  });
  $("map-route").addEventListener("mouseleave", hideTip);
  updateRoute();

  const NS = "http://www.w3.org/2000/svg";
  function el(tag, attrs, parent) {
    const n = document.createElementNS(NS, tag);
    for (const k in attrs) n.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(n);
    return n;
  }
  function svgFor(container, w, h) {
    container.innerHTML = "";
    return el("svg", { viewBox: `0 0 ${w} ${h}`, role: "img" }, container);
  }
  function legend(container, items) {
    const div = document.createElement("div");
    div.className = "chart-legend";
    div.innerHTML = items.map(([name, color]) => `<span><i style="background:${color}"></i>${name}</span>`).join("");
    container.prepend(div);
  }
  function niceTicks(max, count = 4) {
    const raw = max / count, mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw);
    const ticks = [];
    for (let v = 0; v <= max + 1e-9; v += step) ticks.push(v);
    if (ticks[ticks.length - 1] < max) ticks.push(ticks[ticks.length - 1] + step);
    return ticks;
  }
  function barPath(x, y, w, h, r = 4) {
    r = Math.min(r, w / 2, h);
    return `M${x},${y + h}V${y + r}Q${x},${y} ${x + r},${y}H${x + w - r}Q${x + w},${y} ${x + w},${y + r}V${y + h}Z`;
  }

  function barChart(container, items, opts) {
    const w = 520, h = 250, m = { l: 44, r: 8, t: 22, b: 40 };
    const svg = svgFor(container, w, h);
    const max = Math.max(...items.map((d) => d.value));
    const ticks = niceTicks(max), top = ticks[ticks.length - 1];
    const ph = h - m.t - m.b, pw = w - m.l - m.r;
    const y = (v) => m.t + ph - (v / top) * ph;
    ticks.forEach((t) => {
      el("line", { x1: m.l, x2: w - m.r, y1: y(t), y2: y(t), class: t ? "grid" : "base" }, svg);
      el("text", { x: m.l - 8, y: y(t) + 4, "text-anchor": "end" }, svg).textContent = fmt(t, opts.decimals || 0);
    });
    const band = pw / items.length, bw = Math.min(56, band * 0.56);
    items.forEach((d, i) => {
      const x = m.l + band * i + (band - bw) / 2;
      el("path", { d: barPath(x, y(d.value), bw, y(0) - y(d.value)), fill: d.color }, svg);
      el("text", { x: x + bw / 2, y: y(d.value) - 6, "text-anchor": "middle", class: "val" }, svg).textContent = d.label;
      el("text", { x: x + bw / 2, y: h - m.b + 16, "text-anchor": "middle", class: "lbl" }, svg).textContent = d.short;
      const hit = el("rect", { x: m.l + band * i, y: m.t, width: band, height: ph, fill: "transparent" }, svg);
      hit.addEventListener("mousemove", (e) => showTip(e, d.tip));
      hit.addEventListener("mouseleave", hideTip);
    });
  }

  function lineChart(container, cfg) {
    const w = 520, h = 260, m = { l: 52, r: 12, t: 12, b: 34 };
    const svg = svgFor(container, w, h);
    const pw = w - m.l - m.r, ph = h - m.t - m.b;
    const all = cfg.series.flatMap((s) => s.values).filter((v) => v > 0);
    let y, ticks;
    if (cfg.log) {
      const lo = Math.pow(10, Math.floor(Math.log10(Math.min(...all))));
      const hi = Math.pow(10, Math.ceil(Math.log10(Math.max(...all))));
      y = (v) => m.t + ph - ((Math.log10(v) - Math.log10(lo)) / (Math.log10(hi) - Math.log10(lo))) * ph;
      ticks = [];
      for (let v = lo; v <= hi * 1.001; v *= 10) ticks.push(v);
    } else {
      ticks = niceTicks(Math.max(...all));
      const top = ticks[ticks.length - 1];
      y = (v) => m.t + ph - (v / top) * ph;
    }
    const fx = cfg.logX ? Math.log10 : (v) => v;
    const x0 = fx(cfg.x[0]), x1 = fx(cfg.x[cfg.x.length - 1]);
    const x = (v) => m.l + ((fx(v) - x0) / (x1 - x0)) * pw;

    ticks.forEach((t, i) => {
      el("line", { x1: m.l, x2: w - m.r, y1: y(t), y2: y(t), class: i ? "grid" : "base" }, svg);
      el("text", { x: m.l - 8, y: y(t) + 4, "text-anchor": "end" }, svg).textContent = cfg.yFmt(t);
    });
    cfg.xTicks.forEach((t) => {
      el("text", { x: x(t), y: h - m.b + 18, "text-anchor": "middle" }, svg).textContent = cfg.xFmt(t);
    });
    (cfg.markers || []).forEach((mk) => {
      el("line", { x1: x(mk.x), x2: x(mk.x), y1: m.t, y2: m.t + ph, stroke: mk.color, "stroke-dasharray": "4 4", "stroke-width": 1.5 }, svg);
      const right = x(mk.x) > m.l + pw * 0.6;
      el("text", { x: x(mk.x) + (right ? -4 : 4), y: m.t + ph - 8 - (mk.row || 0) * 15, class: "lbl",
        "text-anchor": right ? "end" : "start" }, svg).textContent = mk.label;
    });
    cfg.series.forEach((s) => {
      const d = s.values.map((v, i) => `${i ? "L" : "M"}${x(cfg.x[i]).toFixed(1)},${y(Math.max(v, cfg.log ? 1e-9 : 0)).toFixed(1)}`).join("");
      el("path", { d, fill: "none", stroke: s.color, "stroke-width": 2, "stroke-linejoin": "round" }, svg);
      if (cfg.dots) s.values.forEach((v, i) => el("circle", { cx: x(cfg.x[i]), cy: y(v), r: 4, fill: s.color, stroke: css("surface"), "stroke-width": 2 }, svg));
    });

    const cross = el("line", { y1: m.t, y2: m.t + ph, stroke: css("axis"), "stroke-width": 1, visibility: "hidden" }, svg);
    const hit = el("rect", { x: m.l, y: m.t, width: pw, height: ph, fill: "transparent" }, svg);
    hit.addEventListener("mousemove", (e) => {
      const r = svg.getBoundingClientRect();
      const px = ((e.clientX - r.left) / r.width) * w;
      let best = 0;
      cfg.x.forEach((v, i) => { if (Math.abs(x(v) - px) < Math.abs(x(cfg.x[best]) - px)) best = i; });
      cross.setAttribute("x1", x(cfg.x[best])); cross.setAttribute("x2", x(cfg.x[best]));
      cross.setAttribute("visibility", "visible");
      showTip(e, `<b>${cfg.xLabel(cfg.x[best])}</b>` + cfg.series
        .map((s) => `<div><i class="key" style="display:inline-block;width:8px;height:8px;border-radius:2px;margin-right:6px;background:${s.color}"></i>${s.name}: <b style="display:inline">${cfg.tipFmt(s.values[best])}</b></div>`).join(""));
    });
    hit.addEventListener("mouseleave", () => { cross.setAttribute("visibility", "hidden"); hideTip(); });
    legend(container, cfg.series.map((s) => [s.name, s.color]));
  }

  let analyticsDrawn = false;
  function renderAnalytics(force) {
    if (analyticsDrawn && !force) return;
    if (!$("tab-stats").classList.contains("active") && !analyticsDrawn) return;
    analyticsDrawn = true;
    const [a, b] = D.sims.map((s) => s.stats);
    const dist = D.algorithms.distance;
    const dj = dist.find((x) => x.key === "dijkstra"), as = dist.find((x) => x.key === "astar");
    const big = D.bench[D.bench.length - 1];

    $("hero").innerHTML = `
      <div class="tile"><span class="label">Ambulance response, green corridor</span><span class="val">${fmtTime(b.ambTime)}</span>
        <span class="note"><span class="good-text">−${Math.round(a.ambTime - b.ambTime)} s</span> vs ${fmtTime(a.ambTime)} without</span></div>
      <div class="tile"><span class="label">A* nodes explored vs Dijkstra</span><span class="val">−${Math.round(100 - (100 * as.visited) / dj.visited)}%</span>
        <span class="note">${fmt(as.visited, 1)} vs ${fmt(dj.visited, 1)} per query, same optimal routes</span></div>
      <div class="tile"><span class="label">Heap vs array Dijkstra @ ${fmt(big.V)} nodes</span><span class="val">${fmt(big.us.dijkstra_array / big.us.dijkstra, 0)}×</span>
        <span class="note">faster with the priority queue</span></div>
      <div class="tile"><span class="label">Trips completed</span><span class="val">${fmt(b.done)}</span>
        <span class="note">of ${fmt(b.vehicles)} vehicles · avg trip ${fmtTime(b.avgTrip)}</span></div>`;

    barChart($("c-visited"), dist.map((d) => ({
      value: d.visited, color: css("a-" + d.key), label: fmt(d.visited, 1),
      short: d.name.replace(" (heap)", "").replace(" (array)", " arr").replace(" (min hops)", "").replace(" search", ""),
      tip: `<b>${d.name}</b>${fmt(d.visited, 1)} nodes explored<br>${fmt(d.relax, 1)} edges relaxed<br>${fmt(d.optimal, 0)}% optimal routes`,
    })), { decimals: 0 });

    lineChart($("c-scale"), {
      x: D.bench.map((r) => r.V), xTicks: D.bench.map((r) => r.V), log: true, logX: true, dots: true,
      xFmt: (v) => fmt(v), yFmt: (v) => (v >= 1000 ? fmt(v / 1000) + " ms" : fmt(v) + " µs"),
      xLabel: (v) => `${fmt(v)} intersections`, tipFmt: (v) => (v >= 1000 ? fmt(v / 1000, 2) + " ms" : fmt(v, 1) + " µs"),
      series: ALGOS.map((k) => ({ name: ALGO_NAME[k], color: css("a-" + k), values: D.bench.map((r) => r.us[k]) })),
    });

    const s0 = D.sims[0], s1 = D.sims[1];
    const times = s0.frames.map((f) => f.t);
    lineChart($("c-cong"), {
      x: times, xTicks: times.filter((t) => t % 120 === 0),
      xFmt: fmtTime, yFmt: (v) => Math.round(v * 100) + "%", xLabel: (t) => "t = " + fmtTime(t), tipFmt: (v) => fmt(v * 100, 1) + "%",
      markers: [
        { x: s0.ambulance.t, color: css("critical"), label: "dispatch" },
        { x: s0.ambulance.t + b.ambTime, color: css("run-1"), label: "arrives (corridor)", row: 1 },
        { x: s0.ambulance.t + a.ambTime, color: css("run-0"), label: "arrives (no corridor)", row: 2 },
      ],
      series: [
        { name: "Without corridor", color: css("run-0"), values: s0.series.cong },
        { name: "With corridor", color: css("run-1"), values: s1.series.cong },
      ],
    });

    barChart($("c-amb"), [
      { value: a.ambTime, color: css("run-0"), label: fmtTime(a.ambTime), short: "Without corridor",
        tip: `<b>Without corridor</b>${Math.round(a.ambTime)} s total<br>${Math.round(a.ambWait)} s stopped at red lights` },
      { value: b.ambTime, color: css("run-1"), label: fmtTime(b.ambTime), short: "With corridor",
        tip: `<b>With corridor</b>${Math.round(b.ambTime)} s total<br>${b.preemptions} signals pre-empted, 0 s at reds` },
    ], { decimals: 0 });

    renderAlgoTable("distance");
    let rows = `<tr><th>Intersections (V)</th><th>Roads (E)</th>${ALGOS.map((k) => `<th>${ALGO_NAME[k]}</th>`).join("")}</tr>`;
    D.bench.forEach((r) => {
      rows += `<tr><td>${fmt(r.V)}</td><td>${fmt(r.E)}</td>${ALGOS.map((k) => `<td>${fmt(r.us[k], 1)} µs<br><span class="muted small">${fmt(r.visited[k])} nodes</span></td>`).join("")}</tr>`;
    });
    $("t-bench").innerHTML = rows;
  }

  function renderAlgoTable(mode) {
    const rows = D.algorithms[mode];
    $("t-algos").innerHTML = `<tr><th>Algorithm</th><th>Complexity</th><th>Nodes explored</th><th>Edges relaxed</th><th>Heap ops</th><th>Time / query</th><th>Optimal</th><th>Extra cost</th></tr>` +
      rows.map((r) => `<tr><td><i class="key" style="background:var(--a-${r.key})"></i>${r.name}</td>
        <td>${{ dijkstra: "O((V+E) log V)", astar: "O((V+E) log V)", bfs: "O(V+E)", bellman: "O(V·E)", dijkstra_array: "O(V²)" }[r.key]}</td>
        <td>${fmt(r.visited, 1)}</td><td>${fmt(r.relax, 1)}</td><td>${fmt(r.pqOps, 1)}</td><td>${fmt(r.us, 2)} µs</td>
        <td>${fmt(r.optimal, 0)}%</td><td>${r.excess > 0 ? "+" + fmt(r.excess, 2) + "%" : "—"}</td></tr>`).join("");
  }
  segmented("t-mode", (d) => renderAlgoTable(d.mode));
})();
