/* field-library.js — 場地 / 路徑資料庫（前端，只負責資料與座標換算，不碰 DOM）
 * 座標系統與 field_library.py 完全一致：
 *   原點 (0,0)=本壘，X=本壘→一壘，Y=本壘→三壘，二壘=(間距,間距)，單位公尺。
 * 畫面上為了讓內野「菱形朝上」，在 worldToSvg() 內才旋轉 45°，資料本身不旋轉。
 */
const FieldLibrary = (() => {
  let cache = null;

  // 抓不到 field_library.json（例如用 file:// 開啟）時的內建備援
  const FALLBACK = {
    presets: {
      "成棒（國際規格）": { base_spacing_m: 27.43 },
      "青少棒": { base_spacing_m: 25.00 },
      "少棒": { base_spacing_m: 18.29 },
      "壘球": { base_spacing_m: 3.00 },
    },
    path_templates: {
      perimeter_full: { label: "整備路徑（全場）", waypoints: [
        { point: "home", line: true, drop: false },
        { point: "first", line: false, drop: false },
        { point: "second", line: false, drop: false },
        { point: "third", line: true, drop: false },
        { point: "home", line: false, drop: false }] },
      perimeter_half: { label: "整備路徑（半場）", waypoints: [
        { point: "home", line: true, drop: false },
        { point: "first", line: false, drop: false },
        { point: "second", line: false, drop: false }] },
      line: { label: "整備路徑（線性）", waypoints: [
        { point: "home", line: true, drop: false },
        { point: "first", line: false, drop: false }] },
      lay_bases: { label: "放壘包＋畫線（全場）", waypoints: [
        { point: "home", line: true, drop: true },
        { point: "first", line: false, drop: true },
        { point: "second", line: false, drop: true },
        { point: "third", line: true, drop: false },
        { point: "home", line: false, drop: false }] },
      L_bases: { label: "畫L線（全場）", waypoints: [
        { point: "home", line: true, drop: false },
        { point: "firstgo", line: false, drop: false },
        { point: "home", line: true, drop: false },
        { point: "thirdgo", line: false, drop: false },
        { point: "home", line: false, drop: false }] },
    },
  };

  async function load() {
    if (cache) return cache;
    try {
      const res = await fetch("field_library.json");
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      cache = await res.json();
    } catch (err) {
      console.warn("[FieldLibrary] 讀取 field_library.json 失敗，改用內建預設資料：", err);
      cache = FALLBACK;
    }
    return cache;
  }

  function getBaseCoords(s) {
    return { home: { x: 0, y: 0 }, first: { x: s, y: 0 }, third: { x: 0, y: s }, second: { x: s, y: s }, thirdgo: { x: 0, y: 2*s }, firstgo: { x: 2*s, y: 0 } };
  }

  async function getPreset(name) {
    const lib = await load();
    if (!lib.presets[name]) throw new Error(`找不到場地規格：${name}`);
    return lib.presets[name];
  }

  /** 展開路徑範本成航點（規則與 field_library.py 的 resolve_waypoints 相同）：
   *  字串 = 舊格式 (line=true, drop=false)；{point|xy, line, drop}。
   *  旗標寫在「出發的那個點」：line = 此點到下一個點的路上要畫線；drop = 到了下一個點要丟壘包。
   *  回傳值已換算成「到達」語意：line = 走到此點的路上要畫線，drop = 到此點要丟壘包（起點一律 false）。 */
  async function resolveWaypoints(pathName, spacingM) {
    const lib = await load();
    const t = lib.path_templates[pathName];
    if (!t) throw new Error(`找不到路徑範本：${pathName}`);
    const c = getBaseCoords(spacingM);
    const raw = [];
    const pts = t.waypoints.map((item, i) => {
      if (typeof item === "string") item = { point: item, line: true, drop: false };
      let name, x, y;
      if (item.xy) { [x, y] = item.xy; name = item.name || `P${i}`; }
      else {
        name = item.point;
        if (!c[name]) throw new Error(`路徑範本 ${pathName} 的第 ${i} 點 ${name} 不是壘包名稱`);
        ({ x, y } = c[name]);
      }
      raw.push([!!item.line, !!item.drop]);
      return { name, x, y, line: false, drop: false };
    });
    for (let i = 1; i < pts.length; i++) [pts[i].line, pts[i].drop] = raw[i - 1];
    return pts;
  }

  /** 展開路徑範本成線段（不帶進度；狀態由 app.js 依任務狀態決定）。 */
  async function getPath(pathName, spacingM) {
    const w = await resolveWaypoints(pathName, spacingM);
    const segs = [];
    for (let i = 0; i < w.length - 1; i++) {
      segs.push({ fromBase: w[i].name, toBase: w[i + 1].name,
                  from: { x: w[i].x, y: w[i].y }, to: { x: w[i + 1].x, y: w[i + 1].y },
                  line: w[i + 1].line, drop: w[i + 1].drop });
    }
    return segs;
  }

  /** 建立投影函式：場地座標 → SVG 像素。
   *  旋轉 45°（本壘在下、二壘在上），並依「所有要顯示的點」（四個壘包＋路徑上每一點，含自訂座標）的範圍
   *  自動縮放、置中，所以路徑點離壘包再遠也不會被切掉。 */
  function makeProjector(points, { viewW = 360, viewH = 320, margin = 50 } = {}) {
    const k = Math.SQRT1_2;
    const rot = points.map(p => ({ dx: (p.x - p.y) * k, dy: (p.x + p.y) * k }));
    const minX = Math.min(...rot.map(r => r.dx)), maxX = Math.max(...rot.map(r => r.dx));
    const minY = Math.min(...rot.map(r => r.dy)), maxY = Math.max(...rot.map(r => r.dy));
    const scale = Math.min((viewW - margin * 2) / Math.max(maxX - minX, 1e-6),
                           (viewH - margin * 2) / Math.max(maxY - minY, 1e-6));
    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    return (p) => ({ x: viewW / 2 + ((p.x - p.y) * k - cx) * scale,
                     y: viewH / 2 - ((p.x + p.y) * k - cy) * scale });
  }

  return { load, getBaseCoords, getPreset, resolveWaypoints, getPath, makeProjector };
})();
