// Cliente del stream Live 3D (protocolo v1, ver live3d/protocol.py).
// Decodificación sin bloquear el hilo principal:
//   - zlib -> DecompressionStream('deflate') nativo del navegador
//   - JPEG -> createImageBitmap (decodificado fuera del hilo principal)

export const HEADER_SIZE = 32;
const DEPTH_CODEC = ['raw', 'zlib'];
const COLOR_CODEC = ['none', 'jpeg', 'raw'];
const HAS_INFLATE = typeof DecompressionStream !== 'undefined';

async function inflate(u8) {
  const stream = new Blob([u8]).stream().pipeThrough(new DecompressionStream('deflate'));
  return new Response(stream).arrayBuffer();
}

export function parseHeader(buf) {
  const dv = new DataView(buf);
  const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
  if (magic !== 'RSF1') throw new Error('magic inválido');
  return {
    seq: dv.getUint32(4, true),
    timestamp: dv.getFloat64(8, true),
    metaId: dv.getUint16(16, true),
    depthCodec: DEPTH_CODEC[dv.getUint8(18)],
    colorCodec: COLOR_CODEC[dv.getUint8(19)],
    depthLen: dv.getUint32(20, true),
    colorLen: dv.getUint32(24, true),
    cam: dv.getUint32(28, true) & 1,  // flags bit 0: cámara de origen (0 | 1)
  };
}

export class LiveStream {
  constructor(url, { color = 'jpeg' } = {}) {
    this.url = url;
    this.colorCodec = color;
    this.depthCodec = HAS_INFLATE ? 'zlib' : 'raw';
    this.meta = null;     // último meta recibido (compatibilidad)
    this.metas = {};      // meta_id -> meta (uno por cámara en modo dual)
    this.ws = null;
    this.paused = false;
    this.handlers = { meta: [], status: [], frame: [], state: [], error: [], source_changed: [] };
    this._retry = 0;
    this._lastSeq = [-1, -1];
    this.bytes = 0;
  }

  on(ev, fn) { (this.handlers[ev] ||= []).push(fn); return this; }
  _emit(ev, x) { for (const fn of this.handlers[ev] || []) fn(x); }

  connect() {
    this._emit('state', 'connecting');
    const ws = new WebSocket(this.url);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;
    ws.onopen = () => {
      this._retry = 0;
      this._emit('state', 'open');
      if (!this.paused) this._subscribe();
    };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') this._onText(ev.data);
      else this._onBinary(ev.data);
    };
    ws.onclose = () => {
      this._emit('state', 'closed');
      this.meta = null;
      this.metas = {};
      const delay = Math.min(5000, 500 * 2 ** this._retry++);
      setTimeout(() => this.connect(), delay);
    };
    ws.onerror = () => {};  // onclose se encarga del reintento
  }

  _send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }
  _subscribe() {
    this._lastSeq = [-1, -1];
    this._send({ type: 'subscribe', client: 'browser', depth: this.depthCodec, color: this.colorCodec, ack: true });
  }

  /** Confirma un cuadro: el servidor mantiene como mucho 2 sin confirmar. */
  ack() { this._send({ type: 'ack' }); }
  sendSettings(changes) { this._send({ type: 'settings', ...changes }); }

  setPaused(p) {
    this.paused = p;
    if (p) this._send({ type: 'unsubscribe' });
    else this._subscribe();
  }

  _onText(text) {
    let msg;
    try { msg = JSON.parse(text); } catch { return; }
    if (msg.type === 'meta') { this.meta = msg; this.metas[msg.meta_id] = msg; this._emit('meta', msg); }
    else if (msg.type === 'status') this._emit('status', msg);
    else if (msg.type === 'error') this._emit('error', msg.message);
    else if (msg.type === 'source_changed') this._emit('source_changed', msg);
  }

  async _onBinary(buf) {
    this.bytes += buf.byteLength;
    let h;
    try { h = parseHeader(buf); } catch { this.ack(); return; }
    const meta = this.metas[h.metaId];
    if (!meta || this.paused) { this.ack(); return; }

    try {
      let off = HEADER_SIZE;
      const depthPromise = h.depthCodec === 'zlib'
        ? inflate(new Uint8Array(buf, off, h.depthLen)).then((ab) => new Uint16Array(ab))
        : Promise.resolve(new Uint16Array(buf.slice(off, off + h.depthLen)));
      off += h.depthLen;
      const colorBlob = h.colorCodec === 'jpeg' && h.colorLen > 0
        ? new Blob([new Uint8Array(buf, off, h.colorLen)], { type: 'image/jpeg' }) : null;
      const colorPromise = colorBlob
        ? createImageBitmap(colorBlob, { colorSpaceConversion: 'none', premultiplyAlpha: 'none' })
            .catch(() => createImageBitmap(colorBlob))
        : Promise.resolve(null);

      const [depth, color] = await Promise.all([depthPromise, colorPromise]);
      // Llegó tarde (otro cuadro más nuevo ya se decodificó) o cambió el meta
      if (h.seq <= this._lastSeq[h.cam] || this.metas[h.metaId] !== meta) {
        if (color) color.close();
        this.ack();
        return;
      }
      this._lastSeq[h.cam] = h.seq;
      this._emit('frame', { ...h, meta, depth, color, bytes: buf.byteLength });
    } catch (e) {
      this.ack();
      this._emit('error', `Error decodificando cuadro: ${e.message}`);
    }
  }
}
