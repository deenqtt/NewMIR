!(function () {
  function a(b, c, d) {
    function e(g, h) {
      if (!c[g]) {
        if (!b[g]) {
          var i = "function" == typeof require && require;
          if (!h && i) return i(g, !0);
          if (f) return f(g, !0);
          var j = new Error("Cannot find module '" + g + "'");
          throw ((j.code = "MODULE_NOT_FOUND"), j);
        }
        var k = (c[g] = { exports: {} });
        b[g][0].call(
          k.exports,
          function (a) {
            var c = b[g][1][a];
            return e(c || a);
          },
          k,
          k.exports,
          a,
          b,
          c,
          d
        );
      }
      return c[g].exports;
    }
    for (
      var f = "function" == typeof require && require, g = 0;
      g < d.length;
      g++
    )
      e(d[g]);
    return e;
  }
  return a;
})()(
  {
    1: [
      function (a, b, c) {
        !(function (a, c) {
          "use strict";
          function d(a) {
            function b(a) {
              for (var b = p.byteLength, c = r + a; b < c; ) b *= 2;
              if (b !== p.byteLength) {
                var d = q;
                (p = new ArrayBuffer(b)), (q = new DataView(p));
                for (var e = (r + 3) >> 2, f = 0; f < e; ++f)
                  q.setUint32(4 * f, d.getUint32(4 * f));
              }
              return (o = a), q;
            }
            function d() {
              r += o;
            }
            function e(a) {
              d(b(8).setFloat64(r, a));
            }
            function f(a) {
              d(b(1).setUint8(r, a));
            }
            function i(a) {
              for (var c = b(a.length), e = 0; e < a.length; ++e)
                c.setUint8(r + e, a[e]);
              d();
            }
            function j(a) {
              d(b(2).setUint16(r, a));
            }
            function k(a) {
              d(b(4).setUint32(r, a));
            }
            function l(a) {
              var c = a % g,
                e = (a - c) / g,
                f = b(8);
              f.setUint32(r, e), f.setUint32(r + 4, c), d();
            }
            function m(a, b) {
              b < 24
                ? f((a << 5) | b)
                : b < 256
                ? (f((a << 5) | 24), f(b))
                : b < 65536
                ? (f((a << 5) | 25), j(b))
                : b < 4294967296
                ? (f((a << 5) | 26), k(b))
                : (f((a << 5) | 27), l(b));
            }
            function n(a) {
              var b;
              if (a === !1) return f(244);
              if (a === !0) return f(245);
              if (null === a) return f(246);
              if (a === c) return f(247);
              switch (typeof a) {
                case "number":
                  if (Math.floor(a) === a) {
                    if (0 <= a && a <= h) return m(0, a);
                    if (-h <= a && a < 0) return m(1, -(a + 1));
                  }
                  return f(251), e(a);
                case "string":
                  var d = [];
                  for (b = 0; b < a.length; ++b) {
                    var g = a.charCodeAt(b);
                    g < 128
                      ? d.push(g)
                      : g < 2048
                      ? (d.push(192 | (g >> 6)), d.push(128 | (63 & g)))
                      : g < 55296
                      ? (d.push(224 | (g >> 12)),
                        d.push(128 | ((g >> 6) & 63)),
                        d.push(128 | (63 & g)))
                      : ((g = (1023 & g) << 10),
                        (g |= 1023 & a.charCodeAt(++b)),
                        (g += 65536),
                        d.push(240 | (g >> 18)),
                        d.push(128 | ((g >> 12) & 63)),
                        d.push(128 | ((g >> 6) & 63)),
                        d.push(128 | (63 & g)));
                  }
                  return m(3, d.length), i(d);
                default:
                  var j;
                  if (Array.isArray(a))
                    for (j = a.length, m(4, j), b = 0; b < j; ++b) n(a[b]);
                  else if (a instanceof Uint8Array) m(2, a.length), i(a);
                  else {
                    var k = Object.keys(a);
                    for (j = k.length, m(5, j), b = 0; b < j; ++b) {
                      var l = k[b];
                      n(l), n(a[l]);
                    }
                  }
              }
            }
            var o,
              p = new ArrayBuffer(256),
              q = new DataView(p),
              r = 0;
            if ((n(a), "slice" in p)) return p.slice(0, r);
            for (
              var s = new ArrayBuffer(r), t = new DataView(s), u = 0;
              u < r;
              ++u
            )
              t.setUint8(u, q.getUint8(u));
            return s;
          }
          function e(a, b, d) {
            function e(a, b) {
              return (v += b), a;
            }
            function h(b) {
              return e(new Uint8Array(a, v, b), b);
            }
            function i() {
              var a = new ArrayBuffer(4),
                b = new DataView(a),
                c = m(),
                d = 32768 & c,
                e = 31744 & c,
                g = 1023 & c;
              if (31744 === e) e = 261120;
              else if (0 !== e) e += 114688;
              else if (0 !== g) return g * f;
              return (
                b.setUint32(0, (d << 16) | (e << 13) | (g << 13)),
                b.getFloat32(0)
              );
            }
            function j() {
              return e(u.getFloat32(v), 4);
            }
            function k() {
              return e(u.getFloat64(v), 8);
            }
            function l() {
              return e(u.getUint8(v), 1);
            }
            function m() {
              return e(u.getUint16(v), 2);
            }
            function n() {
              return e(u.getUint32(v), 4);
            }
            function o() {
              return n() * g + n();
            }
            function p() {
              return 255 === u.getUint8(v) && ((v += 1), !0);
            }
            function q(a) {
              if (a < 24) return a;
              if (24 === a) return l();
              if (25 === a) return m();
              if (26 === a) return n();
              if (27 === a) return o();
              if (31 === a) return -1;
              throw "Invalid length encoding";
            }
            function r(a) {
              var b = l();
              if (255 === b) return -1;
              var c = q(31 & b);
              if (c < 0 || b >> 5 !== a)
                throw "Invalid indefinite length element";
              return c;
            }
            function s(a, b) {
              for (var c = 0; c < b; ++c) {
                var d = l();
                128 & d &&
                  (d < 224
                    ? ((d = ((31 & d) << 6) | (63 & l())), (b -= 1))
                    : d < 240
                    ? ((d = ((15 & d) << 12) | ((63 & l()) << 6) | (63 & l())),
                      (b -= 2))
                    : ((d =
                        ((15 & d) << 18) |
                        ((63 & l()) << 12) |
                        ((63 & l()) << 6) |
                        (63 & l())),
                      (b -= 3))),
                  d < 65536
                    ? a.push(d)
                    : ((d -= 65536),
                      a.push(55296 | (d >> 10)),
                      a.push(56320 | (1023 & d)));
              }
            }
            function t() {
              var a,
                e,
                f = l(),
                g = f >> 5,
                m = 31 & f;
              if (7 === g)
                switch (m) {
                  case 25:
                    return i();
                  case 26:
                    return j();
                  case 27:
                    return k();
                }
              if (((e = q(m)), e < 0 && (g < 2 || 6 < g)))
                throw "Invalid length";
              switch (g) {
                case 0:
                  return e;
                case 1:
                  return -1 - e;
                case 2:
                  if (e < 0) {
                    for (var n = [], o = 0; (e = r(g)) >= 0; )
                      (o += e), n.push(h(e));
                    var u = new Uint8Array(o),
                      v = 0;
                    for (a = 0; a < n.length; ++a)
                      u.set(n[a], v), (v += n[a].length);
                    return u;
                  }
                  return h(e);
                case 3:
                  var w = [];
                  if (e < 0) for (; (e = r(g)) >= 0; ) s(w, e);
                  else s(w, e);
                  return String.fromCharCode.apply(null, w);
                case 4:
                  var x;
                  if (e < 0) for (x = []; !p(); ) x.push(t());
                  else for (x = new Array(e), a = 0; a < e; ++a) x[a] = t();
                  return x;
                case 5:
                  var y = {};
                  for (a = 0; a < e || (e < 0 && !p()); ++a) {
                    var z = t();
                    y[z] = t();
                  }
                  return y;
                case 6:
                  return b(t(), e);
                case 7:
                  switch (e) {
                    case 20:
                      return !1;
                    case 21:
                      return !0;
                    case 22:
                      return null;
                    case 23:
                      return c;
                    default:
                      return d(e);
                  }
              }
            }
            var u = new DataView(a),
              v = 0;
            "function" != typeof b &&
              (b = function (a) {
                return a;
              }),
              "function" != typeof d &&
                (d = function () {
                  return c;
                });
            var w = t();
            if (v !== a.byteLength) throw "Remaining bytes";
            return w;
          }
          var f = Math.pow(2, -24),
            g = Math.pow(2, 32),
            h = Math.pow(2, 53),
            i = { encode: d, decode: e };
          "function" == typeof define && define.amd
            ? define("cbor/cbor", i)
            : "undefined" != typeof b && b.exports
            ? (b.exports = i)
            : a.CBOR || (a.CBOR = i);
        })(this);
      },
      {},
    ],
    2: [
      function (a, b, c) {
        (function (a) {
          !(function (d) {
            function e() {
              (this._events = {}), this._conf && f.call(this, this._conf);
            }
            function f(a) {
              a
                ? ((this._conf = a),
                  a.delimiter && (this.delimiter = a.delimiter),
                  (this._maxListeners =
                    a.maxListeners !== d ? a.maxListeners : l),
                  a.wildcard && (this.wildcard = a.wildcard),
                  a.newListener && (this.newListener = a.newListener),
                  a.verboseMemoryLeak &&
                    (this.verboseMemoryLeak = a.verboseMemoryLeak),
                  this.wildcard && (this.listenerTree = {}))
                : (this._maxListeners = l);
            }
            function g(b, c) {
              var d =
                "(node) warning: possible EventEmitter memory leak detected. " +
                b +
                " listeners added. Use emitter.setMaxListeners() to increase limit.";
              if (
                (this.verboseMemoryLeak && (d += " Event name: " + c + "."),
                "undefined" != typeof a && a.emitWarning)
              ) {
                var e = new Error(d);
                (e.name = "MaxListenersExceededWarning"),
                  (e.emitter = this),
                  (e.count = b),
                  a.emitWarning(e);
              } else console.error(d), console.trace && console.trace();
            }
            function h(a) {
              (this._events = {}),
                (this.newListener = !1),
                (this.verboseMemoryLeak = !1),
                f.call(this, a);
            }
            function i(a, b, c, d) {
              if (!c) return [];
              var e,
                f,
                g,
                h,
                j,
                k,
                l,
                m = [],
                n = b.length,
                o = b[d],
                p = b[d + 1];
              if (d === n && c._listeners) {
                if ("function" == typeof c._listeners)
                  return a && a.push(c._listeners), [c];
                for (e = 0, f = c._listeners.length; e < f; e++)
                  a && a.push(c._listeners[e]);
                return [c];
              }
              if ("*" === o || "**" === o || c[o]) {
                if ("*" === o) {
                  for (g in c)
                    "_listeners" !== g &&
                      c.hasOwnProperty(g) &&
                      (m = m.concat(i(a, b, c[g], d + 1)));
                  return m;
                }
                if ("**" === o) {
                  (l = d + 1 === n || (d + 2 === n && "*" === p)),
                    l && c._listeners && (m = m.concat(i(a, b, c, n)));
                  for (g in c)
                    "_listeners" !== g &&
                      c.hasOwnProperty(g) &&
                      ("*" === g || "**" === g
                        ? (c[g]._listeners &&
                            !l &&
                            (m = m.concat(i(a, b, c[g], n))),
                          (m = m.concat(i(a, b, c[g], d))))
                        : (m =
                            g === p
                              ? m.concat(i(a, b, c[g], d + 2))
                              : m.concat(i(a, b, c[g], d))));
                  return m;
                }
                m = m.concat(i(a, b, c[o], d + 1));
              }
              if (((h = c["*"]), h && i(a, b, h, d + 1), (j = c["**"])))
                if (d < n) {
                  j._listeners && i(a, b, j, n);
                  for (g in j)
                    "_listeners" !== g &&
                      j.hasOwnProperty(g) &&
                      (g === p
                        ? i(a, b, j[g], d + 2)
                        : g === o
                        ? i(a, b, j[g], d + 1)
                        : ((k = {}),
                          (k[g] = j[g]),
                          i(a, b, { "**": k }, d + 1)));
                } else
                  j._listeners
                    ? i(a, b, j, n)
                    : j["*"] && j["*"]._listeners && i(a, b, j["*"], n);
              return m;
            }
            function j(a, b) {
              a = "string" == typeof a ? a.split(this.delimiter) : a.slice();
              for (var c = 0, e = a.length; c + 1 < e; c++)
                if ("**" === a[c] && "**" === a[c + 1]) return;
              for (var f = this.listenerTree, h = a.shift(); h !== d; ) {
                if ((f[h] || (f[h] = {}), (f = f[h]), 0 === a.length))
                  return (
                    f._listeners
                      ? ("function" == typeof f._listeners &&
                          (f._listeners = [f._listeners]),
                        f._listeners.push(b),
                        !f._listeners.warned &&
                          this._maxListeners > 0 &&
                          f._listeners.length > this._maxListeners &&
                          ((f._listeners.warned = !0),
                          g.call(this, f._listeners.length, h)))
                      : (f._listeners = b),
                    !0
                  );
                h = a.shift();
              }
              return !0;
            }
            var k = Array.isArray
                ? Array.isArray
                : function (a) {
                    return (
                      "[object Array]" === Object.prototype.toString.call(a)
                    );
                  },
              l = 10;
            (h.EventEmitter2 = h),
              (h.prototype.delimiter = "."),
              (h.prototype.setMaxListeners = function (a) {
                a !== d &&
                  ((this._maxListeners = a),
                  this._conf || (this._conf = {}),
                  (this._conf.maxListeners = a));
              }),
              (h.prototype.event = ""),
              (h.prototype.once = function (a, b) {
                return this._once(a, b, !1);
              }),
              (h.prototype.prependOnceListener = function (a, b) {
                return this._once(a, b, !0);
              }),
              (h.prototype._once = function (a, b, c) {
                return this._many(a, 1, b, c), this;
              }),
              (h.prototype.many = function (a, b, c) {
                return this._many(a, b, c, !1);
              }),
              (h.prototype.prependMany = function (a, b, c) {
                return this._many(a, b, c, !0);
              }),
              (h.prototype._many = function (a, b, c, d) {
                function e() {
                  return 0 === --b && f.off(a, e), c.apply(this, arguments);
                }
                var f = this;
                if ("function" != typeof c)
                  throw new Error("many only accepts instances of Function");
                return (e._origin = c), this._on(a, e, d), f;
              }),
              (h.prototype.emit = function () {
                this._events || e.call(this);
                var a = arguments[0];
                if (
                  "newListener" === a &&
                  !this.newListener &&
                  !this._events.newListener
                )
                  return !1;
                var b,
                  c,
                  d,
                  f,
                  g,
                  h = arguments.length;
                if (this._all && this._all.length) {
                  if (((g = this._all.slice()), h > 3))
                    for (b = new Array(h), f = 0; f < h; f++)
                      b[f] = arguments[f];
                  for (d = 0, c = g.length; d < c; d++)
                    switch (((this.event = a), h)) {
                      case 1:
                        g[d].call(this, a);
                        break;
                      case 2:
                        g[d].call(this, a, arguments[1]);
                        break;
                      case 3:
                        g[d].call(this, a, arguments[1], arguments[2]);
                        break;
                      default:
                        g[d].apply(this, b);
                    }
                }
                if (this.wildcard) {
                  g = [];
                  var j =
                    "string" == typeof a ? a.split(this.delimiter) : a.slice();
                  i.call(this, g, j, this.listenerTree, 0);
                } else {
                  if (((g = this._events[a]), "function" == typeof g)) {
                    switch (((this.event = a), h)) {
                      case 1:
                        g.call(this);
                        break;
                      case 2:
                        g.call(this, arguments[1]);
                        break;
                      case 3:
                        g.call(this, arguments[1], arguments[2]);
                        break;
                      default:
                        for (b = new Array(h - 1), f = 1; f < h; f++)
                          b[f - 1] = arguments[f];
                        g.apply(this, b);
                    }
                    return !0;
                  }
                  g && (g = g.slice());
                }
                if (g && g.length) {
                  if (h > 3)
                    for (b = new Array(h - 1), f = 1; f < h; f++)
                      b[f - 1] = arguments[f];
                  for (d = 0, c = g.length; d < c; d++)
                    switch (((this.event = a), h)) {
                      case 1:
                        g[d].call(this);
                        break;
                      case 2:
                        g[d].call(this, arguments[1]);
                        break;
                      case 3:
                        g[d].call(this, arguments[1], arguments[2]);
                        break;
                      default:
                        g[d].apply(this, b);
                    }
                  return !0;
                }
                if (!this._all && "error" === a)
                  throw arguments[1] instanceof Error
                    ? arguments[1]
                    : new Error("Uncaught, unspecified 'error' event.");
                return !!this._all;
              }),
              (h.prototype.emitAsync = function () {
                this._events || e.call(this);
                var a = arguments[0];
                if (
                  "newListener" === a &&
                  !this.newListener &&
                  !this._events.newListener
                )
                  return Promise.resolve([!1]);
                var b,
                  c,
                  d,
                  f,
                  g,
                  h = [],
                  j = arguments.length;
                if (this._all) {
                  if (j > 3)
                    for (b = new Array(j), f = 1; f < j; f++)
                      b[f] = arguments[f];
                  for (d = 0, c = this._all.length; d < c; d++)
                    switch (((this.event = a), j)) {
                      case 1:
                        h.push(this._all[d].call(this, a));
                        break;
                      case 2:
                        h.push(this._all[d].call(this, a, arguments[1]));
                        break;
                      case 3:
                        h.push(
                          this._all[d].call(this, a, arguments[1], arguments[2])
                        );
                        break;
                      default:
                        h.push(this._all[d].apply(this, b));
                    }
                }
                if (this.wildcard) {
                  g = [];
                  var k =
                    "string" == typeof a ? a.split(this.delimiter) : a.slice();
                  i.call(this, g, k, this.listenerTree, 0);
                } else g = this._events[a];
                if ("function" == typeof g)
                  switch (((this.event = a), j)) {
                    case 1:
                      h.push(g.call(this));
                      break;
                    case 2:
                      h.push(g.call(this, arguments[1]));
                      break;
                    case 3:
                      h.push(g.call(this, arguments[1], arguments[2]));
                      break;
                    default:
                      for (b = new Array(j - 1), f = 1; f < j; f++)
                        b[f - 1] = arguments[f];
                      h.push(g.apply(this, b));
                  }
                else if (g && g.length) {
                  if (((g = g.slice()), j > 3))
                    for (b = new Array(j - 1), f = 1; f < j; f++)
                      b[f - 1] = arguments[f];
                  for (d = 0, c = g.length; d < c; d++)
                    switch (((this.event = a), j)) {
                      case 1:
                        h.push(g[d].call(this));
                        break;
                      case 2:
                        h.push(g[d].call(this, arguments[1]));
                        break;
                      case 3:
                        h.push(g[d].call(this, arguments[1], arguments[2]));
                        break;
                      default:
                        h.push(g[d].apply(this, b));
                    }
                } else if (!this._all && "error" === a)
                  return arguments[1] instanceof Error
                    ? Promise.reject(arguments[1])
                    : Promise.reject("Uncaught, unspecified 'error' event.");
                return Promise.all(h);
              }),
              (h.prototype.on = function (a, b) {
                return this._on(a, b, !1);
              }),
              (h.prototype.prependListener = function (a, b) {
                return this._on(a, b, !0);
              }),
              (h.prototype.onAny = function (a) {
                return this._onAny(a, !1);
              }),
              (h.prototype.prependAny = function (a) {
                return this._onAny(a, !0);
              }),
              (h.prototype.addListener = h.prototype.on),
              (h.prototype._onAny = function (a, b) {
                if ("function" != typeof a)
                  throw new Error("onAny only accepts instances of Function");
                return (
                  this._all || (this._all = []),
                  b ? this._all.unshift(a) : this._all.push(a),
                  this
                );
              }),
              (h.prototype._on = function (a, b, c) {
                if ("function" == typeof a) return this._onAny(a, b), this;
                if ("function" != typeof b)
                  throw new Error("on only accepts instances of Function");
                return (
                  this._events || e.call(this),
                  this.emit("newListener", a, b),
                  this.wildcard
                    ? (j.call(this, a, b), this)
                    : (this._events[a]
                        ? ("function" == typeof this._events[a] &&
                            (this._events[a] = [this._events[a]]),
                          c
                            ? this._events[a].unshift(b)
                            : this._events[a].push(b),
                          !this._events[a].warned &&
                            this._maxListeners > 0 &&
                            this._events[a].length > this._maxListeners &&
                            ((this._events[a].warned = !0),
                            g.call(this, this._events[a].length, a)))
                        : (this._events[a] = b),
                      this)
                );
              }),
              (h.prototype.off = function (a, b) {
                function c(a) {
                  if (a !== d) {
                    var b = Object.keys(a);
                    for (var e in b) {
                      var f = b[e],
                        g = a[f];
                      g instanceof Function ||
                        "object" != typeof g ||
                        null === g ||
                        (Object.keys(g).length > 0 && c(a[f]),
                        0 === Object.keys(g).length && delete a[f]);
                    }
                  }
                }
                if ("function" != typeof b)
                  throw new Error(
                    "removeListener only takes instances of Function"
                  );
                var e,
                  f = [];
                if (this.wildcard) {
                  var g =
                    "string" == typeof a ? a.split(this.delimiter) : a.slice();
                  f = i.call(this, null, g, this.listenerTree, 0);
                } else {
                  if (!this._events[a]) return this;
                  (e = this._events[a]), f.push({ _listeners: e });
                }
                for (var h = 0; h < f.length; h++) {
                  var j = f[h];
                  if (((e = j._listeners), k(e))) {
                    for (var l = -1, m = 0, n = e.length; m < n; m++)
                      if (
                        e[m] === b ||
                        (e[m].listener && e[m].listener === b) ||
                        (e[m]._origin && e[m]._origin === b)
                      ) {
                        l = m;
                        break;
                      }
                    if (l < 0) continue;
                    return (
                      this.wildcard
                        ? j._listeners.splice(l, 1)
                        : this._events[a].splice(l, 1),
                      0 === e.length &&
                        (this.wildcard
                          ? delete j._listeners
                          : delete this._events[a]),
                      this.emit("removeListener", a, b),
                      this
                    );
                  }
                  (e === b ||
                    (e.listener && e.listener === b) ||
                    (e._origin && e._origin === b)) &&
                    (this.wildcard
                      ? delete j._listeners
                      : delete this._events[a],
                    this.emit("removeListener", a, b));
                }
                return c(this.listenerTree), this;
              }),
              (h.prototype.offAny = function (a) {
                var b,
                  c = 0,
                  d = 0;
                if (a && this._all && this._all.length > 0) {
                  for (b = this._all, c = 0, d = b.length; c < d; c++)
                    if (a === b[c])
                      return (
                        b.splice(c, 1), this.emit("removeListenerAny", a), this
                      );
                } else {
                  for (b = this._all, c = 0, d = b.length; c < d; c++)
                    this.emit("removeListenerAny", b[c]);
                  this._all = [];
                }
                return this;
              }),
              (h.prototype.removeListener = h.prototype.off),
              (h.prototype.removeAllListeners = function (a) {
                if (0 === arguments.length)
                  return !this._events || e.call(this), this;
                if (this.wildcard)
                  for (
                    var b =
                        "string" == typeof a
                          ? a.split(this.delimiter)
                          : a.slice(),
                      c = i.call(this, null, b, this.listenerTree, 0),
                      d = 0;
                    d < c.length;
                    d++
                  ) {
                    var f = c[d];
                    f._listeners = null;
                  }
                else this._events && (this._events[a] = null);
                return this;
              }),
              (h.prototype.listeners = function (a) {
                if (this.wildcard) {
                  var b = [],
                    c =
                      "string" == typeof a
                        ? a.split(this.delimiter)
                        : a.slice();
                  return i.call(this, b, c, this.listenerTree, 0), b;
                }
                return (
                  this._events || e.call(this),
                  this._events[a] || (this._events[a] = []),
                  k(this._events[a]) || (this._events[a] = [this._events[a]]),
                  this._events[a]
                );
              }),
              (h.prototype.eventNames = function () {
                return Object.keys(this._events);
              }),
              (h.prototype.listenerCount = function (a) {
                return this.listeners(a).length;
              }),
              (h.prototype.listenersAny = function () {
                return this._all ? this._all : [];
              }),
              "function" == typeof define && define.amd
                ? define(function () {
                    return h;
                  })
                : "object" == typeof c
                ? (b.exports = h)
                : (window.EventEmitter2 = h);
          })();
        }).call(this, a("_process"));
      },
      { _process: 4 },
    ],
    3: [
      function (a, b, c) {
        "use strict";
        function d(a) {
          if (null === a || void 0 === a)
            throw new TypeError(
              "Object.assign cannot be called with null or undefined"
            );
          return Object(a);
        }
        function e() {
          try {
            if (!Object.assign) return !1;
            var a = new String("abc");
            if (((a[5] = "de"), "5" === Object.getOwnPropertyNames(a)[0]))
              return !1;
            for (var b = {}, c = 0; c < 10; c++)
              b["_" + String.fromCharCode(c)] = c;
            var d = Object.getOwnPropertyNames(b).map(function (a) {
              return b[a];
            });
            if ("0123456789" !== d.join("")) return !1;
            var e = {};
            return (
              "abcdefghijklmnopqrst".split("").forEach(function (a) {
                e[a] = a;
              }),
              "abcdefghijklmnopqrst" ===
                Object.keys(Object.assign({}, e)).join("")
            );
          } catch (f) {
            return !1;
          }
        }
        var f = Object.getOwnPropertySymbols,
          g = Object.prototype.hasOwnProperty,
          h = Object.prototype.propertyIsEnumerable;
        b.exports = e()
          ? Object.assign
          : function (a, b) {
              for (var c, e, i = d(a), j = 1; j < arguments.length; j++) {
                c = Object(arguments[j]);
                for (var k in c) g.call(c, k) && (i[k] = c[k]);
                if (f) {
                  e = f(c);
                  for (var l = 0; l < e.length; l++)
                    h.call(c, e[l]) && (i[e[l]] = c[e[l]]);
                }
              }
              return i;
            };
      },
      {},
    ],
    4: [
      function (a, b, c) {
        function d() {
          throw new Error("setTimeout has not been defined");
        }
        function e() {
          throw new Error("clearTimeout has not been defined");
        }
        function f(a) {
          if (l === setTimeout) return setTimeout(a, 0);
          if ((l === d || !l) && setTimeout)
            return (l = setTimeout), setTimeout(a, 0);
          try {
            return l(a, 0);
          } catch (b) {
            try {
              return l.call(null, a, 0);
            } catch (b) {
              return l.call(this, a, 0);
            }
          }
        }
        function g(a) {
          if (m === clearTimeout) return clearTimeout(a);
          if ((m === e || !m) && clearTimeout)
            return (m = clearTimeout), clearTimeout(a);
          try {
            return m(a);
          } catch (b) {
            try {
              return m.call(null, a);
            } catch (b) {
              return m.call(this, a);
            }
          }
        }
        function h() {
          q &&
            o &&
            ((q = !1),
            o.length ? (p = o.concat(p)) : (r = -1),
            p.length && i());
        }
        function i() {
          if (!q) {
            var a = f(h);
            q = !0;
            for (var b = p.length; b; ) {
              for (o = p, p = []; ++r < b; ) o && o[r].run();
              (r = -1), (b = p.length);
            }
            (o = null), (q = !1), g(a);
          }
        }
        function j(a, b) {
          (this.fun = a), (this.array = b);
        }
        function k() {}
        var l,
          m,
          n = (b.exports = {});
        !(function () {
          try {
            l = "function" == typeof setTimeout ? setTimeout : d;
          } catch (a) {
            l = d;
          }
          try {
            m = "function" == typeof clearTimeout ? clearTimeout : e;
          } catch (a) {
            m = e;
          }
        })();
        var o,
          p = [],
          q = !1,
          r = -1;
        (n.nextTick = function (a) {
          var b = new Array(arguments.length - 1);
          if (arguments.length > 1)
            for (var c = 1; c < arguments.length; c++) b[c - 1] = arguments[c];
          p.push(new j(a, b)), 1 !== p.length || q || f(i);
        }),
          (j.prototype.run = function () {
            this.fun.apply(null, this.array);
          }),
          (n.title = "browser"),
          (n.browser = !0),
          (n.env = {}),
          (n.argv = []),
          (n.version = ""),
          (n.versions = {}),
          (n.on = k),
          (n.addListener = k),
          (n.once = k),
          (n.off = k),
          (n.removeListener = k),
          (n.removeAllListeners = k),
          (n.emit = k),
          (n.prependListener = k),
          (n.prependOnceListener = k),
          (n.listeners = function (a) {
            return [];
          }),
          (n.binding = function (a) {
            throw new Error("process.binding is not supported");
          }),
          (n.cwd = function () {
            return "/";
          }),
          (n.chdir = function (a) {
            throw new Error("process.chdir is not supported");
          }),
          (n.umask = function () {
            return 0;
          });
      },
      {},
    ],
    5: [
      function (a, b, c) {
        var d = arguments[3],
          e = arguments[4],
          f = arguments[5],
          g = JSON.stringify;
        b.exports = function (a, b) {
          function c(a) {
            q[a] = !0;
            for (var b in e[a][1]) {
              var d = e[a][1][b];
              q[d] || c(d);
            }
          }
          for (var h, i = Object.keys(f), j = 0, k = i.length; j < k; j++) {
            var l = i[j],
              m = f[l].exports;
            if (m === a || (m && m["default"] === a)) {
              h = l;
              break;
            }
          }
          if (!h) {
            h = Math.floor(Math.pow(16, 8) * Math.random()).toString(16);
            for (var n = {}, j = 0, k = i.length; j < k; j++) {
              var l = i[j];
              n[l] = l;
            }
            e[h] = ["function(require,module,exports){" + a + "(self); }", n];
          }
          var o = Math.floor(Math.pow(16, 8) * Math.random()).toString(16),
            p = {};
          (p[h] = h),
            (e[o] = [
              "function(require,module,exports){var f = require(" +
                g(h) +
                ");(f.default ? f.default : f)(self);}",
              p,
            ]);
          var q = {};
          c(o);
          var r =
              "(" +
              d +
              ")({" +
              Object.keys(q)
                .map(function (a) {
                  return g(a) + ":[" + e[a][0] + "," + g(e[a][1]) + "]";
                })
                .join(",") +
              "},{},[" +
              g(o) +
              "])",
            s = window.URL || window.webkitURL || window.mozURL || window.msURL,
            t = new Blob([r], { type: "text/javascript" });
          if (b && b.bare) return t;
          var u = s.createObjectURL(t),
            v = new Worker(u);
          return (v.objectURL = u), v;
        };
      },
      {},
    ],
    6: [
      function (a, b, c) {
        var d = this.ROSLIB || { REVISION: "1.1.0" },
          e = a("object-assign");
        e(d, a("./core")),
          e(d, a("./actionlib")),
          e(d, a("./math")),
          e(d, a("./tf")),
          e(d, a("./urdf")),
          (b.exports = d);
      },
      {
        "./actionlib": 12,
        "./core": 21,
        "./math": 26,
        "./tf": 29,
        "./urdf": 41,
        "object-assign": 3,
      },
    ],
    7: [
      function (a, b, c) {
        (function (b) {
          b.ROSLIB = a("./RosLib");
        }).call(
          this,
          "undefined" != typeof global
            ? global
            : "undefined" != typeof self
            ? self
            : "undefined" != typeof window
            ? window
            : {}
        );
      },
      { "./RosLib": 6 },
    ],
    8: [
      function (a, b, c) {
        function d(a) {
          var b = this;
          (a = a || {}),
            (this.ros = a.ros),
            (this.serverName = a.serverName),
            (this.actionName = a.actionName),
            (this.timeout = a.timeout),
            (this.omitFeedback = a.omitFeedback),
            (this.omitStatus = a.omitStatus),
            (this.omitResult = a.omitResult),
            (this.goals = {});
          var c = !1;
          (this.feedbackListener = new e({
            ros: this.ros,
            name: this.serverName + "/feedback",
            messageType: this.actionName + "Feedback",
          })),
            (this.statusListener = new e({
              ros: this.ros,
              name: this.serverName + "/status",
              messageType: "actionlib_msgs/GoalStatusArray",
            })),
            (this.resultListener = new e({
              ros: this.ros,
              name: this.serverName + "/result",
              messageType: this.actionName + "Result",
            })),
            (this.goalTopic = new e({
              ros: this.ros,
              name: this.serverName + "/goal",
              messageType: this.actionName + "Goal",
            })),
            (this.cancelTopic = new e({
              ros: this.ros,
              name: this.serverName + "/cancel",
              messageType: "actionlib_msgs/GoalID",
            })),
            this.goalTopic.advertise(),
            this.cancelTopic.advertise(),
            this.omitStatus ||
              this.statusListener.subscribe(function (a) {
                (c = !0),
                  a.status_list.forEach(function (a) {
                    var c = b.goals[a.goal_id.id];
                    c && c.emit("status", a);
                  });
              }),
            this.omitFeedback ||
              this.feedbackListener.subscribe(function (a) {
                var c = b.goals[a.status.goal_id.id];
                c &&
                  (c.emit("status", a.status), c.emit("feedback", a.feedback));
              }),
            this.omitResult ||
              this.resultListener.subscribe(function (a) {
                var c = b.goals[a.status.goal_id.id];
                c && (c.emit("status", a.status), c.emit("result", a.result));
              }),
            this.timeout &&
              setTimeout(function () {
                c || b.emit("timeout");
              }, this.timeout);
        }
        var e = a("../core/Topic"),
          f = a("../core/Message"),
          g = a("eventemitter2").EventEmitter2;
        (d.prototype.__proto__ = g.prototype),
          (d.prototype.cancel = function () {
            var a = new f();
            this.cancelTopic.publish(a);
          }),
          (d.prototype.dispose = function () {
            this.goalTopic.unadvertise(),
              this.cancelTopic.unadvertise(),
              this.omitStatus || this.statusListener.unsubscribe(),
              this.omitFeedback || this.feedbackListener.unsubscribe(),
              this.omitResult || this.resultListener.unsubscribe();
          }),
          (b.exports = d);
      },
      { "../core/Message": 13, "../core/Topic": 20, eventemitter2: 2 },
    ],
    9: [
      function (a, b, c) {
        function d(a) {
          var b = this;
          (a = a || {}),
            (this.ros = a.ros),
            (this.serverName = a.serverName),
            (this.actionName = a.actionName),
            (this.timeout = a.timeout),
            (this.omitFeedback = a.omitFeedback),
            (this.omitStatus = a.omitStatus),
            (this.omitResult = a.omitResult);
          var c = new e({
              ros: this.ros,
              name: this.serverName + "/goal",
              messageType: this.actionName + "Goal",
            }),
            d = new e({
              ros: this.ros,
              name: this.serverName + "/feedback",
              messageType: this.actionName + "Feedback",
            }),
            f = new e({
              ros: this.ros,
              name: this.serverName + "/status",
              messageType: "actionlib_msgs/GoalStatusArray",
            }),
            g = new e({
              ros: this.ros,
              name: this.serverName + "/result",
              messageType: this.actionName + "Result",
            });
          c.subscribe(function (a) {
            b.emit("goal", a);
          }),
            f.subscribe(function (a) {
              a.status_list.forEach(function (a) {
                b.emit("status", a);
              });
            }),
            d.subscribe(function (a) {
              b.emit("status", a.status), b.emit("feedback", a.feedback);
            }),
            g.subscribe(function (a) {
              b.emit("status", a.status), b.emit("result", a.result);
            });
        }
        var e = a("../core/Topic"),
          f = (a("../core/Message"), a("eventemitter2").EventEmitter2);
        (d.prototype.__proto__ = f.prototype), (b.exports = d);
      },
      { "../core/Message": 13, "../core/Topic": 20, eventemitter2: 2 },
    ],
    10: [
      function (a, b, c) {
        function d(a) {
          var b = this;
          (this.actionClient = a.actionClient),
            (this.goalMessage = a.goalMessage),
            (this.isFinished = !1);
          var c = new Date();
          (this.goalID = "goal_" + Math.random() + "_" + c.getTime()),
            (this.goalMessage = new e({
              goal_id: { stamp: { secs: 0, nsecs: 0 }, id: this.goalID },
              goal: this.goalMessage,
            })),
            this.on("status", function (a) {
              b.status = a;
            }),
            this.on("result", function (a) {
              (b.isFinished = !0), (b.result = a);
            }),
            this.on("feedback", function (a) {
              b.feedback = a;
            }),
            (this.actionClient.goals[this.goalID] = this);
        }
        var e = a("../core/Message"),
          f = a("eventemitter2").EventEmitter2;
        (d.prototype.__proto__ = f.prototype),
          (d.prototype.send = function (a) {
            var b = this;
            b.actionClient.goalTopic.publish(b.goalMessage),
              a &&
                setTimeout(function () {
                  b.isFinished || b.emit("timeout");
                }, a);
          }),
          (d.prototype.cancel = function () {
            var a = new e({ id: this.goalID });
            this.actionClient.cancelTopic.publish(a);
          }),
          (b.exports = d);
      },
      { "../core/Message": 13, eventemitter2: 2 },
    ],
    11: [
      function (a, b, c) {
        function d(a) {
          var b = this;
          (a = a || {}),
            (this.ros = a.ros),
            (this.serverName = a.serverName),
            (this.actionName = a.actionName),
            (this.feedbackPublisher = new e({
              ros: this.ros,
              name: this.serverName + "/feedback",
              messageType: this.actionName + "Feedback",
            })),
            this.feedbackPublisher.advertise();
          var c = new e({
            ros: this.ros,
            name: this.serverName + "/status",
            messageType: "actionlib_msgs/GoalStatusArray",
          });
          c.advertise(),
            (this.resultPublisher = new e({
              ros: this.ros,
              name: this.serverName + "/result",
              messageType: this.actionName + "Result",
            })),
            this.resultPublisher.advertise();
          var d = new e({
              ros: this.ros,
              name: this.serverName + "/goal",
              messageType: this.actionName + "Goal",
            }),
            g = new e({
              ros: this.ros,
              name: this.serverName + "/cancel",
              messageType: "actionlib_msgs/GoalID",
            });
          (this.statusMessage = new f({
            header: { stamp: { secs: 0, nsecs: 100 }, frame_id: "" },
            status_list: [],
          })),
            (this.currentGoal = null),
            (this.nextGoal = null),
            d.subscribe(function (a) {
              b.currentGoal
                ? ((b.nextGoal = a), b.emit("cancel"))
                : ((b.statusMessage.status_list = [
                    { goal_id: a.goal_id, status: 1 },
                  ]),
                  (b.currentGoal = a),
                  b.emit("goal", a.goal));
            });
          var h = function (a, b) {
            return !(a.secs > b.secs) && (a.secs < b.secs || a.nsecs < b.nsecs);
          };
          g.subscribe(function (a) {
            0 === a.stamp.secs && 0 === a.stamp.secs && "" === a.id
              ? ((b.nextGoal = null), b.currentGoal && b.emit("cancel"))
              : (b.currentGoal && a.id === b.currentGoal.goal_id.id
                  ? b.emit("cancel")
                  : b.nextGoal &&
                    a.id === b.nextGoal.goal_id.id &&
                    (b.nextGoal = null),
                b.nextGoal &&
                  h(b.nextGoal.goal_id.stamp, a.stamp) &&
                  (b.nextGoal = null),
                b.currentGoal &&
                  h(b.currentGoal.goal_id.stamp, a.stamp) &&
                  b.emit("cancel"));
          });
          setInterval(function () {
            var a = new Date(),
              d = Math.floor(a.getTime() / 1e3),
              e = Math.round(1e9 * (a.getTime() / 1e3 - d));
            (b.statusMessage.header.stamp.secs = d),
              (b.statusMessage.header.stamp.nsecs = e),
              c.publish(b.statusMessage);
          }, 500);
        }
        var e = a("../core/Topic"),
          f = a("../core/Message"),
          g = a("eventemitter2").EventEmitter2;
        (d.prototype.__proto__ = g.prototype),
          (d.prototype.setSucceeded = function (a) {
            var b = new f({
              status: { goal_id: this.currentGoal.goal_id, status: 3 },
              result: a,
            });
            this.resultPublisher.publish(b),
              (this.statusMessage.status_list = []),
              this.nextGoal
                ? ((this.currentGoal = this.nextGoal),
                  (this.nextGoal = null),
                  this.emit("goal", this.currentGoal.goal))
                : (this.currentGoal = null);
          }),
          (d.prototype.sendFeedback = function (a) {
            var b = new f({
              status: { goal_id: this.currentGoal.goal_id, status: 1 },
              feedback: a,
            });
            this.feedbackPublisher.publish(b);
          }),
          (d.prototype.setPreempted = function () {
            this.statusMessage.status_list = [];
            var a = new f({
              status: { goal_id: this.currentGoal.goal_id, status: 2 },
            });
            this.resultPublisher.publish(a),
              this.nextGoal
                ? ((this.currentGoal = this.nextGoal),
                  (this.nextGoal = null),
                  this.emit("goal", this.currentGoal.goal))
                : (this.currentGoal = null);
          }),
          (b.exports = d);
      },
      { "../core/Message": 13, "../core/Topic": 20, eventemitter2: 2 },
    ],
    12: [
      function (a, b, c) {
        var d = a("../core/Ros"),
          e = a("../mixin"),
          f = (b.exports = {
            ActionClient: a("./ActionClient"),
            ActionListener: a("./ActionListener"),
            Goal: a("./Goal"),
            SimpleActionServer: a("./SimpleActionServer"),
          });
        e(d, ["ActionClient", "SimpleActionServer"], f);
      },
      {
        "../core/Ros": 15,
        "../mixin": 27,
        "./ActionClient": 8,
        "./ActionListener": 9,
        "./Goal": 10,
        "./SimpleActionServer": 11,
      },
    ],
    13: [
      function (a, b, c) {
        function d(a) {
          e(this, a);
        }
        var e = a("object-assign");
        b.exports = d;
      },
      { "object-assign": 3 },
    ],
    14: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}), (this.ros = a.ros), (this.name = a.name);
        }
        var e = a("./Service"),
          f = a("./ServiceRequest");
        (d.prototype.get = function (a) {
          var b = new e({
              ros: this.ros,
              name: "/rosapi/get_param",
              serviceType: "rosapi/GetParam",
            }),
            c = new f({ name: this.name });
          b.callService(c, function (b) {
            var c = JSON.parse(b.value);
            a(c);
          });
        }),
          (d.prototype.set = function (a, b) {
            var c = new e({
                ros: this.ros,
                name: "/rosapi/set_param",
                serviceType: "rosapi/SetParam",
              }),
              d = new f({ name: this.name, value: JSON.stringify(a) });
            c.callService(d, b);
          }),
          (d.prototype["delete"] = function (a) {
            var b = new e({
                ros: this.ros,
                name: "/rosapi/delete_param",
                serviceType: "rosapi/DeleteParam",
              }),
              c = new f({ name: this.name });
            b.callService(c, a);
          }),
          (b.exports = d);
      },
      { "./Service": 16, "./ServiceRequest": 17 },
    ],
    15: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}),
            (this.socket = null),
            (this.idCounter = 0),
            (this.isConnected = !1),
            (this.transportLibrary = a.transportLibrary || "websocket"),
            (this.transportOptions = a.transportOptions || {}),
            "undefined" == typeof a.groovyCompatibility
              ? (this.groovyCompatibility = !0)
              : (this.groovyCompatibility = a.groovyCompatibility),
            this.setMaxListeners(0),
            a.url && this.connect(a.url);
        }
        var e = a("ws"),
          f = a("../util/workerSocket"),
          g = a("./SocketAdapter.js"),
          h = a("./Service"),
          i = a("./ServiceRequest"),
          j = a("object-assign"),
          k = a("eventemitter2").EventEmitter2;
        (d.prototype.__proto__ = k.prototype),
          (d.prototype.connect = function (a) {
            if ("socket.io" === this.transportLibrary)
              (this.socket = j(io(a, { "force new connection": !0 }), g(this))),
                this.socket.on("connect", this.socket.onopen),
                this.socket.on("data", this.socket.onmessage),
                this.socket.on("close", this.socket.onclose),
                this.socket.on("error", this.socket.onerror);
            else if (
              "RTCPeerConnection" === this.transportLibrary.constructor.name
            )
              this.socket = j(
                this.transportLibrary.createDataChannel(
                  a,
                  this.transportOptions
                ),
                g(this)
              );
            else if ("websocket" === this.transportLibrary) {
              if (!this.socket || this.socket.readyState === e.CLOSED) {
                var b = new e(a);
                (b.binaryType = "arraybuffer"), (this.socket = j(b, g(this)));
              }
            } else {
              if ("workersocket" !== this.transportLibrary)
                throw (
                  "Unknown transportLibrary: " +
                  this.transportLibrary.toString()
                );
              this.socket = j(new f(a), g(this));
            }
          }),
          (d.prototype.close = function () {
            this.socket && this.socket.close();
          }),
          (d.prototype.authenticate = function (a, b, c, d, e, f, g) {
            var h = {
              op: "auth",
              mac: a,
              client: b,
              dest: c,
              rand: d,
              t: e,
              level: f,
              end: g,
            };
            this.callOnConnection(h);
          }),
          (d.prototype.callOnConnection = function (a) {
            var b = this,
              c = JSON.stringify(a),
              d = null;
            (d =
              "socket.io" === this.transportLibrary
                ? function (a) {
                    b.socket.emit("operation", a);
                  }
                : function (a) {
                    b.socket.send(a);
                  }),
              this.isConnected
                ? d(c)
                : b.once("connection", function () {
                    d(c);
                  });
          }),
          (d.prototype.setStatusLevel = function (a, b) {
            var c = { op: "set_level", level: a, id: b };
            this.callOnConnection(c);
          }),
          (d.prototype.getActionServers = function (a, b) {
            var c = new h({
                ros: this,
                name: "/rosapi/action_servers",
                serviceType: "rosapi/GetActionServers",
              }),
              d = new i({});
            "function" == typeof b
              ? c.callService(
                  d,
                  function (b) {
                    a(b.action_servers);
                  },
                  function (a) {
                    b(a);
                  }
                )
              : c.callService(d, function (b) {
                  a(b.action_servers);
                });
          }),
          (d.prototype.getTopics = function (a, b) {
            var c = new h({
                ros: this,
                name: "/rosapi/topics",
                serviceType: "rosapi/Topics",
              }),
              d = new i();
            "function" == typeof b
              ? c.callService(
                  d,
                  function (b) {
                    a(b);
                  },
                  function (a) {
                    b(a);
                  }
                )
              : c.callService(d, function (b) {
                  a(b);
                });
          }),
          (d.prototype.getTopicsForType = function (a, b, c) {
            var d = new h({
                ros: this,
                name: "/rosapi/topics_for_type",
                serviceType: "rosapi/TopicsForType",
              }),
              e = new i({ type: a });
            "function" == typeof c
              ? d.callService(
                  e,
                  function (a) {
                    b(a.topics);
                  },
                  function (a) {
                    c(a);
                  }
                )
              : d.callService(e, function (a) {
                  b(a.topics);
                });
          }),
          (d.prototype.getServices = function (a, b) {
            var c = new h({
                ros: this,
                name: "/rosapi/services",
                serviceType: "rosapi/Services",
              }),
              d = new i();
            "function" == typeof b
              ? c.callService(
                  d,
                  function (b) {
                    a(b.services);
                  },
                  function (a) {
                    b(a);
                  }
                )
              : c.callService(d, function (b) {
                  a(b.services);
                });
          }),
          (d.prototype.getServicesForType = function (a, b, c) {
            var d = new h({
                ros: this,
                name: "/rosapi/services_for_type",
                serviceType: "rosapi/ServicesForType",
              }),
              e = new i({ type: a });
            "function" == typeof c
              ? d.callService(
                  e,
                  function (a) {
                    b(a.services);
                  },
                  function (a) {
                    c(a);
                  }
                )
              : d.callService(e, function (a) {
                  b(a.services);
                });
          }),
          (d.prototype.getServiceRequestDetails = function (a, b, c) {
            var d = new h({
                ros: this,
                name: "/rosapi/service_request_details",
                serviceType: "rosapi/ServiceRequestDetails",
              }),
              e = new i({ type: a });
            "function" == typeof c
              ? d.callService(
                  e,
                  function (a) {
                    b(a);
                  },
                  function (a) {
                    c(a);
                  }
                )
              : d.callService(e, function (a) {
                  b(a);
                });
          }),
          (d.prototype.getServiceResponseDetails = function (a, b, c) {
            var d = new h({
                ros: this,
                name: "/rosapi/service_response_details",
                serviceType: "rosapi/ServiceResponseDetails",
              }),
              e = new i({ type: a });
            "function" == typeof c
              ? d.callService(
                  e,
                  function (a) {
                    b(a);
                  },
                  function (a) {
                    c(a);
                  }
                )
              : d.callService(e, function (a) {
                  b(a);
                });
          }),
          (d.prototype.getNodes = function (a, b) {
            var c = new h({
                ros: this,
                name: "/rosapi/nodes",
                serviceType: "rosapi/Nodes",
              }),
              d = new i();
            "function" == typeof b
              ? c.callService(
                  d,
                  function (b) {
                    a(b.nodes);
                  },
                  function (a) {
                    b(a);
                  }
                )
              : c.callService(d, function (b) {
                  a(b.nodes);
                });
          }),
          (d.prototype.getNodeDetails = function (a, b, c) {
            var d = new h({
                ros: this,
                name: "/rosapi/node_details",
                serviceType: "rosapi/NodeDetails",
              }),
              e = new i({ node: a });
            "function" == typeof c
              ? d.callService(
                  e,
                  function (a) {
                    b(a.subscribing, a.publishing, a.services);
                  },
                  function (a) {
                    c(a);
                  }
                )
              : d.callService(e, function (a) {
                  b(a);
                });
          }),
          (d.prototype.getParams = function (a, b) {
            var c = new h({
                ros: this,
                name: "/rosapi/get_param_names",
                serviceType: "rosapi/GetParamNames",
              }),
              d = new i();
            "function" == typeof b
              ? c.callService(
                  d,
                  function (b) {
                    a(b.names);
                  },
                  function (a) {
                    b(a);
                  }
                )
              : c.callService(d, function (b) {
                  a(b.names);
                });
          }),
          (d.prototype.getTopicType = function (a, b, c) {
            var d = new h({
                ros: this,
                name: "/rosapi/topic_type",
                serviceType: "rosapi/TopicType",
              }),
              e = new i({ topic: a });
            "function" == typeof c
              ? d.callService(
                  e,
                  function (a) {
                    b(a.type);
                  },
                  function (a) {
                    c(a);
                  }
                )
              : d.callService(e, function (a) {
                  b(a.type);
                });
          }),
          (d.prototype.getServiceType = function (a, b, c) {
            var d = new h({
                ros: this,
                name: "/rosapi/service_type",
                serviceType: "rosapi/ServiceType",
              }),
              e = new i({ service: a });
            "function" == typeof c
              ? d.callService(
                  e,
                  function (a) {
                    b(a.type);
                  },
                  function (a) {
                    c(a);
                  }
                )
              : d.callService(e, function (a) {
                  b(a.type);
                });
          }),
          (d.prototype.getMessageDetails = function (a, b, c) {
            var d = new h({
                ros: this,
                name: "/rosapi/message_details",
                serviceType: "rosapi/MessageDetails",
              }),
              e = new i({ type: a });
            "function" == typeof c
              ? d.callService(
                  e,
                  function (a) {
                    b(a.typedefs);
                  },
                  function (a) {
                    c(a);
                  }
                )
              : d.callService(e, function (a) {
                  b(a.typedefs);
                });
          }),
          (d.prototype.decodeTypeDefs = function (a) {
            var b = this,
              c = function (a, d) {
                for (var e = {}, f = 0; f < a.fieldnames.length; f++) {
                  var g = a.fieldarraylen[f],
                    h = a.fieldnames[f],
                    i = a.fieldtypes[f];
                  if (i.indexOf("/") === -1)
                    g === -1 ? (e[h] = i) : (e[h] = [i]);
                  else {
                    for (var j = !1, k = 0; k < d.length; k++)
                      if (d[k].type.toString() === i.toString()) {
                        j = d[k];
                        break;
                      }
                    if (j) {
                      var l = c(j, d);
                      g === -1 || (e[h] = [l]);
                    } else
                      b.emit(
                        "error",
                        "Cannot find " + i + " in decodeTypeDefs"
                      );
                  }
                }
                return e;
              };
            return c(a[0], a);
          }),
          (d.prototype.getTopicsAndRawTypes = function (a, b) {
            var c = new h({
                ros: this,
                name: "/rosapi/topics_and_raw_types",
                serviceType: "rosapi/TopicsAndRawTypes",
              }),
              d = new i();
            "function" == typeof b
              ? c.callService(
                  d,
                  function (b) {
                    a(b);
                  },
                  function (a) {
                    b(a);
                  }
                )
              : c.callService(d, function (b) {
                  a(b);
                });
          }),
          (b.exports = d);
      },
      {
        "../util/workerSocket": 47,
        "./Service": 16,
        "./ServiceRequest": 17,
        "./SocketAdapter.js": 19,
        eventemitter2: 2,
        "object-assign": 3,
        ws: 43,
      },
    ],
    16: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}),
            (this.ros = a.ros),
            (this.name = a.name),
            (this.serviceType = a.serviceType),
            (this.isAdvertised = !1),
            (this._serviceCallback = null);
        }
        var e = a("./ServiceResponse"),
          f = (a("./ServiceRequest"), a("eventemitter2").EventEmitter2);
        (d.prototype.__proto__ = f.prototype),
          (d.prototype.callService = function (a, b, c) {
            if (!this.isAdvertised) {
              var d = "call_service:" + this.name + ":" + ++this.ros.idCounter;
              (b || c) &&
                this.ros.once(d, function (a) {
                  void 0 !== a.result && a.result === !1
                    ? "function" == typeof c && c(a.values)
                    : "function" == typeof b && b(new e(a.values));
                });
              var f = {
                op: "call_service",
                id: d,
                service: this.name,
                type: this.serviceType,
                args: a,
              };
              this.ros.callOnConnection(f);
            }
          }),
          (d.prototype.advertise = function (a) {
            this.isAdvertised ||
              "function" != typeof a ||
              ((this._serviceCallback = a),
              this.ros.on(this.name, this._serviceResponse.bind(this)),
              this.ros.callOnConnection({
                op: "advertise_service",
                type: this.serviceType,
                service: this.name,
              }),
              (this.isAdvertised = !0));
          }),
          (d.prototype.unadvertise = function () {
            this.isAdvertised &&
              (this.ros.callOnConnection({
                op: "unadvertise_service",
                service: this.name,
              }),
              (this.isAdvertised = !1));
          }),
          (d.prototype._serviceResponse = function (a) {
            var b = {},
              c = this._serviceCallback(a.args, b),
              d = {
                op: "service_response",
                service: this.name,
                values: new e(b),
                result: c,
              };
            a.id && (d.id = a.id), this.ros.callOnConnection(d);
          }),
          (b.exports = d);
      },
      { "./ServiceRequest": 17, "./ServiceResponse": 18, eventemitter2: 2 },
    ],
    17: [
      function (a, b, c) {
        function d(a) {
          e(this, a);
        }
        var e = a("object-assign");
        b.exports = d;
      },
      { "object-assign": 3 },
    ],
    18: [
      function (a, b, c) {
        function d(a) {
          e(this, a);
        }
        var e = a("object-assign");
        b.exports = d;
      },
      { "object-assign": 3 },
    ],
    19: [
      function (a, b, c) {
        "use strict";
        function d(a) {
          function b(b) {
            "publish" === b.op
              ? a.emit(b.topic, b.msg)
              : "service_response" === b.op
              ? a.emit(b.id, b)
              : "call_service" === b.op
              ? a.emit(b.service, b)
              : "status" === b.op &&
                (b.id ? a.emit("status:" + b.id, b) : a.emit("status", b));
          }
          function c(a, b) {
            "png" === a.op ? e(a.data, b) : b(a);
          }
          function d(a, b) {
            if (!h)
              throw "Cannot process BSON encoded message without BSON header.";
            var c = new FileReader();
            (c.onload = function () {
              var a = new Uint8Array(this.result),
                c = h.deserialize(a);
              b(c);
            }),
              c.readAsArrayBuffer(a);
          }
          return {
            onopen: function (b) {
              (a.isConnected = !0), a.emit("connection", b);
            },
            onclose: function (b) {
              (a.isConnected = !1), a.emit("close", b);
            },
            onerror: function (b) {
              a.emit("error", b);
            },
            onmessage: function (a) {
              if ("undefined" != typeof Blob && a.data instanceof Blob)
                d(a.data, function (a) {
                  c(a, b);
                });
              else if (a.data instanceof ArrayBuffer) {
                var e = f.decode(a.data, g);
                b(e);
              } else {
                var h = JSON.parse("string" == typeof a ? a : a.data);
                c(h, b);
              }
            },
          };
        }
        var e = a("../util/decompressPng"),
          f = a("cbor-js"),
          g = a("../util/cborTypedArrayTags"),
          h = null;
        "undefined" != typeof bson && (h = bson().BSON), (b.exports = d);
      },
      {
        "../util/cborTypedArrayTags": 42,
        "../util/decompressPng": 45,
        "cbor-js": 1,
      },
    ],
    20: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}),
            (this.ros = a.ros),
            (this.name = a.name),
            (this.messageType = a.messageType),
            (this.isAdvertised = !1),
            (this.compression = a.compression || "none"),
            (this.throttle_rate = a.throttle_rate || 0),
            (this.latch = a.latch || !1),
            (this.queue_size = a.queue_size || 100),
            (this.queue_length = a.queue_length || 0),
            (this.reconnect_on_close =
              void 0 === a.reconnect_on_close || a.reconnect_on_close),
            this.compression &&
              "png" !== this.compression &&
              "cbor" !== this.compression &&
              "cbor-raw" !== this.compression &&
              "none" !== this.compression &&
              (this.emit(
                "warning",
                this.compression +
                  " compression is not supported. No compression will be used."
              ),
              (this.compression = "none")),
            this.throttle_rate < 0 &&
              (this.emit(
                "warning",
                this.throttle_rate + " is not allowed. Set to 0"
              ),
              (this.throttle_rate = 0));
          var b = this;
          this.reconnect_on_close
            ? (this.callForSubscribeAndAdvertise = function (a) {
                b.ros.callOnConnection(a),
                  (b.waitForReconnect = !1),
                  (b.reconnectFunc = function () {
                    b.waitForReconnect ||
                      ((b.waitForReconnect = !0),
                      b.ros.callOnConnection(a),
                      b.ros.once("connection", function () {
                        b.waitForReconnect = !1;
                      }));
                  }),
                  b.ros.on("close", b.reconnectFunc);
              })
            : (this.callForSubscribeAndAdvertise = this.ros.callOnConnection),
            (this._messageCallback = function (a) {
              b.emit("message", new f(a));
            });
        }
        var e = a("eventemitter2").EventEmitter2,
          f = a("./Message");
        (d.prototype.__proto__ = e.prototype),
          (d.prototype.subscribe = function (a) {
            "function" == typeof a && this.on("message", a),
              this.subscribeId ||
                (this.ros.on(this.name, this._messageCallback),
                (this.subscribeId =
                  "subscribe:" + this.name + ":" + ++this.ros.idCounter),
                this.callForSubscribeAndAdvertise({
                  op: "subscribe",
                  id: this.subscribeId,
                  type: this.messageType,
                  topic: this.name,
                  compression: this.compression,
                  throttle_rate: this.throttle_rate,
                  queue_length: this.queue_length,
                }));
          }),
          (d.prototype.unsubscribe = function (a) {
            (a && (this.off("message", a), this.listeners("message").length)) ||
              (this.subscribeId &&
                (this.ros.off(this.name, this._messageCallback),
                this.reconnect_on_close &&
                  this.ros.off("close", this.reconnectFunc),
                this.emit("unsubscribe"),
                this.ros.callOnConnection({
                  op: "unsubscribe",
                  id: this.subscribeId,
                  topic: this.name,
                }),
                (this.subscribeId = null)));
          }),
          (d.prototype.advertise = function () {
            if (
              !this.isAdvertised &&
              ((this.advertiseId =
                "advertise:" + this.name + ":" + ++this.ros.idCounter),
              this.callForSubscribeAndAdvertise({
                op: "advertise",
                id: this.advertiseId,
                type: this.messageType,
                topic: this.name,
                latch: this.latch,
                queue_size: this.queue_size,
              }),
              (this.isAdvertised = !0),
              !this.reconnect_on_close)
            ) {
              var a = this;
              this.ros.on("close", function () {
                a.isAdvertised = !1;
              });
            }
          }),
          (d.prototype.unadvertise = function () {
            this.isAdvertised &&
              (this.reconnect_on_close &&
                this.ros.off("close", this.reconnectFunc),
              this.emit("unadvertise"),
              this.ros.callOnConnection({
                op: "unadvertise",
                id: this.advertiseId,
                topic: this.name,
              }),
              (this.isAdvertised = !1));
          }),
          (d.prototype.publish = function (a) {
            this.isAdvertised || this.advertise(), this.ros.idCounter++;
            var b = {
              op: "publish",
              id: "publish:" + this.name + ":" + this.ros.idCounter,
              topic: this.name,
              msg: a,
              latch: this.latch,
            };
            this.ros.callOnConnection(b);
          }),
          (b.exports = d);
      },
      { "./Message": 13, eventemitter2: 2 },
    ],
    21: [
      function (a, b, c) {
        var d = a("../mixin"),
          e = (b.exports = {
            Ros: a("./Ros"),
            Topic: a("./Topic"),
            Message: a("./Message"),
            Param: a("./Param"),
            Service: a("./Service"),
            ServiceRequest: a("./ServiceRequest"),
            ServiceResponse: a("./ServiceResponse"),
          });
        d(e.Ros, ["Param", "Service", "Topic"], e);
      },
      {
        "../mixin": 27,
        "./Message": 13,
        "./Param": 14,
        "./Ros": 15,
        "./Service": 16,
        "./ServiceRequest": 17,
        "./ServiceResponse": 18,
        "./Topic": 20,
      },
    ],
    22: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}),
            (this.position = new e(a.position)),
            (this.orientation = new f(a.orientation));
        }
        var e = a("./Vector3"),
          f = a("./Quaternion");
        (d.prototype.applyTransform = function (a) {
          this.position.multiplyQuaternion(a.rotation),
            this.position.add(a.translation);
          var b = a.rotation.clone();
          b.multiply(this.orientation), (this.orientation = b);
        }),
          (d.prototype.clone = function () {
            return new d(this);
          }),
          (d.prototype.multiply = function (a) {
            var b = a.clone();
            return (
              b.applyTransform({
                rotation: this.orientation,
                translation: this.position,
              }),
              b
            );
          }),
          (d.prototype.getInverse = function () {
            var a = this.clone();
            return (
              a.orientation.invert(),
              a.position.multiplyQuaternion(a.orientation),
              (a.position.x *= -1),
              (a.position.y *= -1),
              (a.position.z *= -1),
              a
            );
          }),
          (b.exports = d);
      },
      { "./Quaternion": 23, "./Vector3": 25 },
    ],
    23: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}),
            (this.x = a.x || 0),
            (this.y = a.y || 0),
            (this.z = a.z || 0),
            (this.w = "number" == typeof a.w ? a.w : 1);
        }
        (d.prototype.conjugate = function () {
          (this.x *= -1), (this.y *= -1), (this.z *= -1);
        }),
          (d.prototype.norm = function () {
            return Math.sqrt(
              this.x * this.x +
                this.y * this.y +
                this.z * this.z +
                this.w * this.w
            );
          }),
          (d.prototype.normalize = function () {
            var a = Math.sqrt(
              this.x * this.x +
                this.y * this.y +
                this.z * this.z +
                this.w * this.w
            );
            0 === a
              ? ((this.x = 0), (this.y = 0), (this.z = 0), (this.w = 1))
              : ((a = 1 / a),
                (this.x = this.x * a),
                (this.y = this.y * a),
                (this.z = this.z * a),
                (this.w = this.w * a));
          }),
          (d.prototype.invert = function () {
            this.conjugate(), this.normalize();
          }),
          (d.prototype.multiply = function (a) {
            var b = this.x * a.w + this.y * a.z - this.z * a.y + this.w * a.x,
              c = -this.x * a.z + this.y * a.w + this.z * a.x + this.w * a.y,
              d = this.x * a.y - this.y * a.x + this.z * a.w + this.w * a.z,
              e = -this.x * a.x - this.y * a.y - this.z * a.z + this.w * a.w;
            (this.x = b), (this.y = c), (this.z = d), (this.w = e);
          }),
          (d.prototype.clone = function () {
            return new d(this);
          }),
          (b.exports = d);
      },
      {},
    ],
    24: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}),
            (this.translation = new e(a.translation)),
            (this.rotation = new f(a.rotation));
        }
        var e = a("./Vector3"),
          f = a("./Quaternion");
        (d.prototype.clone = function () {
          return new d(this);
        }),
          (b.exports = d);
      },
      { "./Quaternion": 23, "./Vector3": 25 },
    ],
    25: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}),
            (this.x = a.x || 0),
            (this.y = a.y || 0),
            (this.z = a.z || 0);
        }
        (d.prototype.add = function (a) {
          (this.x += a.x), (this.y += a.y), (this.z += a.z);
        }),
          (d.prototype.subtract = function (a) {
            (this.x -= a.x), (this.y -= a.y), (this.z -= a.z);
          }),
          (d.prototype.multiplyQuaternion = function (a) {
            var b = a.w * this.x + a.y * this.z - a.z * this.y,
              c = a.w * this.y + a.z * this.x - a.x * this.z,
              d = a.w * this.z + a.x * this.y - a.y * this.x,
              e = -a.x * this.x - a.y * this.y - a.z * this.z;
            (this.x = b * a.w + e * -a.x + c * -a.z - d * -a.y),
              (this.y = c * a.w + e * -a.y + d * -a.x - b * -a.z),
              (this.z = d * a.w + e * -a.z + b * -a.y - c * -a.x);
          }),
          (d.prototype.clone = function () {
            return new d(this);
          }),
          (b.exports = d);
      },
      {},
    ],
    26: [
      function (a, b, c) {
        b.exports = {
          Pose: a("./Pose"),
          Quaternion: a("./Quaternion"),
          Transform: a("./Transform"),
          Vector3: a("./Vector3"),
        };
      },
      { "./Pose": 22, "./Quaternion": 23, "./Transform": 24, "./Vector3": 25 },
    ],
    27: [
      function (a, b, c) {
        b.exports = function (a, b, c) {
          b.forEach(function (b) {
            var d = c[b];
            a.prototype[b] = function (a) {
              return (a.ros = this), new d(a);
            };
          });
        };
      },
      {},
    ],
    28: [
      function (a, b, c) {
        function d(a) {
          (a = a || {}),
            (this.ros = a.ros),
            (this.fixedFrame = a.fixedFrame || "/base_link"),
            (this.angularThres = a.angularThres || 2),
            (this.transThres = a.transThres || 0.01),
            (this.rate = a.rate || 10),
            (this.updateDelay = a.updateDelay || 50);
          var b = a.topicTimeout || 2,
            c = Math.floor(b),
            d = Math.floor(1e9 * (b - c));
          (this.topicTimeout = { secs: c, nsecs: d }),
            (this.serverName = a.serverName || "/tf2_web_republisher"),
            (this.repubServiceName = a.repubServiceName || "/republish_tfs"),
            (this.currentGoal = !1),
            (this.currentTopic = !1),
            (this.frameInfos = {}),
            (this.republisherUpdateRequested = !1),
            (this.actionClient = new e({
              ros: a.ros,
              serverName: this.serverName,
              actionName: "tf2_web_republisher/TFSubscriptionAction",
              omitStatus: !0,
              omitResult: !0,
            })),
            (this.serviceClient = new g({
              ros: a.ros,
              name: this.repubServiceName,
              serviceType: "tf2_web_republisher/RepublishTFs",
            }));
        }
        var e = a("../actionlib/ActionClient"),
          f = a("../actionlib/Goal"),
          g = a("../core/Service.js"),
          h = a("../core/ServiceRequest.js"),
          i = a("../core/Topic.js"),
          j = a("../math/Transform");
        (d.prototype.processTFArray = function (a) {
          a.transforms.forEach(function (a) {
            var b = a.child_frame_id;
            "/" === b[0] && (b = b.substring(1));
            var c = this.frameInfos[b];
            c &&
              ((c.transform = new j({
                translation: a.transform.translation,
                rotation: a.transform.rotation,
              })),
              c.cbs.forEach(function (a) {
                a(c.transform);
              }));
          }, this);
        }),
          (d.prototype.updateGoal = function () {
            var a = {
              source_frames: Object.keys(this.frameInfos),
              target_frame: this.fixedFrame,
              angular_thres: this.angularThres,
              trans_thres: this.transThres,
              rate: this.rate,
            };
            if (this.ros.groovyCompatibility)
              this.currentGoal && this.currentGoal.cancel(),
                (this.currentGoal = new f({
                  actionClient: this.actionClient,
                  goalMessage: a,
                })),
                this.currentGoal.on("feedback", this.processTFArray.bind(this)),
                this.currentGoal.send();
            else {
              a.timeout = this.topicTimeout;
              var b = new h(a);
              this.serviceClient.callService(
                b,
                this.processResponse.bind(this)
              );
            }
            this.republisherUpdateRequested = !1;
          }),
          (d.prototype.processResponse = function (a) {
            this.currentTopic && this.currentTopic.unsubscribe(),
              (this.currentTopic = new i({
                ros: this.ros,
                name: a.topic_name,
                messageType: "tf2_web_republisher/TFArray",
              })),
              this.currentTopic.subscribe(this.processTFArray.bind(this));
          }),
          (d.prototype.subscribe = function (a, b) {
            "/" === a[0] && (a = a.substring(1)),
              this.frameInfos[a]
                ? this.frameInfos[a].transform &&
                  b(this.frameInfos[a].transform)
                : ((this.frameInfos[a] = { cbs: [] }),
                  this.republisherUpdateRequested ||
                    (setTimeout(this.updateGoal.bind(this), this.updateDelay),
                    (this.republisherUpdateRequested = !0))),
              this.frameInfos[a].cbs.push(b);
          }),
          (d.prototype.unsubscribe = function (a, b) {
            "/" === a[0] && (a = a.substring(1));
            for (
              var c = this.frameInfos[a], d = (c && c.cbs) || [], e = d.length;
              e--;

            )
              d[e] === b && d.splice(e, 1);
            (b && 0 !== d.length) || delete this.frameInfos[a];
          }),
          (d.prototype.dispose = function () {
            this.actionClient.dispose(),
              this.currentTopic && this.currentTopic.unsubscribe();
          }),
          (b.exports = d);
      },
      {
        "../actionlib/ActionClient": 8,
        "../actionlib/Goal": 10,
        "../core/Service.js": 16,
        "../core/ServiceRequest.js": 17,
        "../core/Topic.js": 20,
        "../math/Transform": 24,
      },
    ],
    29: [
      function (a, b, c) {
        var d = a("../core/Ros"),
          e = a("../mixin"),
          f = (b.exports = { TFClient: a("./TFClient") });
        e(d, ["TFClient"], f);
      },
      { "../core/Ros": 15, "../mixin": 27, "./TFClient": 28 },
    ],
    30: [
      function (a, b, c) {
        function d(a) {
          (this.dimension = null), (this.type = f.URDF_BOX);
          var b = a.xml.getAttribute("size").split(" ");
          this.dimension = new e({
            x: parseFloat(b[0]),
            y: parseFloat(b[1]),
            z: parseFloat(b[2]),
          });
        }
        var e = a("../math/Vector3"),
          f = a("./UrdfTypes");
        b.exports = d;
      },
      { "../math/Vector3": 25, "./UrdfTypes": 39 },
    ],
    31: [
      function (a, b, c) {
        function d(a) {
          var b = a.xml.getAttribute("rgba").split(" ");
          (this.r = parseFloat(b[0])),
            (this.g = parseFloat(b[1])),
            (this.b = parseFloat(b[2])),
            (this.a = parseFloat(b[3]));
        }
        b.exports = d;
      },
      {},
    ],
    32: [
      function (a, b, c) {
        function d(a) {
          (this.type = e.URDF_CYLINDER),
            (this.length = parseFloat(a.xml.getAttribute("length"))),
            (this.radius = parseFloat(a.xml.getAttribute("radius")));
        }
        var e = a("./UrdfTypes");
        b.exports = d;
      },
      { "./UrdfTypes": 39 },
    ],
    33: [
      function (a, b, c) {
        function d(a) {
          (this.name = a.xml.getAttribute("name")),
            (this.type = a.xml.getAttribute("type"));
          var b = a.xml.getElementsByTagName("parent");
          b.length > 0 && (this.parent = b[0].getAttribute("link"));
          var c = a.xml.getElementsByTagName("child");
          c.length > 0 && (this.child = c[0].getAttribute("link"));
          var d = a.xml.getElementsByTagName("limit");
          d.length > 0 &&
            ((this.minval = parseFloat(d[0].getAttribute("lower"))),
            (this.maxval = parseFloat(d[0].getAttribute("upper"))));
          var h = a.xml.getElementsByTagName("origin");
          if (0 === h.length) this.origin = new e();
          else {
            var i = h[0].getAttribute("xyz"),
              j = new f();
            i &&
              ((i = i.split(" ")),
              (j = new f({
                x: parseFloat(i[0]),
                y: parseFloat(i[1]),
                z: parseFloat(i[2]),
              })));
            var k = h[0].getAttribute("rpy"),
              l = new g();
            if (k) {
              k = k.split(" ");
              var m = parseFloat(k[0]),
                n = parseFloat(k[1]),
                o = parseFloat(k[2]),
                p = m / 2,
                q = n / 2,
                r = o / 2,
                s =
                  Math.sin(p) * Math.cos(q) * Math.cos(r) -
                  Math.cos(p) * Math.sin(q) * Math.sin(r),
                t =
                  Math.cos(p) * Math.sin(q) * Math.cos(r) +
                  Math.sin(p) * Math.cos(q) * Math.sin(r),
                u =
                  Math.cos(p) * Math.cos(q) * Math.sin(r) -
                  Math.sin(p) * Math.sin(q) * Math.cos(r),
                v =
                  Math.cos(p) * Math.cos(q) * Math.cos(r) +
                  Math.sin(p) * Math.sin(q) * Math.sin(r);
              (l = new g({ x: s, y: t, z: u, w: v })), l.normalize();
            }
            this.origin = new e({ position: j, orientation: l });
          }
        }
        var e = a("../math/Pose"),
          f = a("../math/Vector3"),
          g = a("../math/Quaternion");
        b.exports = d;
      },
      { "../math/Pose": 22, "../math/Quaternion": 23, "../math/Vector3": 25 },
    ],
    34: [
      function (a, b, c) {
        function d(a) {
          (this.name = a.xml.getAttribute("name")), (this.visuals = []);
          for (
            var b = a.xml.getElementsByTagName("visual"), c = 0;
            c < b.length;
            c++
          )
            this.visuals.push(new e({ xml: b[c] }));
        }
        var e = a("./UrdfVisual");
        b.exports = d;
      },
      { "./UrdfVisual": 40 },
    ],
    35: [
      function (a, b, c) {
        function d(a) {
          (this.textureFilename = null),
            (this.color = null),
            (this.name = a.xml.getAttribute("name"));
          var b = a.xml.getElementsByTagName("texture");
          b.length > 0 &&
            (this.textureFilename = b[0].getAttribute("filename"));
          var c = a.xml.getElementsByTagName("color");
          c.length > 0 && (this.color = new e({ xml: c[0] }));
        }
        var e = a("./UrdfColor");
        d.prototype.isLink = function () {
          return null === this.color && null === this.textureFilename;
        };
        var f = a("object-assign");
        (d.prototype.assign = function (a) {
          return f(this, a);
        }),
          (b.exports = d);
      },
      { "./UrdfColor": 31, "object-assign": 3 },
    ],
    36: [
      function (a, b, c) {
        function d(a) {
          (this.scale = null),
            (this.type = f.URDF_MESH),
            (this.filename = a.xml.getAttribute("filename"));
          var b = a.xml.getAttribute("scale");
          if (b) {
            var c = b.split(" ");
            this.scale = new e({
              x: parseFloat(c[0]),
              y: parseFloat(c[1]),
              z: parseFloat(c[2]),
            });
          }
        }
        var e = a("../math/Vector3"),
          f = a("./UrdfTypes");
        b.exports = d;
      },
      { "../math/Vector3": 25, "./UrdfTypes": 39 },
    ],
    37: [
      function (a, b, c) {
        function d(a) {
          a = a || {};
          var b = a.xml,
            c = a.string;
          if (
            ((this.materials = {}), (this.links = {}), (this.joints = {}), c)
          ) {
            var d = new h();
            b = d.parseFromString(c, "text/xml");
          }
          var i = b.documentElement;
          this.name = i.getAttribute("name");
          for (var j = i.childNodes, k = 0; k < j.length; k++) {
            var l = j[k];
            if ("material" === l.tagName) {
              var m = new e({ xml: l });
              void 0 !== this.materials[m.name]
                ? this.materials[m.name].isLink()
                  ? this.materials[m.name].assign(m)
                  : console.warn("Material " + m.name + "is not unique.")
                : (this.materials[m.name] = m);
            } else if ("link" === l.tagName) {
              var n = new f({ xml: l });
              if (void 0 !== this.links[n.name])
                console.warn("Link " + n.name + " is not unique.");
              else {
                for (var o = 0; o < n.visuals.length; o++) {
                  var p = n.visuals[o].material;
                  null !== p &&
                    (void 0 !== this.materials[p.name]
                      ? (n.visuals[o].material = this.materials[p.name])
                      : (this.materials[p.name] = p));
                }
                this.links[n.name] = n;
              }
            } else if ("joint" === l.tagName) {
              var q = new g({ xml: l });
              this.joints[q.name] = q;
            }
          }
        }
        var e = a("./UrdfMaterial"),
          f = a("./UrdfLink"),
          g = a("./UrdfJoint"),
          h = a("xmldom").DOMParser;
        b.exports = d;
      },
      { "./UrdfJoint": 33, "./UrdfLink": 34, "./UrdfMaterial": 35, xmldom: 46 },
    ],
    38: [
      function (a, b, c) {
        function d(a) {
          (this.type = e.URDF_SPHERE),
            (this.radius = parseFloat(a.xml.getAttribute("radius")));
        }
        var e = a("./UrdfTypes");
        b.exports = d;
      },
      { "./UrdfTypes": 39 },
    ],
    39: [
      function (a, b, c) {
        b.exports = {
          URDF_SPHERE: 0,
          URDF_BOX: 1,
          URDF_CYLINDER: 2,
          URDF_MESH: 3,
        };
      },
      {},
    ],
    40: [
      function (a, b, c) {
        function d(a) {
          var b = a.xml;
          (this.origin = null),
            (this.geometry = null),
            (this.material = null),
            (this.name = a.xml.getAttribute("name"));
          var c = b.getElementsByTagName("origin");
          if (0 === c.length) this.origin = new e();
          else {
            var d = c[0].getAttribute("xyz"),
              m = new f();
            d &&
              ((d = d.split(" ")),
              (m = new f({
                x: parseFloat(d[0]),
                y: parseFloat(d[1]),
                z: parseFloat(d[2]),
              })));
            var n = c[0].getAttribute("rpy"),
              o = new g();
            if (n) {
              n = n.split(" ");
              var p = parseFloat(n[0]),
                q = parseFloat(n[1]),
                r = parseFloat(n[2]),
                s = p / 2,
                t = q / 2,
                u = r / 2,
                v =
                  Math.sin(s) * Math.cos(t) * Math.cos(u) -
                  Math.cos(s) * Math.sin(t) * Math.sin(u),
                w =
                  Math.cos(s) * Math.sin(t) * Math.cos(u) +
                  Math.sin(s) * Math.cos(t) * Math.sin(u),
                x =
                  Math.cos(s) * Math.cos(t) * Math.sin(u) -
                  Math.sin(s) * Math.sin(t) * Math.cos(u),
                y =
                  Math.cos(s) * Math.cos(t) * Math.cos(u) +
                  Math.sin(s) * Math.sin(t) * Math.sin(u);
              (o = new g({ x: v, y: w, z: x, w: y })), o.normalize();
            }
            this.origin = new e({ position: m, orientation: o });
          }
          var z = b.getElementsByTagName("geometry");
          if (z.length > 0) {
            for (var A = z[0], B = null, C = 0; C < A.childNodes.length; C++) {
              var D = A.childNodes[C];
              if (1 === D.nodeType) {
                B = D;
                break;
              }
            }
            var E = B.nodeName;
            "sphere" === E
              ? (this.geometry = new l({ xml: B }))
              : "box" === E
              ? (this.geometry = new i({ xml: B }))
              : "cylinder" === E
              ? (this.geometry = new h({ xml: B }))
              : "mesh" === E
              ? (this.geometry = new k({ xml: B }))
              : console.warn("Unknown geometry type " + E);
          }
          var F = b.getElementsByTagName("material");
          F.length > 0 && (this.material = new j({ xml: F[0] }));
        }
        var e = a("../math/Pose"),
          f = a("../math/Vector3"),
          g = a("../math/Quaternion"),
          h = a("./UrdfCylinder"),
          i = a("./UrdfBox"),
          j = a("./UrdfMaterial"),
          k = a("./UrdfMesh"),
          l = a("./UrdfSphere");
        b.exports = d;
      },
      {
        "../math/Pose": 22,
        "../math/Quaternion": 23,
        "../math/Vector3": 25,
        "./UrdfBox": 30,
        "./UrdfCylinder": 32,
        "./UrdfMaterial": 35,
        "./UrdfMesh": 36,
        "./UrdfSphere": 38,
      },
    ],
    41: [
      function (a, b, c) {
        b.exports = a("object-assign")(
          {
            UrdfBox: a("./UrdfBox"),
            UrdfColor: a("./UrdfColor"),
            UrdfCylinder: a("./UrdfCylinder"),
            UrdfLink: a("./UrdfLink"),
            UrdfMaterial: a("./UrdfMaterial"),
            UrdfMesh: a("./UrdfMesh"),
            UrdfModel: a("./UrdfModel"),
            UrdfSphere: a("./UrdfSphere"),
            UrdfVisual: a("./UrdfVisual"),
          },
          a("./UrdfTypes")
        );
      },
      {
        "./UrdfBox": 30,
        "./UrdfColor": 31,
        "./UrdfCylinder": 32,
        "./UrdfLink": 34,
        "./UrdfMaterial": 35,
        "./UrdfMesh": 36,
        "./UrdfModel": 37,
        "./UrdfSphere": 38,
        "./UrdfTypes": 39,
        "./UrdfVisual": 40,
        "object-assign": 3,
      },
    ],
    42: [
      function (a, b, c) {
        "use strict";
        function d() {
          j ||
            ((j = !0),
            console.warn(
              "CBOR 64-bit integer array values may lose precision. No further warnings."
            ));
        }
        function e(a) {
          d();
          for (
            var b = a.byteLength,
              c = a.byteOffset,
              e = b / 8,
              f = a.buffer.slice(c, c + b),
              g = new Uint32Array(f),
              h = new Array(e),
              j = 0;
            j < e;
            j++
          ) {
            var k = 2 * j,
              l = g[k],
              m = g[k + 1];
            h[j] = l + i * m;
          }
          return h;
        }
        function f(a) {
          d();
          for (
            var b = a.byteLength,
              c = a.byteOffset,
              e = b / 8,
              f = a.buffer.slice(c, c + b),
              g = new Uint32Array(f),
              h = new Int32Array(f),
              j = new Array(e),
              k = 0;
            k < e;
            k++
          ) {
            var l = 2 * k,
              m = g[l],
              n = h[l + 1];
            j[k] = m + i * n;
          }
          return j;
        }
        function g(a, b) {
          var c = a.byteLength,
            d = a.byteOffset,
            e = a.buffer.slice(d, d + c);
          return new b(e);
        }
        function h(a, b) {
          if (b in k) {
            var c = k[b];
            return g(a, c);
          }
          return b in l ? l[b](a) : a;
        }
        var i = Math.pow(2, 32),
          j = !1,
          k = {
            64: Uint8Array,
            69: Uint16Array,
            70: Uint32Array,
            72: Int8Array,
            77: Int16Array,
            78: Int32Array,
            85: Float32Array,
            86: Float64Array,
          },
          l = { 71: e, 79: f };
        "undefined" != typeof b && b.exports && (b.exports = h);
      },
      {},
    ],
    43: [
      function (a, b, c) {
        b.exports = "undefined" != typeof window ? window.WebSocket : WebSocket;
      },
      {},
    ],
    44: [
      function (a, b, c) {
        b.exports = function () {
          return document.createElement("canvas");
        };
      },
      {},
    ],
    45: [
      function (a, b, c) {
        "use strict";
        function d(a, b) {
          var c = new f();
          (c.onload = function () {
            var a = new e(),
              d = a.getContext("2d");
            (a.width = c.width),
              (a.height = c.height),
              (d.imageSmoothingEnabled = !1),
              (d.webkitImageSmoothingEnabled = !1),
              (d.mozImageSmoothingEnabled = !1),
              d.drawImage(c, 0, 0);
            for (
              var f = d.getImageData(0, 0, c.width, c.height).data,
                g = "",
                h = 0;
              h < f.length;
              h += 4
            )
              g += String.fromCharCode(f[h], f[h + 1], f[h + 2]);
            b(JSON.parse(g));
          }),
            (c.src = "data:image/png;base64," + a);
        }
        var e = a("canvas"),
          f = e.Image || window.Image;
        b.exports = d;
      },
      { canvas: 44 },
    ],
    46: [
      function (a, b, c) {
        (c.DOMImplementation = window.DOMImplementation),
          (c.XMLSerializer = window.XMLSerializer),
          (c.DOMParser = window.DOMParser);
      },
      {},
    ],
    47: [
      function (a, b, c) {
        function d(a) {
          (this.socket_ = e(f)),
            this.socket_.addEventListener(
              "message",
              this.handleWorkerMessage_.bind(this)
            ),
            this.socket_.postMessage({ uri: a });
        }
        var e = a("webworkify"),
          f = a("./workerSocketImpl");
        (d.prototype.handleWorkerMessage_ = function (a) {
          var b = a.data;
          if (b instanceof ArrayBuffer || "string" == typeof b)
            this.onmessage(a);
          else {
            var c = b.type;
            if ("close" === c) this.onclose(null);
            else if ("open" === c) this.onopen(null);
            else {
              if ("error" !== c) throw "Unknown message from workersocket";
              this.onerror(null);
            }
          }
        }),
          (d.prototype.send = function (a) {
            this.socket_.postMessage(a);
          }),
          (d.prototype.close = function () {
            this.socket_.postMessage({ close: !0 });
          }),
          (b.exports = d);
      },
      { "./workerSocketImpl": 48, webworkify: 5 },
    ],
    48: [
      function (a, b, c) {
        var d = d || a("ws");
        b.exports = function (a) {
          function b(b) {
            var c = b.data;
            c instanceof ArrayBuffer ? a.postMessage(c, [c]) : a.postMessage(c);
          }
          function c(b) {
            a.postMessage({ type: b.type });
          }
          var e = null;
          a.addEventListener("message", function (a) {
            var f = a.data;
            if ("string" == typeof f) e.send(f);
            else if (f.hasOwnProperty("close")) e.close(), (e = null);
            else {
              if (!f.hasOwnProperty("uri"))
                throw "Unknown message to WorkerSocket";
              var g = f.uri;
              (e = new d(g)),
                (e.binaryType = "arraybuffer"),
                (e.onmessage = b),
                (e.onclose = c),
                (e.onopen = c),
                (e.onerror = c);
            }
          });
        };
      },
      { ws: 43 },
    ],
  },
  {},
  [7]
);
