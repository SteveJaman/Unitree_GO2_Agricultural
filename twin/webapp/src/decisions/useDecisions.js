// Live traversability decisions for the Decisions tab, from rosbridge:
//   /semantic/labels_json  every tracked object: measured height, VLM answer, final label,
//                          source (geometry / vlm / protected), protected list
//   /gait/status           the gait supervisor's choice and why
//   /step_over/events      the step-over coordinator's decisions (pause, crawl, resume)
//   /step_over/status      its mode and state, once a second
// Without the robot (or with ?demo=1) a scripted sample plays, marked as such.
import { useEffect, useRef, useState } from 'react';
import { usePublisher, useRos, useTopic } from '../RosProvider.jsx';
import { HEIGHT_ERROR_CM } from './content.js';

const MAX_EVENTS = 300;
const DEMO_PARAM = new URLSearchParams(window.location.search).get('demo') === '1';

/** One plain sentence: why this object got its decision. */
export function explain(o, limits) {
  const h = o.height;
  const step = limits.step;
  const hTxt = h == null ? null : `${(h * 100).toFixed(1)} cm`;
  const band = h == null ? null : h <= step ? 'Step over' : h < limits.lethal ? 'Careful' : 'Must avoid';
  const range = h == null ? '' : h <= step ? `(up to ${step * 100} cm)` : h < limits.lethal
    ? `(${step * 100}–${limits.lethal * 100} cm)` : `(${limits.lethal * 100} cm or more)`;
  const src = o.source || '';
  if (src === 'protected') return `${cap(o.class)} is on the operator’s protected list and the LiDAR measured it (${hTxt}): must avoid, whatever its height.`;
  if (src === 'protected_unmeasured') return `${cap(o.class)} is protected but not measured by the LiDAR yet: careful until it is (one signal never blocks the route).`;
  if (src === 'vlm_unmeasured') return `The VLM answered ${word(o.vlm)}, but there is no LiDAR height yet: held at Careful until measured.`;
  if (h == null) return 'Seen by the camera, not yet measured by the LiDAR.';
  if (src === 'vlm') return `Measured ${hTxt} (${band} by height), but the VLM judged it ${word(o.vlm)}: the model can only make it more careful.`;
  const vlm = o.vlm ? `; the VLM agreed or was less careful (${word(o.vlm)})` : '';
  return `Measured ${hTxt}: ${band} band ${range}${vlm}.`;
}

/** Within 1.5 cm of a cutoff: the 1.2 cm median error could put it on either side. */
export function nearCutoff(h, limits) {
  if (h == null) return null;
  for (const c of [limits.step, limits.lethal]) {
    if (Math.abs(h - c) < 0.015) return c;
  }
  return null;
}

const cap = (s) => (s ? s[0].toUpperCase() + s.slice(1) : 'Object');
const word = (l) => ({ STEP_OVER: 'step over', CAUTION: 'careful', AVOID: 'must avoid', UNKNOWN: 'unsure' }[l] || 'nothing');

// --- the sample (no robot) -------------------------------------------------------------
const DEMO_OBJECTS = [
  { track_id: 1, class: 'cable', height: 0.021, vlm: 'STEP_OVER', label: 'STEP_OVER', source: 'agree', x: 1.2, y: 0.1 },
  { track_id: 2, class: 'book', height: 0.058, vlm: 'STEP_OVER', label: 'CAUTION', source: 'geometry', x: 2.0, y: -0.2 },
  { track_id: 3, class: 'laptop', height: 0.060, vlm: null, label: 'AVOID', source: 'protected', protected: true, x: 2.4, y: 0.5 },
  { track_id: 4, class: 'shoe', height: 0.124, vlm: 'STEP_OVER', label: 'CAUTION', source: 'geometry', x: 3.1, y: -0.4 },
  { track_id: 5, class: 'potted plant', height: null, vlm: null, label: 'CAUTION', source: 'protected_unmeasured', protected: true, x: 3.6, y: 0.8 },
  { track_id: 6, class: 'cup', height: 0.095, vlm: 'AVOID', label: 'AVOID', source: 'vlm', x: 4.0, y: 0.0 },
  { track_id: 7, class: 'box', height: 0.31, vlm: 'STEP_OVER', label: 'AVOID', source: 'geometry', x: 4.6, y: -0.6 },
];
const DEMO_SCRIPT = [
  ['gait', 'NORMAL', 'Nothing ahead: normal gait'],
  ['seen', 1, null],
  ['gait', 'NORMAL', 'Cable 2.1 cm ahead: within the step, normal gait'],
  ['seen', 2, null],
  ['step', 'info', 'would step over book #2: 6 cm high, 4 cm deep, near edge 0.78 m ahead, +0.02 m to the side'],
  ['seen', 3, null],
  ['gait', 'CAREFUL', 'Laptop ahead (protected): careful gait; the route goes around it'],
  ['seen', 4, null],
  ['seen', 5, null],
  ['seen', 6, null],
  ['seen', 7, null],
  ['gait', 'STOP', 'Box 31 cm ahead: blocked; the planner reroutes'],
  ['gait', 'NORMAL', 'Clear ahead again'],
];

export function useDecisions() {
  const { status } = useRos();
  const live = status === 'connected' && !DEMO_PARAM;
  const [objects, setObjects] = useState([]);
  const [events, setEvents] = useState([]);
  const [gait, setGait] = useState(null);
  const [stepOver, setStepOver] = useState(null);
  const [protectedList, setProtectedList] = useState(null);
  const [meta, setMeta] = useState({});
  const lastLabels = useRef({});
  const lastGait = useRef(null);
  const sendProtected = usePublisher('/semantic/protected_classes_set', 'std_msgs/msg/String');

  const push = (e) => setEvents((ev) => [{ t: Date.now() / 1000, ...e }, ...ev].slice(0, MAX_EVENTS));
  const limits = { step: stepOver?.limits?.h_min ?? meta.step ?? 0.03, lethal: 0.22 };

  // live
  useTopic('/semantic/labels_json', 'std_msgs/msg/String', (m) => {
    let d;
    try { d = JSON.parse(m.data); } catch { return; }
    const objs = d.objects || [];
    setObjects(objs);
    if (d.protected_classes) setProtectedList(d.protected_classes);
    setMeta({ model: d.model, asked: d.asked, failed: d.failed, step: d.step_over_height_max });
    for (const o of objs) {
      const prev = lastLabels.current[o.track_id];
      if (prev !== o.label) {
        push({ kind: prev === undefined ? 'seen' : 'label', level: o.label === 'AVOID' ? 'warn' : 'info',
               text: `${cap(o.class)} #${o.track_id}: ${prev === undefined ? '' : `${word(prev)} → `}${word(o.label)}`,
               why: explain(o, limits) });
        lastLabels.current[o.track_id] = o.label;
      }
    }
  }, { enabled: live, throttle: 500 });
  useTopic('/gait/status', 'std_msgs/msg/String', (m) => {
    let g;
    try { g = JSON.parse(m.data); } catch { return; }
    setGait(g);
    if (g.gait !== lastGait.current) {
      const objs = (g.objects || []).join(', ');
      push({ kind: 'gait', level: g.gait === 'STOP' ? 'warn' : 'info',
             text: `Gait ${lastGait.current ? `${lastGait.current} → ` : ''}${g.gait}`,
             why: g.inputs_stale ? 'inputs lost: careful by default'
               : `${objs ? `ahead: ${objs}; ` : ''}decided by ${g.backstop_raised ? `the backstop (${g.backstop_source})` : 'the measured boxes'}`
                 + (g.backend_state && /DRY|inactive|dry/i.test(String(g.backend_state)) ? ' (logged; the real dog walks on)' : '') });
      lastGait.current = g.gait;
    }
  }, { enabled: live, throttle: 250 });
  useTopic('/step_over/events', 'std_msgs/msg/String', (m) => {
    try {
      const e = JSON.parse(m.data);
      push({ kind: 'step', level: e.level === 'error' ? 'bad' : e.level === 'warn' ? 'warn' : 'info',
             text: e.text, why: `step-over ${e.mode}, ${e.state}`, t: e.t });
    } catch { /* ignore */ }
  }, { enabled: live, queueLength: 20 });
  useTopic('/step_over/status', 'std_msgs/msg/String', (m) => {
    try { setStepOver(JSON.parse(m.data)); } catch { /* ignore */ }
  }, { enabled: live, throttle: 1000 });

  // the sample
  useEffect(() => {
    if (live) return undefined;
    setObjects([]);
    setEvents([]);
    setProtectedList(['potted plant', 'vase', 'laptop', 'cell phone', 'tv', 'person', 'cat', 'dog', 'wine glass']);
    setStepOver({ mode: 'dry', state: 'idle', limits: { h_min: 0.03, h_max: 0.10, depth_max: 0.10 } });
    let i = 0;
    const tick = () => {
      const [kind, a, b] = DEMO_SCRIPT[i % DEMO_SCRIPT.length];
      // First pass: objects appear one by one. After that they stay; only the timeline moves.
      if (kind === 'seen' && i >= DEMO_SCRIPT.length) { i += 1; tick(); return; }
      if (kind === 'seen') {
        const o = DEMO_OBJECTS.find((x) => x.track_id === a);
        setObjects((os) => (os.some((x) => x.track_id === a) ? os : [...os, o]));
        push({ kind: 'seen', level: o.label === 'AVOID' ? 'warn' : 'info', text: `${cap(o.class)} #${o.track_id}: ${word(o.label)}`,
               why: explain(o, { step: 0.03, lethal: 0.22 }) });
      } else if (kind === 'gait') {
        setGait({ gait: a });
        push({ kind: 'gait', level: a === 'STOP' ? 'warn' : 'info', text: `Gait ${a}`, why: b });
      } else {
        push({ kind: 'step', level: a, text: b, why: 'step-over dry, idle' });
      }
      i += 1;
    };
    tick();
    const id = setInterval(tick, 2200);
    return () => clearInterval(id);
  }, [live]);

  const setProtected = (classes) => {
    setProtectedList(classes);
    if (!live) return 'sample';
    return sendProtected({ data: JSON.stringify({ classes }) }) ? 'sent' : 'offline';
  };

  return { live, demo: !live, objects, events, gait, stepOver, protectedList, setProtected, limits, meta,
           heightErrorCm: HEIGHT_ERROR_CM };
}
