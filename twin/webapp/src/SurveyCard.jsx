import { useLatest, usePublisher } from './RosProvider.jsx';

const ACTIVE = ['EXPLORING', 'SEGMENTING', 'VISITING', 'SCANNING', 'PAUSED', 'RETURNING'];
const STEPS = ['EXPLORING', 'SEGMENTING', 'VISITING', 'RETURNING'];   // progress dots

const mmss = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

function phase(st) {
  const room = st.room ? `${st.room.replace(/^room_/, 'Room ')} (${st.room_index}/${st.rooms_total})` : '';
  switch (st.state) {
    case 'EXPLORING': return ['Mapping the area', 'Exploring until nothing is left unseen'];
    case 'SEGMENTING': return ['Finding rooms', 'Splitting the map at doorways'];
    case 'VISITING': return ['Going to ' + room, 'Visiting every room to identify it'];
    case 'SCANNING': return ['Looking around ' + room, 'Spinning while the AI identifies objects'];
    case 'PAUSED': return ['Paused: charging', 'Continues when the battery is back'];
    case 'RETURNING': return ['Returning to dock', 'All rooms surveyed'];
    case 'DONE': return [`Survey complete`, `${st.results?.length ?? 0} rooms mapped and named`];
    case 'STOPPED': return ['Survey stopped', st.note || ''];
    default: return ['Auto Survey', 'Map and name every room, no input needed'];
  }
}

/** Autonomous survey (survey_manager.py): start / stop and live progress. */
export default function SurveyCard({ connected }) {
  const [status] = useLatest('/survey/status', 'std_msgs/msg/String');
  const send = usePublisher('/survey/command', 'std_msgs/msg/String');
  let st = {};
  try { st = status ? JSON.parse(status.data) : {}; } catch { st = {}; }
  const active = ACTIVE.includes(st.state);
  const [title, sub] = phase(st);
  const stepIdx = Math.max(0, STEPS.indexOf(st.state === 'SCANNING' ? 'VISITING' : st.state));
  const named = (st.results || []).filter((r) => r.label);

  return (
    <div className={`survey ${active ? 'on' : ''}`}>
      <button className="survey-main" disabled={!connected || !status}
              onClick={() => send({ data: JSON.stringify({ op: active ? 'stop' : 'start' }) })}
              title={status ? '' : 'Survey manager not running'}>
        <span className="survey-icon">{active ? '■' : '◎'}</span>
        <span className="explore-text">
          <span className="explore-title">{active ? title : st.state === 'DONE' ? title : 'Auto Survey'}</span>
          <span className="explore-sub">{active ? `${sub} · ${mmss(st.elapsed_s || 0)}` : st.state === 'DONE' ? sub : 'Map and name every room, no input needed'}</span>
        </span>
      </button>
      {active && (
        <div className="survey-steps">
          {STEPS.map((s, i) => <span key={s} className={`step ${i < stepIdx ? 'done' : i === stepIdx ? 'now' : ''}`} />)}
          {st.state === 'PAUSED' && <span className="pill warn">Charging</span>}
        </div>
      )}
      {named.length > 0 && (
        <div className="survey-results">
          {named.slice(-4).map((r) => (
            <span key={r.id} className="chip small result" title={r.reason}>✨ {r.label}</span>
          ))}
        </div>
      )}
    </div>
  );
}
