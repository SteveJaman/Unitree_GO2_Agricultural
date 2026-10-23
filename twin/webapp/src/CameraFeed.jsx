import { useEffect, useRef, useState } from 'react';
import { useTopic } from './RosProvider.jsx';

// JPEG copies made in go2-3d by twin/image_compressor.py: the raw
// /camera/image_raw (~2.7 MB per 720p frame) is too heavy for rosbridge.
const SOURCES = {
  camera: { label: 'Live', topic: '/twin/camera/compressed' },
  yolo: { label: 'AI', topic: '/twin/yolo/compressed' },
};

/** Picture-in-picture camera, bottom right; click to enlarge. */
export default function CameraFeed() {
  const [source, setSource] = useState('camera');
  const [big, setBig] = useState(false);
  const [last, setLast] = useState(0);
  const [now, setNow] = useState(Date.now());
  const img = useRef();
  const url = useRef(null);

  useTopic(SOURCES[source].topic, 'sensor_msgs/msg/CompressedImage', (m) => {
    if (!img.current) return;
    if (typeof m.data === 'string') {
      img.current.src = `data:image/jpeg;base64,${m.data}`;
    } else {
      if (url.current) URL.revokeObjectURL(url.current);
      url.current = URL.createObjectURL(new Blob([m.data], { type: 'image/jpeg' }));
      img.current.src = url.current;
    }
    setLast(Date.now());
  });

  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => {
      clearInterval(id);
      if (url.current) URL.revokeObjectURL(url.current);
    };
  }, []);

  const live = now - last < 3000;
  return (
    <div className={`pip ${big ? 'big' : ''}`}>
      <img ref={img} alt="" style={{ visibility: live ? 'visible' : 'hidden' }} onClick={() => setBig(!big)} />
      {!live && <div className="pip-empty">Camera unavailable</div>}
      <div className="pip-bar">
        {live && <span className="live"><span className="live-dot" />LIVE</span>}
        <div className="pip-tabs">
          {Object.entries(SOURCES).map(([key, s]) => (
            <button key={key} className={key === source ? 'active' : ''} onClick={() => setSource(key)}>{s.label}</button>
          ))}
          <button onClick={() => setBig(!big)} title={big ? 'Shrink' : 'Enlarge'}>{big ? '⤡' : '⤢'}</button>
        </div>
      </div>
    </div>
  );
}
