import { useEffect, useRef, useState } from 'react';
import { useTopic } from './RosProvider.jsx';

// JPEG copies made in go2-3d by twin/image_compressor.py: the raw
// /camera/image_raw (~2.7 MB per 720p frame) is too heavy for rosbridge.
const SOURCES = {
  camera: { label: 'Camera', topic: '/twin/camera/compressed' },
  yolo: { label: 'YOLO', topic: '/twin/yolo/compressed' },
};

/** Live camera (or YOLO-annotated) feed from sensor_msgs/CompressedImage. */
export default function CameraFeed() {
  const [source, setSource] = useState('camera');
  const [fps, setFps] = useState(0);
  const [last, setLast] = useState(0);
  const img = useRef();
  const url = useRef(null);
  const frames = useRef(0);

  useTopic(SOURCES[source].topic, 'sensor_msgs/msg/CompressedImage', (m) => {
    if (!img.current) return;
    if (typeof m.data === 'string') {
      img.current.src = `data:image/jpeg;base64,${m.data}`;
    } else {
      if (url.current) URL.revokeObjectURL(url.current);
      url.current = URL.createObjectURL(new Blob([m.data], { type: 'image/jpeg' }));
      img.current.src = url.current;
    }
    frames.current += 1;
    setLast(Date.now());
  });

  useEffect(() => {
    const id = setInterval(() => { setFps(frames.current); frames.current = 0; }, 1000);
    return () => {
      clearInterval(id);
      if (url.current) URL.revokeObjectURL(url.current);
    };
  }, []);

  const live = Date.now() - last < 3000;
  return (
    <section className="camera">
      <div className="camera-head">
        {Object.entries(SOURCES).map(([key, s]) => (
          <button key={key} className={`tab ${key === source ? 'active' : ''}`}
                  onClick={() => setSource(key)}>{s.label}</button>
        ))}
        <span className="fps">{live ? `${fps} fps` : 'no frames'}</span>
      </div>
      <div className="camera-body">
        <img ref={img} alt="" style={{ visibility: live ? 'visible' : 'hidden' }} />
        {!live && (
          <div className="camera-empty">
            Waiting for {SOURCES[source].topic}
            <small>{source === 'yolo' ? 'YOLO runs with --ai and the camera' : 'camera relay must be publishing'}</small>
          </div>
        )}
      </div>
    </section>
  );
}
