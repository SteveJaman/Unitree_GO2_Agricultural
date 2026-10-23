import { createContext, useContext, useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

// The dashboard is opened from the VM (localhost) or the Windows host
// (http://<VM IP>:3000), so rosbridge is on the same host that served the
// page, not necessarily "localhost". Override with ?ws=ws://host:9090.
export function defaultRosUrl() {
  const fromQuery = new URLSearchParams(window.location.search).get('ws');
  return fromQuery || `ws://${window.location.hostname || 'localhost'}:9090`;
}

const RosContext = createContext(null);

export function RosProvider({ url = defaultRosUrl(), children }) {
  const [ros] = useState(() => new ROSLIB.Ros());
  const [status, setStatus] = useState('connecting');
  // Bumped on every (re)connection: subscriptions re-subscribe on change,
  // because roslib does not replay them after a reconnect.
  const [connection, setConnection] = useState(0);

  useEffect(() => {
    let closed = false;
    let retry = null;
    const connect = () => {
      setStatus('connecting');
      ros.connect(url);
    };
    const onConnection = () => {
      setStatus('connected');
      setConnection((n) => n + 1);
    };
    const onError = () => setStatus('error');
    const onClose = () => {
      if (closed) return;
      setStatus('disconnected');
      clearTimeout(retry);
      retry = setTimeout(connect, 2000);
    };
    ros.on('connection', onConnection);
    ros.on('error', onError);
    ros.on('close', onClose);
    connect();
    return () => {
      closed = true;
      clearTimeout(retry);
      ros.off('connection', onConnection);
      ros.off('error', onError);
      ros.off('close', onClose);
      ros.close();
    };
  }, [ros, url]);

  return (
    <RosContext.Provider value={{ ros, status, connection, url }}>{children}</RosContext.Provider>
  );
}

export function useRos() {
  const ctx = useContext(RosContext);
  if (!ctx) throw new Error('useRos must be used inside <RosProvider>');
  return ctx;
}

/**
 * Subscribe to a topic while connected. `onMessage` may change on every
 * render; only the topic options re-create the subscription.
 * options: { throttle (ms), compression ('none' | 'cbor'), queueLength }
 */
export function useTopic(name, messageType, onMessage, options = {}) {
  const { ros, connection } = useRos();
  const handler = useRef(onMessage);
  handler.current = onMessage;
  const { throttle = 0, compression = 'none', queueLength = 1, enabled = true } = options;

  useEffect(() => {
    if (!connection || !enabled || !name) return undefined;
    const topic = new ROSLIB.Topic({
      ros, name, messageType, compression, throttle_rate: throttle, queue_length: queueLength,
    });
    const cb = (msg) => handler.current(msg);
    topic.subscribe(cb);
    return () => topic.unsubscribe(cb);
  }, [ros, connection, name, messageType, throttle, compression, queueLength, enabled]);
}

/** Latest message of a topic as React state (for small, slow topics). */
export function useLatest(name, messageType, options) {
  const [msg, setMsg] = useState(null);
  const [time, setTime] = useState(0);
  useTopic(name, messageType, (m) => { setMsg(m); setTime(Date.now()); }, options);
  return [msg, time];
}

/**
 * A publisher advertised as soon as the connection is up. Advertising early
 * matters: a ROS 2 publisher created at the moment of the first message
 * would lose it while DDS discovery is still matching subscribers.
 * Returns publish(message) -> true if sent.
 */
export function usePublisher(name, messageType) {
  const { ros, connection } = useRos();
  const topic = useRef(null);

  useEffect(() => {
    if (!connection) return undefined;
    const t = new ROSLIB.Topic({ ros, name, messageType });
    t.advertise();
    topic.current = t;
    return () => {
      topic.current = null;
      t.unadvertise();
    };
  }, [ros, connection, name, messageType]);

  return (message) => {
    if (!topic.current) return false;
    topic.current.publish(new ROSLIB.Message(message));
    return true;
  };
}
