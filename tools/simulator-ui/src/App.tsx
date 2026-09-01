import React, { useEffect, useState, useRef } from 'react';
import { io, Socket } from 'socket.io-client';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';

export default function App() {
  const [telemetry, setTelemetry] = useState<any>(null);
  const [command, setCommand] = useState({
    lx: 0, ly: 0, lz: 0,
    ax: 0, ay: 0, az: 0
  });
  const socketRef = useRef<Socket | null>(null);

  useEffect(() => {
    // Connect to Node.js proxy server
    socketRef.current = io('http://localhost:3001');

    socketRef.current.on('telemetry', (data) => {
      setTelemetry(data);
    });

    return () => {
      socketRef.current?.disconnect();
    };
  }, []);

  const sendCommand = (newCommand: any) => {
    setCommand(newCommand);
    if (socketRef.current) {
      socketRef.current.emit('command', newCommand);
    }
  };

  return (
    <div style={{ display: 'flex', width: '100%', height: '100%' }}>
      {/* 3D Visualization */}
      <div style={{ flex: 1, position: 'relative' }}>
        <Canvas camera={{ position: [2, 2, 2] }}>
          <ambientLight intensity={0.5} />
          <pointLight position={[10, 10, 10]} />
          <Grid infiniteGrid fadeDistance={10} sectionColor="gray" cellColor="darkgray" />
          <OrbitControls />
          
          {/* Render the drone dynamically based on telemetry/config */}
          {telemetry && (
            <group 
              position={[telemetry.position[0], telemetry.position[1], -telemetry.position[2]]} // Convert NED to standard WebGL Z-up
              rotation={[telemetry.euler_angles[0], telemetry.euler_angles[1], -telemetry.euler_angles[2]]}
            >
              {/* Center Body */}
              <mesh>
                <boxGeometry args={[0.2, 0.2, 0.1]} />
                <meshStandardMaterial color="orange" />
              </mesh>

              {/* Rotors based on config */}
              {telemetry.rotors && telemetry.rotors.map((r: any, idx: number) => (
                <mesh key={idx} position={[r.position[0], r.position[1], -r.position[2]]}>
                  <cylinderGeometry args={[0.08, 0.08, 0.02, 16]} />
                  <meshStandardMaterial color="lightgray" transparent opacity={0.7} />
                </mesh>
              ))}
            </group>
          )}
        </Canvas>
      </div>

      {/* UI Sidebar */}
      <div style={{ width: '350px', backgroundColor: '#222', padding: '20px', overflowY: 'auto' }}>
        <h2>Simulation UI</h2>
        
        <div style={{ marginBottom: '20px', padding: '10px', background: '#333', borderRadius: '8px' }}>
          <h3>Telemetry Readout</h3>
          {telemetry ? (
            <div style={{ fontSize: '14px', fontFamily: 'monospace' }}>
              <div><b>Pos:</b> {telemetry.position.map((v: number) => v.toFixed(2)).join(', ')}</div>
              <div><b>Vel:</b> {telemetry.velocity.map((v: number) => v.toFixed(2)).join(', ')}</div>
              <div><b>Ang:</b> {telemetry.euler_angles.map((v: number) => v.toFixed(2)).join(', ')}</div>
              <hr/>
              <div><b>Voltage:</b> {telemetry.battery_voltage.toFixed(2)}V</div>
              <div><b>Current:</b> {telemetry.current_draw.toFixed(2)}A</div>
              <div><b>Consumed:</b> {telemetry.consumed_capacity.toFixed(2)}mAh</div>
            </div>
          ) : <div>Waiting for telemetry...</div>}
        </div>

        <div style={{ marginBottom: '20px', padding: '10px', background: '#333', borderRadius: '8px' }}>
          <h3>Command Setpoint (6D)</h3>
          {Object.entries(command).map(([key, val]) => (
            <div key={key} style={{ display: 'flex', alignItems: 'center', marginBottom: '8px' }}>
              <label style={{ width: '30px' }}>{key}</label>
              <input 
                type="range" 
                min="-1" max="1" step="0.01" 
                value={val} 
                onChange={(e) => sendCommand({ ...command, [key]: parseFloat(e.target.value) })}
                style={{ flex: 1, margin: '0 10px' }}
              />
              <span style={{ width: '40px', fontSize: '12px' }}>{val.toFixed(2)}</span>
            </div>
          ))}
          <button 
            style={{ width: '100%', padding: '10px', marginTop: '10px', cursor: 'pointer' }}
            onClick={() => sendCommand({ lx: 0, ly: 0, lz: 0, ax: 0, ay: 0, az: 0 })}
          >
            Zero Setpoints
          </button>
        </div>

        <div style={{ padding: '10px', background: '#333', borderRadius: '8px' }}>
          <h3>Hardware Config</h3>
          <p style={{ fontSize: '12px', color: '#ccc' }}>
            Future: Add inputs here to send "config" messages to dynamically change mass, coefficients, etc.
          </p>
          <button 
            style={{ width: '100%', padding: '10px', cursor: 'pointer' }}
            onClick={() => {
              socketRef.current?.emit('config', { mass: 1.5 });
            }}
          >
            Test Config Update
          </button>
        </div>
      </div>
    </div>
  );
}
