import React, { useState, useEffect, useCallback, useRef } from 'react';
import { Joystick, Utensils, Syringe, ChevronUp, ChevronDown, RefreshCcw, Wifi } from 'lucide-react'; 
// Using lucide-react for icons

// --- CONFIGURATION ---
const ROSBRIDGE_URL = "ws://ROBOT_IP:9090"; // Assume rosbridge server is running on Jetson Nano
const MAX_LINEAR = 0.5; // m/s
const MAX_ANGULAR = 0.8; // rad/s
const UPDATE_INTERVAL_MS = 100;

// Mock WebSocket implementation for demonstration
const MockWebSocket = {
  readyState: 1, // 1 = OPEN
  onopen: () => console.log("Mock WebSocket opened."),
  onmessage: (e) => console.log("Mock RX:", e.data),
  send: (data) => console.log("Mock TX:", data),
  close: () => console.log("Mock WebSocket closed."),
};

const App = () => {
  // Global State
  const [isConnected, setIsConnected] = useState(false);
  const [mode, setMode] = useState('autonomous'); // 'manual' or 'autonomous'
  const [linearVel, setLinearVel] = useState(0);
  const [angularVel, setAngularVel] = useState(0);
  const [touchActive, setTouchActive] = useState(false);
  const [currentHeadPos, setCurrentHeadPos] = useState('Center');

  // Refs for control and WebSocket
  const wsRef = useRef(MockWebSocket); // Use mock WS for runnable code
  const joystickRef = useRef(null);
  const controlAreaRef = useRef(null);
  
  // ROS 2 Topic Publishers
  const cmdVelTopic = useRef(null);
  const modeTopic = useRef(null);
  const manipulatorTopic = useRef(null);
  const headTopic = useRef(null);

  // --- WebSocket & ROS Bridge Mock Setup ---

  const sendRosMessage = useCallback((topic, data) => {
    const rosMsg = {
        op: 'publish',
        topic: topic,
        msg: { data: data } // Assuming most are std_msgs/String for simple actions
    };
    if (wsRef.current.readyState === 1) {
        wsRef.current.send(JSON.stringify(rosMsg));
        console.log(`ROS TX [${topic}]: ${data}`);
        return true;
    }
    console.error(`ROS TX FAILED [${topic}]: Not connected.`);
    return false;
  }, []);

  useEffect(() => {
    // In a real environment, you would use a library like roslibjs here.
    wsRef.current.onopen = () => {
      setIsConnected(true);
      console.log('Connected to ROS Bridge!');
      
      // Define publishers (Twist for command, String for mode, etc.)
      cmdVelTopic.current = (lin, ang) => {
        const twistMsg = {
          op: 'publish',
          topic: '/joy_vel', 
          msg: {
            linear: { x: lin, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: ang }
          }
        };
        wsRef.current.send(JSON.stringify(twistMsg));
      };

      modeTopic.current = (newMode) => sendRosMessage('/robot/mode_switch', newMode);
      // New topics for specific actions
      manipulatorTopic.current = (action) => sendRosMessage('/manipulator/control', action);
      headTopic.current = (position) => sendRosMessage('/head/control', position);

      // Mock connection success
      setTimeout(() => setIsConnected(true), 100); 
    };

    wsRef.current.onopen();

    return () => {
      wsRef.current.close();
    };
  }, [sendRosMessage]);

  // --- Teleoperation Loop ---
  useEffect(() => {
    if (!isConnected || mode !== 'manual') return;

    const interval = setInterval(() => {
      if (cmdVelTopic.current) {
        // Publish current velocity state
        cmdVelTopic.current(linearVel, angularVel);
      }
    }, UPDATE_INTERVAL_MS);

    return () => clearInterval(interval);
  }, [isConnected, mode, linearVel, angularVel]);

  // --- Mode Switching ---
  const handleModeSwitch = useCallback((newMode) => {
    setMode(newMode);
    if (modeTopic.current) {
      modeTopic.current(newMode);
    }
    // Stop robot immediately when switching out of manual mode
    if (newMode !== 'manual') {
      setLinearVel(0);
      setAngularVel(0);
      if (cmdVelTopic.current) {
        cmdVelTopic.current(0, 0);
      }
    }
  }, []);

  // --- Joystick Logic (Mouse/Touch) ---
  const calculateVelocity = useCallback((clientX, clientY) => {
    if (!controlAreaRef.current) return;
    const rect = controlAreaRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    const x = clientX - centerX; // Horizontal deviation
    const y = centerY - clientY; // Vertical deviation (Y axis is inverted in screen coords)

    const maxRadius = Math.min(rect.width, rect.height) / 2;

    // Normalize deviation to [-1, 1] range
    const normX = Math.min(Math.max(x / maxRadius, -1), 1);
    const normY = Math.min(Math.max(y / maxRadius, -1), 1);

    // X (Strafe) is used for angular velocity (turning)
    // Y (Forward/Back) is used for linear velocity
    const lin = normY * MAX_LINEAR;
    const ang = -normX * MAX_ANGULAR; 

    // Update state and joystick position
    setLinearVel(lin);
    setAngularVel(ang);
    if (joystickRef.current) {
      // Limit joystick handle movement to the base radius
      const distance = Math.sqrt(x*x + y*y);
      if (distance > maxRadius) {
        // Scale back to edge
        const ratio = maxRadius / distance;
        joystickRef.current.style.transform = `translate(${x * ratio}px, ${y * ratio}px)`;
      } else {
        joystickRef.current.style.transform = `translate(${x}px, ${y}px)`;
      }
    }
  }, []);

  const handleStart = useCallback((e) => {
    if (mode === 'manual') {
      e.preventDefault();
      setTouchActive(true);
      const clientX = e.touches ? e.touches[0].clientX : e.clientX;
      const clientY = e.touches ? e.touches[0].clientY : e.clientY;
      calculateVelocity(clientX, clientY);
    }
  }, [mode, calculateVelocity]);

  const handleMove = useCallback((e) => {
    if (touchActive && mode === 'manual') {
      e.preventDefault();
      const clientX = e.touches ? e.touches[0].clientX : e.clientX;
      const clientY = e.touches ? e.touches[0].clientY : e.clientY;
      calculateVelocity(clientX, clientY);
    }
  }, [touchActive, mode, calculateVelocity]);

  const handleEnd = useCallback(() => {
    setTouchActive(false);
    setLinearVel(0);
    setAngularVel(0);
    if (joystickRef.current) {
      joystickRef.current.style.transform = `translate(0px, 0px)`;
    }
    // Send final zero velocity command to stop robot quickly
    if (cmdVelTopic.current) {
        cmdVelTopic.current(0, 0);
    }
  }, []);

  // Attach global touch/mouse handlers
  useEffect(() => {
    if (mode === 'manual') {
      document.addEventListener('mousemove', handleMove);
      document.addEventListener('mouseup', handleEnd);
      document.addEventListener('touchmove', handleMove, { passive: false });
      document.addEventListener('touchend', handleEnd);
    }
    return () => {
      document.removeEventListener('mousemove', handleMove);
      document.removeEventListener('mouseup', handleEnd);
      document.removeEventListener('touchmove', handleMove);
      document.removeEventListener('touchend', handleEnd);
    };
  }, [mode, handleMove, handleEnd]);

  // --- Manipulator/Head Controls ---
  const handleManipulatorAction = (action) => {
    if (mode === 'manual' && manipulatorTopic.current) {
      manipulatorTopic.current(action);
    }
  };

  const handleHeadAction = (position) => {
    if (mode === 'manual' && headTopic.current) {
      headTopic.current(position);
      setCurrentHeadPos(position);
    }
  };


  const statusColor = isConnected ? 'bg-green-500' : 'bg-red-500';

  return (
    <div className="min-h-screen bg-gray-900 text-white font-sans flex flex-col p-4 sm:p-8">
      <script src="https://cdn.tailwindcss.com"></script>
      <style>{`
        /* Theming: RoboNurse Blue/Indigo */
        .robonurse-primary { background-color: #4338ca; } /* Indigo 700 */
        .robonurse-accent { color: #6366f1; } /* Indigo 500 */
        .joystick-base {
          position: relative;
          width: 300px;
          height: 300px;
          border-radius: 50%;
          border: 4px solid #4f46e5;
          box-shadow: 0 0 20px rgba(79, 70, 229, 0.5);
          display: flex;
          justify-content: center;
          align-items: center;
          touch-action: none; 
          margin-top: 2rem;
        }
        .joystick-handle {
          width: 80px;
          height: 80px;
          border-radius: 50%;
          background: linear-gradient(145deg, #6366f1, #4f46e5);
          position: absolute;
          transition: transform 0.05s; /* Faster response */
          cursor: grab;
          box-shadow: 0 4px 10px rgba(0, 0, 0, 0.5);
          display: flex;
          justify-content: center;
          align-items: center;
        }
        .control-button {
            transition: all 0.2s;
            box-shadow: 0 4px #1e293b; /* Shadow for pressed effect */
        }
        .control-button:active {
            box-shadow: 0 2px #1e293b;
            transform: translateY(2px);
        }
      `}</style>
      
      {/* Header */}
      <div className="flex justify-between items-center mb-6 pb-2 border-b border-indigo-700">
        <h1 className="text-3xl font-extrabold tracking-widest robonurse-accent">ROBONURSE OPERATOR HMI</h1>
        <div className="flex items-center space-x-2">
          <Wifi className={isConnected ? "text-green-400" : "text-red-400"} size={20}/>
          <span className="text-sm font-medium">{isConnected ? 'Connected' : 'Disconnected'}</span>
        </div>
      </div>

      {/* Mode Switch */}
      <div className="flex justify-center mb-8">
        <button
          onClick={() => handleModeSwitch('manual')}
          className={`px-6 py-3 font-semibold rounded-l-xl transition duration-300 ${
            mode === 'manual' ? 'robonurse-primary shadow-lg shadow-indigo-500/50' : 'bg-gray-700 hover:bg-gray-600'
          }`}
        >
          Manual Control (Teleop)
        </button>
        <button
          onClick={() => handleModeSwitch('autonomous')}
          className={`px-6 py-3 font-semibold rounded-r-xl transition duration-300 ${
            mode === 'autonomous' ? 'robonurse-primary shadow-lg shadow-indigo-500/50' : 'bg-gray-700 hover:bg-gray-600'
          }`}
        >
          Autonomous Mode
        </button>
      </div>

      {/* Main Control Layout: Joystick (Left/Center) and Manipulator/Head Controls (Right) */}
      <div className="flex-grow grid grid-cols-1 lg:grid-cols-3 gap-8 items-start">
        
        {/* Column 1: Joystick / Drive Control */}
        <div className="lg:col-span-2 flex flex-col items-center p-4 bg-gray-800 rounded-2xl shadow-xl">
            <h2 className="text-xl font-bold mb-4 robonurse-accent uppercase">Base Mobility</h2>
            <div className="text-lg font-medium mb-4">
                Mode: <span className={`font-extrabold ${mode === 'manual' ? 'text-red-400' : 'text-green-400'}`}>{mode.toUpperCase()}</span>
            </div>

            {/* Joystick Control */}
            <div
                ref={controlAreaRef}
                className={`joystick-base ${mode !== 'manual' ? 'opacity-30 pointer-events-none' : 'cursor-pointer'}`}
                onMouseDown={handleStart}
                onTouchStart={handleStart}
            >
                <div 
                    ref={joystickRef} 
                    className="joystick-handle"
                >
                    <Joystick size={32} color="#f9fafb" />
                </div>
                <div className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-full h-full border-2 border-dashed border-indigo-500/50 rounded-full flex justify-center items-center text-center text-indigo-300 text-lg">
                    {mode === 'manual' ? 'Drag to Move' : 'Autonomous Control'}
                </div>
            </div>

            {/* Diagnostic Overlay */}
            <div className="mt-6 text-sm font-mono flex justify-around w-full max-w-sm p-2 bg-gray-700 rounded-lg">
                <div>Lin: <span className="text-yellow-400">{linearVel.toFixed(2)} m/s</span></div>
                <div>Ang: <span className="text-yellow-400">{angularVel.toFixed(2)} rad/s</span></div>
            </div>
        </div>

        {/* Column 2 & 3: Manipulator and Head Controls */}
        <div className="lg:col-span-1 flex flex-col space-y-8 p-4 bg-gray-800 rounded-2xl shadow-xl h-full">

            {/* Manipulator Controls (Loading Actions) */}
            <div>
                <h2 className="text-xl font-bold mb-4 robonurse-accent uppercase border-b border-gray-700 pb-2">Loading Actions</h2>
                <div className="space-y-4">
                    <button
                        onClick={() => handleManipulatorAction('load_food')}
                        disabled={mode !== 'manual'}
                        className={`control-button w-full flex items-center justify-center p-4 rounded-xl text-lg font-semibold transition ${
                            mode === 'manual' ? 'bg-green-600 hover:bg-green-700' : 'bg-gray-700 text-gray-500 cursor-not-allowed'
                        }`}
                    >
                        <Utensils size={24} className="mr-3" /> Load Food Plate
                    </button>
                    <button
                        onClick={() => handleManipulatorAction('load_meds')}
                        disabled={mode !== 'manual'}
                        className={`control-button w-full flex items-center justify-center p-4 rounded-xl text-lg font-semibold transition ${
                            mode === 'manual' ? 'bg-orange-600 hover:bg-orange-700' : 'bg-gray-700 text-gray-500 cursor-not-allowed'
                        }`}
                    >
                        <Syringe size={24} className="mr-3" /> Load Medicine
                    </button>
                </div>
                <p className="text-xs mt-2 text-gray-400">Triggers specific, pre-programmed loading/unloading sequences.</p>
            </div>

            {/* Head Movement Controls */}
            <div>
                <h2 className="text-xl font-bold mb-4 robonurse-accent uppercase border-b border-gray-700 pb-2">Head/Camera Control</h2>
                <div className="grid grid-cols-3 gap-4">
                    <button
                        onClick={() => handleHeadAction('Up')}
                        disabled={mode !== 'manual'}
                        className={`control-button col-span-1 p-3 rounded-xl text-lg font-semibold transition ${
                            mode === 'manual' ? 'bg-indigo-500 hover:bg-indigo-600' : 'bg-gray-700 text-gray-500 cursor-not-allowed'
                        }`}
                    >
                        <ChevronUp size={24} className="mx-auto" />
                    </button>
                    <button
                        onClick={() => handleHeadAction('Center')}
                        disabled={mode !== 'manual'}
                        className={`control-button col-span-1 p-3 rounded-xl text-lg font-semibold transition ${
                            mode === 'manual' ? 'bg-indigo-700 hover:bg-indigo-800' : 'bg-gray-700 text-gray-500 cursor-not-allowed'
                        }`}
                    >
                        <RefreshCcw size={24} className="mx-auto" />
                    </button>
                    <button
                        onClick={() => handleHeadAction('Down')}
                        disabled={mode !== 'manual'}
                        className={`control-button col-span-1 p-3 rounded-xl text-lg font-semibold transition ${
                            mode === 'manual' ? 'bg-indigo-500 hover:bg-indigo-600' : 'bg-gray-700 text-gray-500 cursor-not-allowed'
                        }`}
                    >
                        <ChevronDown size={24} className="mx-auto" />
                    </button>
                </div>
                <p className="text-sm mt-3 text-gray-300">Current Position: <span className="font-bold text-yellow-400">{currentHeadPos}</span></p>
                <p className="text-xs mt-1 text-gray-400">Controls the camera/head tilt for better situational awareness.</p>
            </div>
            
        </div>
      </div>
    </div>
  );
};

export default App;