import React, { useState, useEffect, useCallback, useRef } from 'react';
import { Phone, Utensils, MessageCircle, AlertTriangle, User, Mail, Zap, Hospital, Clock } from 'lucide-react';

// --- CONFIGURATION ---
const ROSBRIDGE_URL = "ws://ROBOT_IP:9090";

// Mock WebSocket implementation for demonstration
const MockWebSocket = {
  readyState: 1, // 1 = OPEN
  onopen: () => console.log("Patient Mock WS opened."),
  onmessage: (e) => console.log("Patient Mock RX:", e.data),
  send: (data) => console.log("Patient Mock TX:", data),
  close: () => console.log("Patient Mock WS closed."),
};

// Services available to the patient
const services = [
  { name: "Emergency Call", icon: AlertTriangle, color: "bg-red-600", topic: "/patient/emergency" },
  { name: "Video/Phone Call", icon: Phone, color: "bg-indigo-600", topic: "/patient/call" },
  { name: "Hospital Cafe Order", icon: Utensils, color: "bg-green-600", topic: "/patient/order_cafe" },
  { name: "Send Email", icon: Mail, color: "bg-yellow-600", topic: "/patient/send_email" },
  { name: "Request Nurse", icon: User, color: "bg-purple-600", topic: "/patient/request_nurse" },
  { name: "Check Services Status", icon: Zap, color: "bg-gray-600", topic: "/patient/status_check" },
];

const App = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [message, setMessage] = useState('');
  const [isAlerting, setIsAlerting] = useState(false);
  const wsRef = useRef(MockWebSocket);

  // Function to mock sending a ROS message
  const publishServiceRequest = useCallback((topic, payload = {}) => {
    const rosMsg = {
      op: 'publish',
      topic: topic,
      msg: { data: JSON.stringify(payload) } // General purpose JSON payload
    };
    if (wsRef.current.readyState === 1) {
      wsRef.current.send(JSON.stringify(rosMsg));
      console.log(`ROS TX [${topic}]: ${JSON.stringify(payload)}`);
      return true;
    }
    console.error(`ROS TX FAILED [${topic}]: Not connected.`);
    return false;
  }, []);

  // --- WebSocket & ROS Bridge Mock Setup ---
  useEffect(() => {
    wsRef.current.onopen = () => {
      setIsConnected(true);
      console.log('Patient App Connected.');
    };

    wsRef.current.onopen();
    
    // Simulate incoming status messages from the robot system
    const interval = setInterval(() => {
        setMessage(`Hello, Patient John Doe! Time: ${new Date().toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit' })}. Robot Status: Ready.`);
    }, 5000);

    return () => {
      wsRef.current.close();
      clearInterval(interval);
    };
  }, []);

  // --- Service Handler ---
  const handleServiceClick = (service) => {
    const timestamp = new Date().toLocaleTimeString();
    
    let payload = {
        timestamp: timestamp,
        service_name: service.name,
        patient_id: "Hosp_P123" // Mock Patient ID
    };

    if (service.name === "Emergency Call") {
        setIsAlerting(true);
        payload.type = "CRITICAL";
    } else if (service.name === "Hospital Cafe Order") {
        // Mock payload for an order
        payload.details = "Requesting standard menu link.";
    }

    const success = publishServiceRequest(service.topic, payload);

    if (success) {
        setMessage(`${service.name} request sent at ${timestamp}. Staff are being notified.`);
    } else {
        setMessage(`ERROR: Could not connect to robot server. Check network.`);
    }

    // Auto-clear non-emergency messages after a delay
    if (service.name !== "Emergency Call") {
        setTimeout(() => setMessage('Please select a service below.'), 5000); 
    }
  };

  // --- Emergency Stop UI ---
  const stopEmergency = () => {
      setIsAlerting(false);
      setMessage("Emergency alert cancelled. Staff have been notified of cancellation.");
      // Publish cancellation message to ROS topic
      publishServiceRequest("/patient/emergency_cancel", {patient_id: "Hosp_P123"});
      setTimeout(() => setMessage('Please select a service below.'), 5000);
  };

  const statusColor = isConnected ? 'bg-green-500' : 'bg-red-500';
  const accentColor = 'text-indigo-700';
  const accentBg = 'bg-indigo-600';

  return (
    <div className="min-h-screen bg-gray-50 font-sans flex flex-col p-4 sm:p-8">
      <script src="https://cdn.tailwindcss.com"></script>
      <style>{`
        /* Custom responsive grid for large buttons */
        .service-grid {
          display: grid;
          grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
          gap: 20px;
        }
        /* Shake animation for emergency */
        @keyframes shake {
          0%, 100% { transform: translateX(0); }
          20%, 60% { transform: translateX(-8px); }
          40%, 80% { transform: translateX(8px); }
        }
        .emergency-shake {
          animation: shake 0.5s infinite;
        }
        .service-button {
            transition: all 0.3s;
            box-shadow: 0 8px 15px rgba(0,0,0,0.1);
        }
        .service-button:hover {
            box-shadow: 0 10px 20px rgba(0,0,0,0.2);
            transform: translateY(-2px);
        }
        .robonurse-title {
            color: #4338ca; /* Indigo 700 */
        }
      `}</style>
      
      {/* Header */}
      <header className="mb-8 p-4 bg-white rounded-xl shadow-2xl flex justify-between items-center border-b-4 border-indigo-500">
        <h1 className="text-4xl font-extrabold robonurse-title flex items-center">
            <Hospital size={36} className="mr-3 text-red-500"/>
            RoboNurse Patient HMI
        </h1>
        <div className="flex items-center space-x-2">
          <Clock size={20} className="text-gray-500"/>
          <span className="text-base font-medium text-gray-700">{isConnected ? 'Online' : 'Offline'}</span>
        </div>
      </header>

      {/* Notification/Status Bar */}
      <div className={`p-5 mb-8 rounded-xl font-semibold text-center transition-all duration-500 shadow-lg ${isAlerting ? 'bg-red-100 text-red-800 border-4 border-red-500 emergency-shake' : 'bg-white text-gray-700'}`}>
        <MessageCircle className="inline-block mr-2" size={24} />
        {message || "Welcome! Select a service to begin."}
        {isAlerting && (
            <button 
                onClick={stopEmergency} 
                className="ml-6 px-6 py-2 text-base font-bold bg-red-700 text-white rounded-full hover:bg-red-800 transition shadow-xl"
            >
                CANCEL EMERGENCY ALERT
            </button>
        )}
      </div>

      {/* Services Grid */}
      <div className="service-grid flex-grow">
        {services.map((service) => (
          <button
            key={service.name}
            onClick={() => handleServiceClick(service)}
            className={`service-button flex flex-col items-center justify-center p-8 rounded-3xl text-white transform transition duration-300 ${service.color}`}
            style={{minHeight: '180px'}}
          >
            <service.icon size={56} strokeWidth={2.5} />
            <span className="mt-4 text-xl font-bold text-center">{service.name}</span>
          </button>
        ))}
      </div>
      
      {/* Footer */}
      <footer className="mt-8 pt-4 text-center text-sm text-gray-500 border-t border-gray-300">
          RoboNurse System - Dedicated to Patient Care and Safety.
      </footer>
    </div>
  );
};

export default App;