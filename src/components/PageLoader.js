import React from 'react';
import { FaRobot } from 'react-icons/fa';

export default function PageLoader() {
  return (
    <div style={{
      position: 'fixed',
      top: 0,
      left: 0,
      width: '100vw',
      height: '100vh',
      zIndex: 10000,
      backgroundColor: '#000000',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center'
    }}>
      <FaRobot style={{
        fontSize: '80px',
        color: '#00D9FF',
        animation: 'pulse 1.5s ease-in-out infinite'
      }} />
    </div>
  );
}
