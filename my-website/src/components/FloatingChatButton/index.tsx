/**
 * FloatingChatButton - Floating chat button that appears on all pages
 */

import React, { useState } from "react";
import ChatWidget from "../Chat";
import styles from "./styles.module.css";

const RiRobot2Fill = ({ size = 24, color = "#f63bf0ff" }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    stroke={color}
    strokeWidth="1.5"
    strokeLinecap="round"
    strokeLinejoin="round"
    className="drop-shadow-lg"
  >
    {/* Sleek rounded head with subtle antenna */}
    <rect x="4" y="5" width="16" height="15" rx="5" strokeWidth="2" />
    
    {/* Glowing visor band */}
    <rect x="5" y="10" width="14" height="4" rx="2" fill={color} opacity="0.2" />
    <rect x="5" y="10" width="14" height="1" rx="0.5" fill={color} />

    {/* Modern glowing eyes */}
    <circle cx="9" cy="9" r="2.5" fill={color} opacity="0.9" />
    <circle cx="9" cy="9" r="1.5" fill="#fff" opacity="0.9" />
    
    <circle cx="15" cy="9" r="2.5" fill={color} opacity="0.9" />
    <circle cx="15" cy="9" r="1.5" fill="#fff" opacity="0.9" />

    {/* Antenna / side details */}
    <circle cx="3" cy="10" r="1.5" fill={color} opacity="0.6" />
    <circle cx="21" cy="10" r="1.5" fill={color} opacity="0.6" />

    {/* Neck connectors */}
    <line x1="9" y1="19" x2="9" y2="22" strokeWidth="2" />
    <line x1="15" y1="19" x2="15" y2="22" strokeWidth="2" />

    {/* Subtle mouth grill */}
    <path d="M8 16h8" strokeWidth="1" opacity="0.5" />
    <path d="M8 17h8" strokeWidth="1" opacity="0.3" />
  </svg>
); 

const FloatingChatButton: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };
  return (
    <>
      {/* Floating Chat Window */}
      {isOpen && ( // Use isChatOpen
        <div className={styles.floatingChatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.headerTitle}>
              <RiRobot2Fill size={24} color="#C6613F" />
              <h3>Assistant</h3>
            </div>
            <button className={styles.closeButton} onClick={toggleChat}>
              ×
            </button>
          </div>
          <div className={styles.chatContent}>
            <ChatWidget />
          </div>
        </div>
      )}

      {/* Floating Button */}
      {
        // Only show FAB if chat is not open
        <button
          className={styles.floatingButton}
          onClick={toggleChat}
          aria-label="Open AI chat assistant"
          title="Ask AI Assistant"
        >
          <RiRobot2Fill size={32} />
        </button>
      }
    </>
  );
};

export default FloatingChatButton;
