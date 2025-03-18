"""
Speech Recognition Node for ROS2 Robot

This node listens for basic voice commands and publishes them to a ROS2 topic.
Currently supports 'forward' and 'stop' commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import speech_recognition as sr
import threading
import time
import difflib
from typing import Optional

class SpeechRecognitionNode(Node):
    """ROS2 Node that listens for speech commands and publishes them to a topic."""
    
    def __init__(self):
        """Initialize the speech recognition node."""
        super().__init__('speech_recognition_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('microphone_device_index', 3),
                ('energy_threshold', 300),
                ('dynamic_energy_threshold', True),
                ('pause_threshold', 0.8),
                ('phrase_time_limit', 15.0),
                ('timeout', 10.0),
                ('confidence_threshold', 0.6),
                ('command_cooldown', 2.0),
            ]
        )

        self.mic_device_index = self.get_parameter('microphone_device_index').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        self.dynamic_energy_threshold = self.get_parameter('dynamic_energy_threshold').value
        self.pause_threshold = self.get_parameter('pause_threshold').value
        self.phrase_time_limit = self.get_parameter('phrase_time_limit').value
        self.timeout = self.get_parameter('timeout').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.command_cooldown = self.get_parameter('command_cooldown').value

        # Create a reliable QoS profile for commands
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create a publisher for recognized commands
        self.command_publisher = self.create_publisher(
            String,
            '/robot/voice_command',
            qos_profile
        )
        
        # Create a publisher for energy levels
        self.energy_publisher = self.create_publisher(
            Float32,
            '/robot/mic_energy',
            10
        )
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(device_index=self.mic_device_index)
    
        self.command_mapping = {
            "forward": ["forward", "go forward", "move forward", "ahead", "straight"],
            "backward": ["backward", "back", "backwards", "move backward", "reverse"],
            "left": ["left", "go left", "turn left", "rotate left"],
            "right": ["right", "go right", "turn right", "rotate right"],
            "stop": ["stop", "halt", "halt robot", "freeze", "abort"]
        }

        self.valid_commands = list(self.command_mapping.keys())

        # Flatten the mapping to create lookup for alternative phrasings
        self.phrase_to_command = {}
        for cmd, phrases in self.command_mapping.items():
            for phrase in phrases:
                self.phrase_to_command[phrase] = cmd
        
        # Command history for debouncing and context
        self.last_command = None
        self.last_command_time = 0

        # Configure speech recognition parameters
        self.recognizer.energy_threshold = self.energy_threshold
        self.recognizer.dynamic_energy_threshold = self.dynamic_energy_threshold
        self.recognizer.pause_threshold = self.pause_threshold

        self.get_logger().info(
            f'Speech recognition parameters: energy_threshold={self.recognizer.energy_threshold}, '
            f'dynamic={self.recognizer.dynamic_energy_threshold}, '
            f'pause_threshold={self.recognizer.pause_threshold}s'
        )
        
        self.get_logger().info(f'Valid commands: {", ".join(sorted(self.valid_commands))}')
        
        # Initialize state variables
        self.is_listening = False
        
        # Adjust for ambient noise
        self.setup_microphone()
        
        # Start listening in a separate thread
        self.start_listening()
        
        self.get_logger().info('Speech recognition node initialized and listening!')

    def setup_microphone(self):
        """Adjust microphone for ambient noise levels with proper error handling."""
        try:
            with self.microphone as source:
                self.get_logger().info('Adjusting for ambient noise for 5 seconds... Please be quiet.')
                self.recognizer.adjust_for_ambient_noise(source, duration=5)
                self.get_logger().info(f'Microphone calibrated. Energy threshold set to {self.recognizer.energy_threshold}')
                
                # Publish initial energy level
                self.publish_energy_level(self.recognizer.energy_threshold)
        except Exception as e:
            self.get_logger().error(f'Microphone calibration failed: {e}')
            self.get_logger().warn('Using default energy threshold settings')

    def publish_energy_level(self, energy: float):
        """Publish microphone energy level for diagnostics."""
        msg = Float32()
        msg.data = float(energy)
        self.energy_publisher.publish(msg)

    def start_listening(self):
        """Start the speech recognition in a separate thread with proper error handling."""
        if hasattr(self, 'listening_thread') and self.listening_thread.is_alive():
            self.get_logger().warn('Speech recognition already running')
            return
            
        self.is_listening = True
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        
        try:
            self.listening_thread.start()
            self.get_logger().info('Speech recognition thread started')
        except Exception as e:
            self.get_logger().error(f'Failed to start speech recognition thread: {e}')
            self.is_listening = False

    def calculate_command_confidence(self, text: str, command: str) -> float:
        """Calculate confidence score for a recognized command."""
        # Direct phrase match
        if command in text:
            return 0.9
        
        # Check for command variations
        for phrase in self.command_mapping.get(command, []):
            if phrase in text:
                return 0.85
        
        # String similarity for fuzzy matching
        # Get best match score against any phrase for this command
        best_score = 0.0
        phrases = self.command_mapping.get(command, [command])
        
        for phrase in phrases:
            # Calculate similarity ratio
            ratio = difflib.SequenceMatcher(None, text, phrase).ratio()
            if ratio > best_score:
                best_score = ratio
        
        return best_score
    
    def find_best_command(self, text: str) -> Optional[tuple]:
        """Find the best matching command in the speech text."""
        best_command = None
        best_confidence = 0.0
        
        # Exact phrase matching first
        for phrase, command in self.phrase_to_command.items():
            if phrase in text:
                # Direct phrase match with high confidence
                return (command, 0.9)
        
        # If no exact match, try fuzzy matching for each command
        for command in self.valid_commands:
            confidence = self.calculate_command_confidence(text, command)
            if confidence > best_confidence:
                best_confidence = confidence
                best_command = command
        
        # Only return if confidence is above threshold
        if best_command and best_confidence >= self.confidence_threshold:
            return (best_command, best_confidence)
        
        return None

    def should_debounce(self, command: str) -> bool:
        """Determine if command should be ignored due to recent recognition."""
        current_time = time.time()
        
        # If it's the same command and it's within the cooldown period, debounce it
        if (command == self.last_command and 
            current_time - self.last_command_time < self.command_cooldown):
            return True
            
        return False

    def publish_command(self, command: str, confidence: float):
        """Publish recognized command to ROS2 topic."""
        # Check for debouncing
        if self.should_debounce(command):
            self.get_logger().debug(f'Command "{command}" debounced (cooldown period)')
            return
            
        # Update command history for debouncing
        self.last_command = command
        self.last_command_time = time.time()
        
        # Create and publish message
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        
        self.get_logger().info(f'Published command: "{command}" (confidence: {confidence:.2f})')

    def listen_continuously(self):
        """Continuously listen for speech commands in a separate thread."""
        self.get_logger().info('Starting continuous listening loop')
        
        while rclpy.ok() and self.is_listening:
            try:
                with self.microphone as source:
                    self.get_logger().debug('Listening for commands...')
                    
                    try:
                        audio = self.recognizer.listen(
                            source, 
                            timeout=self.timeout,
                            phrase_time_limit=self.phrase_time_limit
                        )
                        
                        # Process the captured audio
                        try:
                            # Use Google's speech recognition service
                            text = self.recognizer.recognize_google(audio).lower()
                            self.get_logger().info(f'Recognized: "{text}"')
                            
                            # Find best matching command
                            result = self.find_best_command(text)
                            if result:
                                command, confidence = result
                                self.publish_command(command, confidence)
                                
                        except sr.UnknownValueError:
                            self.get_logger().debug('Speech not understood')
                        except sr.RequestError as e:
                            self.get_logger().error(f'Could not request results: {e}')
                        
                    except sr.WaitTimeoutError:
                        # No speech detected within timeout - this is normal
                        pass
                    
            except Exception as e:
                self.get_logger().error(f'Error in speech recognition: {e}')
                time.sleep(1)  # Prevent tight loop in case of repeated errors
        
        self.get_logger().info('Exiting continuous listening loop')

    def cleanup(self):
        """Clean up resources before node shutdown."""
        self.get_logger().info('Shutting down speech recognition...')
        self.is_listening = False
        
        if hasattr(self, 'listening_thread') and self.listening_thread.is_alive():
            self.get_logger().info('Waiting for speech recognition thread to terminate...')
            self.listening_thread.join(timeout=2.0)
        
        self.get_logger().info('Speech recognition node cleaned up')


def main(args=None):
    """Run the speech recognition node."""
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Speech recognition node stopped by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()