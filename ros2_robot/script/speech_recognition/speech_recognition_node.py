"""
Speech Recognition Node for ROS2 Robot

This node listens for basic voice commands and publishes them to a ROS2 topic.
Currently supports 'forward' and 'stop' commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading
import time

class SpeechRecognitionNode(Node):
    """ROS2 Node that listens for speech commands and publishes them to a topic."""
    
    def __init__(self):
        """Initialize the speech recognition node."""
        super().__init__('speech_recognition_node')
        
        # Create a publisher for recognized commands
        self.command_publisher = self.create_publisher(
            String,
            '/robot/voice_command',
            10
        )
        
        # Initialize the speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Configure speech recognition parameters
        self.recognizer.energy_threshold = 300
        self.recognizer.dynamic_energy_threshold = True
        
        # List of supported commands
        self.commands = ["forward", "backward", "left", "right", "stop"]        
        # Track recognition state
        self.is_listening = False
        
        # Adjust for ambient noise
        self.setup_microphone()
        
        # Start listening in a separate thread
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        self.listening_thread.start()
        
        self.get_logger().info('Speech recognition node initialized and listening!')

    def setup_microphone(self):
        """Adjust microphone for ambient noise levels."""
        try:
            with self.microphone as source:
                self.get_logger().info('Adjusting for ambient noise... Please be quiet.')
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
                self.get_logger().info('Microphone calibrated. Ready to listen!')
        except Exception as e:
            self.get_logger().error(f'Microphone setup failed: {e}')
            self.get_logger().warn('Continuing with default settings')

    def listen_continuously(self):
        """Continuously listen for speech commands in a separate thread."""
        self.is_listening = True
        
        while rclpy.ok() and self.is_listening:
            try:
                with self.microphone as source:
                    self.get_logger().debug('Listening for commands...')
                    audio = self.recognizer.listen(
                        source, 
                        timeout=1.0, 
                        phrase_time_limit=5.0
                    )
                
                try:
                    # Use Google's speech recognition service
                    text = self.recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f'Recognized: "{text}"')
                    
                    # Check if the recognized text contains any of our commands
                    for command in self.commands:
                        if command in text:
                            self.publish_command(command)
                            break
                            
                except sr.UnknownValueError:
                    self.get_logger().debug('Speech not understood')
                except sr.RequestError as e:
                    self.get_logger().error(f'Could not request results: {e}')
                    
            except Exception as e:
                self.get_logger().error(f'Error in speech recognition: {e}')
                time.sleep(1)  # Prevent tight loop in case of repeated errors
    
    def publish_command(self, command):
        """Publish recognized command to ROS2 topic."""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Published command: "{command}"')
    
    def cleanup(self):
        """Clean up resources before node shutdown."""
        self.is_listening = False
        if hasattr(self, 'listening_thread') and self.listening_thread.is_alive():
            self.listening_thread.join(timeout=1.0)
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