import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading
import time

class SpeechRecognitionNode(Node):
    def __init__(self):
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
        
                # Adjust for ambient noise
        with self.microphone as source:
            self.get_logger().info('Adjusting for ambient noise...')
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info('Ready to listen!')

        # Start listening in a separate thread
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        self.listening_thread.start()
        
        # List of commands to recognize
        self.commands = ["stop"];

                        
    def listen_continuously(self):
        """Continuously listen for speech commands in a separate thread"""
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().debug('Listening for commands...')
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)
                
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
        """Publish recognized command to ROS2 topic"""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Published command: "{command}"')

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Speech recognition node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()