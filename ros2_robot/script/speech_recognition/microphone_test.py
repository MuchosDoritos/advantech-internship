#!/usr/bin/env python3
"""
Simple microphone test script for your robot.
This script will test if your microphone is working correctly without requiring ROS2.
"""

import speech_recognition as sr
import time

def test_microphone(device_index=None):
    """Test if the microphone is working and can capture audio."""
    # Initialize recognizer and microphone
    recognizer = sr.Recognizer()
    
    # List available microphones
    print("Available microphones:")
    mic_names = sr.Microphone.list_microphone_names()
    for i, microphone_name in enumerate(mic_names):
        print(f"  {i}: {microphone_name}")
    
    # Use specified microphone or prompt for selection
    if device_index is None:
        print("\nWhich microphone would you like to use? (Enter the number)")
        device_index = int(input("> "))
    
    print(f"\nUsing microphone: {mic_names[device_index]}")
    microphone = sr.Microphone(device_index=device_index)
    
    print("\nTesting selected microphone...")
    print("I'll listen for audio for 10 seconds and report on what I hear.")
    
    # Test microphone by measuring audio level
    with microphone as source:
        print("Adjusting for ambient noise... Be quiet for 2 seconds.")
        recognizer.adjust_for_ambient_noise(source, duration=2)
        print(f"Energy threshold set to {recognizer.energy_threshold}")
        
        print("\nNow make some noise! Listening for 10 seconds...")
        start_time = time.time()
        
        # Listen for audio and report what was heard
        try:
            audio = recognizer.listen(source, timeout=10, phrase_time_limit=15)
            end_time = time.time()
            duration = end_time - start_time
            
            print(f"Audio detected! Captured {len(audio.frame_data)} bytes over {duration:.2f} seconds.")
            print("Microphone test succeeded!")
            
            try:
                # Try to recognize the audio (optional)
                print("\nAttempting to recognize what was said (this might fail and that's OK)...")
                text = recognizer.recognize_google(audio)
                print(f"Recognized: {text}")
            except sr.UnknownValueError:
                print("Couldn't recognize any words, but audio was captured.")
            except sr.RequestError:
                print("Couldn't connect to recognition service, but audio was captured.")
                
        except sr.WaitTimeoutError:
            print("No audio detected. Microphone might not be working correctly.")
            
if __name__ == "__main__":
    # Use Jabra SPEAK 510 USB (device index 3)
    test_microphone(device_index=3)