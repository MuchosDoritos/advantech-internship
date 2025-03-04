import os
from pathlib import Path
import re

def make_labels(image_dir, label_dir):
    class_mapping = {'rock': 0, 'paper': 1, 'scissors': 2}

    for filename in os.listdir(image_dir):
        if filename.endswith('.jpg'):
            print(f"Processing: {filename}")
            
            # Use regex to extract the first occurrence of 'rock', 'paper', or 'scissors'
            match = re.search(r'(rock|paper|scissors)', filename, re.IGNORECASE)
            
            if match:
                class_name = match.group(1).lower()  # Extract class name in lowercase
                
                if class_name in class_mapping:
                    class_index = class_mapping[class_name] 
                else:
                    continue
            

                label_file = label_dir / Path(filename).with_suffix('.txt')

                with open(label_file, 'w') as file:
                    file.write(f"{class_index} 0.5 0.5 1.0 1.0\n")
            else:
                print(f"Skipping file: {filename} (No valid class found)")

def main():
    image_dir = Path('/home/ubuntu/rps-project/valid/images/paper')
    label_dir = Path('/home/ubuntu/rps-project/valid/labels/paper')
    make_labels(image_dir, label_dir)

if __name__ == "__main__":
    main()
