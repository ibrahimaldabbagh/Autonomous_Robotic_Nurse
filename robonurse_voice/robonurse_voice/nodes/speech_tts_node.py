import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
from io import BytesIO
import subprocess
import os

# Create a service (simplified: just subscribes to a topic) 
# For bachelor level simplicity, we use a subscription instead of a service
# to trigger immediate speech output.

class SpeechTTSNode(Node):
    def __init__(self):
        super().__init__('speech_tts_node')
        
        # --- Subscription ---
        self.tts_sub = self.create_subscription(
            String,
            '/speech/say',
            self.tts_callback,
            10
        )
        self.get_logger().info('TTS node ready. Subscribing to /speech/say')

    def tts_callback(self, msg: String):
        """Generates and plays speech from the received text."""
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'TTS synthesizing: "{text}"')
        
        try:
            # 1. Generate the audio data in memory
            tts = gTTS(text=text, lang='en', slow=False)
            mp3_fp = BytesIO()
            tts.write_to_fp(mp3_fp)
            mp3_fp.seek(0)
            
            # 2. Save to a temporary file (required by mpg123)
            temp_file = "/tmp/tts_output.mp3"
            with open(temp_file, "wb") as f:
                f.write(mp3_fp.read())

            # 3. Play the audio using mpg123 (fast command-line player)
            # -q: quiet, -o alsa: default ALSA device
            subprocess.run(["mpg123", "-q", temp_file], check=True)
            
            # 4. Clean up temporary file
            os.remove(temp_file)
            
        except subprocess.CalledProcessError:
            self.get_logger().error("MPG123 not installed or failed to play audio.")
            self.get_logger().warn("Install mpg123: sudo apt install mpg123")
        except Exception as e:
            self.get_logger().error(f"TTS generation or file operation failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SpeechTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()