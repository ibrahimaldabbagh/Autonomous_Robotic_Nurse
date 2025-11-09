import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import json
from vosk import Model, KaldiRecognizer, list_devices

# --- Configuration ---
MODEL_PATH = "/path/to/your/vosk-model" # CHANGE THIS PATH
CHANNELS = 1
RATE = 16000 # Standard sample rate for speech recognition
CHUNK = 1024 # Buffer size

class SpeechASRNode(Node):
    def __init__(self):
        super().__init__('speech_asr_node')
        
        # --- Publishers ---
        self.asr_pub = self.create_publisher(String, '/speech/text', 10)
        
        # --- ASR Initialization (VOSK) ---
        try:
            self.model = Model(MODEL_PATH)
            self.recognizer = KaldiRecognizer(self.model, RATE)
            self.get_logger().info('VOSK model loaded successfully.')
        except Exception as e:
            self.get_logger().error(f"Failed to load VOSK model: {e}. Check MODEL_PATH.")
            self.model = None

        # --- Audio Stream Setup (PyAudio) ---
        self.p = pyaudio.PyAudio()
        
        # Find the correct microphone input device
        try:
            input_device_index = self._find_mic_device()
            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK,
                input_device_index=input_device_index
            )
            self.get_logger().info(f'Audio stream opened on device index {input_device_index}.')
        except Exception as e:
            self.get_logger().error(f"Could not open audio stream: {e}")
            self.stream = None
        
        # --- Timer for reading audio data ---
        self.timer = self.create_timer(CHUNK / RATE, self.audio_callback)

    def _find_mic_device(self):
        """Tries to find a suitable microphone device index."""
        try:
            # Simple check: try to find the default input device
            default_index = self.p.get_default_input_device_info()['index']
            return default_index
        except Exception:
            self.get_logger().warn("Could not find default input device. Defaulting to index 0.")
            return 0 # Fallback

    def audio_callback(self):
        """Reads audio data and feeds it to the VOSK recognizer."""
        if self.stream is None or self.model is None:
            return

        try:
            data = self.stream.read(CHUNK, exception_on_overflow=False)
        except IOError as e:
            # Handle potential buffer overflow or I/O error
            self.get_logger().error(f"Audio read error: {e}")
            return

        if self.recognizer.AcceptWaveform(data):
            # Recognition result is final
            result_json = json.loads(self.recognizer.Result())
            text = result_json.get('text', '')
            
            if text:
                msg = String()
                msg.data = text.lower() # Standardize to lowercase
                self.asr_pub.publish(msg)
                self.get_logger().info(f'ASR recognized: "{text}"')
        else:
            # Recognition is partial (can be ignored for simplicity)
            pass

    def destroy_node(self):
        """Cleanup resources."""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechASRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()