#!/home/lucas/catkin_ws/src/edurobot/myenv/bin/python3

import rospy
import sounddevice as sd
import numpy as np
import tempfile
import subprocess
import whisper
from scipy.io.wavfile import write
from std_msgs.msg import String, Bool


class VoiceTriggerNode:
    def __init__(self):
        rospy.init_node('stt_node')
        self.pub = rospy.Publisher('/voice_trigger', String, queue_size=10)
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=1)

        # Optional TTS status (recommended)
        self.tts_busy = False
        self.tts_status_seen = False
        rospy.Subscriber('/tts_busy', Bool, self._tts_busy_cb)

        # Load Whisper small model (offline)
        rospy.loginfo("🔊 Loading Whisper small model...")
        self.model = whisper.load_model("small")

        # Mic settings
        self.sample_rate = 16000  # Whisper expects 16kHz
        devices = sd.query_devices()
        if not devices:
            rospy.logerr("❌ No microphone detected.")
            raise RuntimeError("No microphone found.")
        rospy.loginfo(f"🎙️ Detected audio devices: {devices}")

        rospy.Subscriber("/ask_stop_follow", String, self.ask_callback)

        self.waiting_for_answer = False
        self.keep_listening = True

    # ---------- TTS coordination ----------

    def _tts_busy_cb(self, msg: Bool):
        self.tts_busy = bool(msg.data)
        self.tts_status_seen = True

    def say_and_wait(self, text: str, fallback_wpm: int = 160, extra_pad_s: float = 0.35, max_wait_s: float = 20.0):
        """
        Publish TTS and block until TTS finishes.
        If /tts_busy is available, wait on it. Otherwise, estimate duration from text length.
        """
        # Publish TTS request
        self.tts_pub.publish(text)
        rospy.loginfo(f"🤖 Saying: {text}")

        if self.tts_status_seen:
            # Wait briefly for TTS to enter 'busy' (some nodes set busy immediately, some with a tiny delay)
            r = rospy.Rate(100)
            start = rospy.Time.now()
            wait_until = start + rospy.Duration(1.0)
            while not rospy.is_shutdown() and not self.tts_busy and rospy.Time.now() < wait_until:
                r.sleep()

            # Now wait until TTS finishes (busy -> False), with a safety timeout
            deadline = start + rospy.Duration(max_wait_s)
            while not rospy.is_shutdown() and self.tts_busy and rospy.Time.now() < deadline:
                r.sleep()

            # A little extra pad to let room acoustics settle
            rospy.sleep(extra_pad_s)
        else:
            # Fallback timing based on words per minute
            words = max(1, len(text.split()))
            est_secs = min(max_wait_s, (words / (float(fallback_wpm) / 60.0)) + extra_pad_s)
            rospy.sleep(est_secs)

    # ---------- Mic + Whisper ----------

    def record_audio(self, duration=3):
        """Record audio from microphone and save to temp WAV file."""
        rospy.loginfo(f"🎧 Recording for {duration} seconds...")
        audio = sd.rec(int(duration * self.sample_rate), samplerate=self.sample_rate, channels=1, dtype='float32')
        sd.wait()
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name
            write(wav_path, self.sample_rate, (audio * 32767).astype(np.int16))
        return wav_path

    def transcribe_audio(self, wav_path):
        """Transcribe recorded audio using Whisper."""
        # Set fp16=False for CPU; set True if you have a CUDA GPU and want speed.
        result = self.model.transcribe(wav_path, fp16=False,language='en')
        return result["text"].strip().lower()

    # ---------- Dialog flow ----------

    def ask_callback(self, msg):
        if msg.data == "ask" and not self.waiting_for_answer:
            self.waiting_for_answer = True
            self.keep_listening = True
            self.ask_and_listen()
            self.waiting_for_answer = False

        elif msg.data == "cancel_ask":
            rospy.loginfo("🛑 Cancel asking because robot moved.")
            self.keep_listening = False

    def ask_and_listen(self):
        question = "Do you want me to stop following you? Please say yes or no."
        self.say_and_wait(question)  # <-- ensures TTS fully completes before listening
        self.executed = False
        while not rospy.is_shutdown() and self.keep_listening and not self.executed:
            try:
                # small guard pause to avoid capturing tail-end room echo
                rospy.sleep(0.15)

                rospy.loginfo("🎧 Listening for 'yes' or 'no'...")
                wav_path = self.record_audio(duration=3)
                if not self.keep_listening:
                    rospy.loginfo("⏹️ Asking interrupted because robot moved.")
                    break

                text = self.transcribe_audio(wav_path)
                rospy.loginfo(f"🗣️ You said: {text}")

                if "yes" in text:
                    self.say_and_wait("I will stop following you and return to the origin.")
                    self.pub.publish("stop_following")
                    self.run_waypoint_script()
                    rospy.signal_shutdown("User requested stop and return to origin")
                    break

                elif "no" in text:
                    self.say_and_wait("I will continue following you.")
                    self.pub.publish("continue_following")
                    break

                else:
                    self.say_and_wait("Please say yes or no.")
                self.executed = True  # Exit loop after one attempt
            except Exception as e:
                rospy.logerr(f"Speech recognition error: {e}")
                self.say_and_wait("I didn't understand. Please say yes or no.")

    def run_waypoint_script(self):
        rospy.loginfo("📍 Returning to origin")
        subprocess.Popen(['rosrun', 'cml', 'back_origin.py'])


if __name__ == "__main__":
    try:
        node = VoiceTriggerNode()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError):
        pass
