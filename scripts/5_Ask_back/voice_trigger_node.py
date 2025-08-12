#! /home/lucas/catkin_ws/src/cml/venv/bin/python3

import rospy
import speech_recognition as sr
from std_msgs.msg import String
import subprocess


class VoiceTriggerNode:
    def __init__(self):
        rospy.init_node('voice_trigger_node')
        self.pub = rospy.Publisher('/voice_trigger', String, queue_size=10)
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)
        self.recognizer = sr.Recognizer()

        # Use default microphone
        mic_list = sr.Microphone.list_microphone_names()
        if not mic_list:
            rospy.logerr("❌ No microphone detected.")
            raise RuntimeError("No microphone found.")
        rospy.loginfo(f"🎙️ Detected microphones: {mic_list}")
        #The device_index is different for each laptop
        self.mic = sr.Microphone(device_index=11)

        rospy.Subscriber("/ask_stop_follow", String, self.ask_callback)

        self.waiting_for_answer = False
        self.keep_listening = True

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
        self.tts_pub.publish(question)
        rospy.loginfo(f"🤖 Asking: {question}")
        rospy.sleep(0.2 * len(question.split()))  # wait for TTS

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)
            while not rospy.is_shutdown() and self.keep_listening:
                try:
                    rospy.loginfo("🎧 Listening for 'yes' or 'no'...")
                    audio = self.recognizer.listen(source, timeout=2)
                    if not self.keep_listening:
                        rospy.loginfo("⏹️ Asking interrupted because robot moved.")
                        break

                    text = self.recognizer.recognize_google(audio).lower()
                    rospy.loginfo(f"🗣️ You said: {text}")

                    if "yes" in text:
                        self.tts_pub.publish("I will stop following you and return to the origin.")
                        self.pub.publish("stop_following")
                        self.run_waypoint_script()
                        rospy.signal_shutdown("User requested stop and return to origin")
                        break

                    elif "no" in text:
                        self.tts_pub.publish("I will continue following you.")
                        self.pub.publish("continue_following")
                        break

                    else:
                        self.tts_pub.publish("Please say yes or no.")
                except sr.WaitTimeoutError:
                    if not self.keep_listening:
                        break
                    self.tts_pub.publish("I didn't hear anything. Please say yes or no.")
                except sr.UnknownValueError:
                    self.tts_pub.publish("I didn't understand. Please say yes or no.")
                except sr.RequestError as e:
                    self.tts_pub.publish("There was a problem with speech recognition.")

    def run_waypoint_script(self):
        rospy.loginfo(f"📍 Returning to origin")
        subprocess.Popen(['rosrun','voice_trigger_pkg','waypoint_navi'])

if __name__ == "__main__":
    try:
        node = VoiceTriggerNode()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError):
        pass
