#!/usr/bin/env python3 
import sys
import numpy as np
import soundfile as sf
import soundcard as sc
import whisper
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QPushButton
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from defFewShotForMission import defFewShotForMission
from classifyQuetions import classifyQuetions
from langchain.memory import ConversationBufferMemory,ConversationSummaryBufferMemory
from langchain import OpenAI, LLMChain, PromptTemplate
from langchain.prompts import PromptTemplate
from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
from langchain.chains import LLMChain
from calculatorBuddi import calculator
from summarizeMission import summarizePrompt
from listTargetPoses import listTargetPose
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QPushButton, QLineEdit
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from datetime import datetime
from buddy_navigation import RobotNavigation
import json
import os
openai_api_key = os.getenv("OPENAI_API_KEY")
llm=ChatOpenAI(openai_api_key=openai_api_key, model_name='gpt-4',temperature=0 )
memory = ConversationSummaryBufferMemory(llm=llm, max_token_limit=1000)
robot_navigation = RobotNavigation()
            
class RecordThread(QThread):
    def __init__(self, filename, stop_signal):
        super().__init__()
        self.filename = filename
        self.stop_signal = stop_signal

    def run(self):
        record_audio(self.filename, self.stop_signal)

def record_audio(filename, stop_signal):
    microphone = sc.default_microphone()
    
    with microphone.recorder(samplerate=48000) as mic, sf.SoundFile(filename, mode='w', samplerate=48000, channels=1, subtype='PCM_24') as file:
        while not stop_signal.is_set():
            data = mic.record(numframes=1024)
            data_mono = np.mean(data, axis=1)
            file.write(data_mono)

class TranscribeThread(QThread):
    result_signal = pyqtSignal(str, str)

    def run(self):
        try:
            model = whisper.load_model("base")
            audio = whisper.load_audio("output.wav")
            audio = whisper.pad_or_trim(audio)
            mel = whisper.log_mel_spectrogram(audio).to(model.device)
            options = whisper.DecodingOptions()
            result = whisper.decode(model, mel, options)

            few_shot_prompt = defFewShotForMission()
            llm = OpenAI(temperature=0)
            chain = LLMChain(llm=llm, prompt=few_shot_prompt)
            openai_result = chain.run(result.text)

            self.result_signal.emit(result.text, openai_result)
        except Exception as e:
            self.result_signal.emit(f"Error during transcription: {e}", "")

class ProcessThread(QThread):
    result_signal = pyqtSignal(str)

    def __init__(self, text):
        super().__init__()
        self.text = text

    def run(self):
        response = self.process_transcription(self.text)
        self.result_signal.emit(response)

    def prompt_langchain(self, prompt_type):
        llm_chain = LLMChain(
            llm=llm, 
            prompt=prompt_type, 
            verbose=True, 
            memory=memory,
            )
        return llm_chain

    def few_shot_prompt(self):
        return defFewShotForMission()
    
    def classify_questions(self):
        return classifyQuetions()

    def calculate_tasks(self):
        return calculator()
    
    def summarize_tasks(self):
        return summarizePrompt()
    
    def save_poses_to_json(self, data, filename):
        with open(filename, 'w') as json_file:
            json.dump(data, json_file)
    
    def process_transcription(self, text):
        # Add the original contents of this method from MainWindow class
        llm_chain = self.prompt_langchain(self.classify_questions())
        response = llm_chain.run(text)
        
        if response == "Action":
            llm_chain = self.prompt_langchain(self.few_shot_prompt())
            response = llm_chain.run(text) 
            # from response to detailed Jason File, pose_name, wait_time, repeat_time
            promt_pose,output_parser = listTargetPose()
            llm_chain = self.prompt_langchain(promt_pose)
            prompt, output_parser = listTargetPose()
            #  for test case 
            _input = prompt.format_prompt(question="Buddi is a warehouse robot, \
                     List the target poses with {'list_pose': }: " + response)
            output = llm(_input.to_messages())
            print( output_parser.parse(output.content) )
            list_pose_filename = "/home/susan/BuddyDemoAGI/src/buddy_AGI/json/list_pose.json"
            self.save_poses_to_json(output_parser.parse(output.content), list_pose_filename)
            robot_navigation.continuous_navigation(list_pose_filename)

        elif response == "Calculate":
            llm_chain = self.prompt_langchain(self.calculate_tasks())
            response = llm_chain.run(text)
            
        else: 
            llm_chain = self.prompt_langchain(self.summarize_tasks())
            response = llm_chain.run(text)
        return response
                    
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        
        self.chat_panel = QTextEdit(self)
        self.input_line = QLineEdit(self)
        self.record_button = QPushButton("Record", self) 
        self.exit_button = QPushButton("Exit", self)
        self.enter_button = QPushButton("Enter", self)

        self.layout = QVBoxLayout()
        self.bottom_layout = QHBoxLayout()

        self.bottom_layout.addWidget(self.record_button)
        self.bottom_layout.addWidget(self.enter_button)
        self.bottom_layout.addWidget(self.exit_button)

        self.layout.addWidget(self.chat_panel)
        self.layout.addWidget(self.input_line)
        self.layout.addLayout(self.bottom_layout)

        self.setLayout(self.layout)

        self.record_button.pressed.connect(self.start_recording)
        self.record_button.released.connect(self.stop_recording)
        self.enter_button.clicked.connect(self.on_input_enter)
        self.exit_button.clicked.connect(self.close_application)

        self.chat_panel.setLayoutDirection(Qt.RightToLeft)
        self.stop_signal = threading.Event()
        
    def on_input_enter(self): 
        text = self.input_line.text()
        if text:
            current_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.chat_panel.append(f"[{current_timestamp}] Me: {text}")

            self.process_thread = ProcessThread(text)
            self.process_thread.result_signal.connect(self.on_process_finished)
            self.process_thread.start()

            self.input_line.clear()
        else:
            self.transcribe_audio()
            
    def on_process_finished(self, response):
        current_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.chat_panel.append(f"[{current_timestamp}] Buddi: {response}")

    def send_message(self):
        user_text = self.user_input.text()
        if user_text:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.chat_panel.append(f"{timestamp} You: {user_text}")

            # Call process_transcription here
            response = self.process_transcription(user_text)
            self.chat_panel.append(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} Buddi: {response}")

            # Clear the user input field
            self.user_input.clear()
        
    def close_application(self):
        self.stop_signal.set()
        if hasattr(self, 'record_thread'):
            self.record_thread.wait()
        QApplication.quit()

    def start_recording(self):
        try:
            self.chat_panel.append("Start recording...")
            self.stop_signal.clear()
            self.record_thread = RecordThread("output.wav", self.stop_signal)
            self.record_thread.start()
        except Exception as e:
            self.chat_panel.append(f"Error during recording: {e}")
            
    def stop_recording(self):
        try:
            self.stop_signal.set()
            self.record_thread.wait()
            self.chat_panel.append("Finished recording. Saved as output.wav.")
        except Exception as e:
            self.chat_panel.append(f"Error stopping recording: {e}")
            
    # Modify the transcribe_audio function
    def transcribe_audio(self):
        self.chat_panel.append("Transcribing audio...")
        self.transcribe_thread = TranscribeThread()
        self.transcribe_thread.result_signal.connect(self.on_transcribe_finished)
        self.transcribe_thread.start()

    def on_transcribe_finished(self, transcription_result, openai_result):
        if transcription_result.startswith("Error during transcription"):
            self.chat_panel.append(transcription_result)
        else:
            self.chat_panel.append(f"Transcribed text: {transcription_result}")
            self.chat_panel.append("Transcription completed.")
            self.chat_panel.append(f"Buddy Receive: {openai_result}")
                        
if __name__=='__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())