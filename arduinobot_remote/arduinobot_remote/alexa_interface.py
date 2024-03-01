#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from arduinobot_msgs.action import ArduinobotTask
import rclpy
from rclpy.node import Node
import threading
from rclpy.action import ActionClient

threading.Thread(target=lambda: rclpy.init()).start()
action_client = ActionClient(Node('alexa_interface'), ArduinobotTask, "task_server")

app = Flask(__name__)

# THIS CLASS IS TO TAKE THE INPUT FROM VOICE COMMAND AND TURN INTO ACTION GOAL
class LaunchRequestHandler(AbstractRequestHandler):
    # THE HANDLE IS CATEGORIZED INTO THE LAUNCH REQUEST INTENT
    def can_handle(self, handler_input):
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # IF THE INTENT IS FOR ACTIVATION RETURN THE READY TO WORK MESSAGE
        speech_text = "Hi, I'm ready to work"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Online", speech_text)).set_should_end_session(
            False)
        # ASSIGN THE GOAL AS THE 0 NUMBER TO INDICATE TO TASK SERVER THE CORRECT GOAL
        goal = ArduinobotTask.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response

#THIS HANDLER IS USED TO GIVE THE PICK INSTRUCTION TO THE ROBOT
class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)
#THIS IS THE REPLY TEXT THAT THE ROBOT PROVIDES THE USER
    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm moving"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(
            True)
        goal = ArduinobotTask.Goal()
        goal.task_number = 1
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response
#THIS IS THE HANDLER TO SEND THE ROBOT TO SLEEP WHEN NECESSARY

class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        #THIS IS THE SPEECH RESPONSE THE ROBOT RETURNS
        speech_text = "Ok, see you later"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 2
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response

#THIS IS THE HANDLER TO WAKE THE ROBOT FROM SLEEP
class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, I am ready"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(
            True)
            
        goal = ArduinobotTask.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response

#IN CASE THE REQUEST ISNT RECOGNIZED THIS IS THE EXCEPTION HANDLER
class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        #THE HANDLER REPLY FOR EXCEPTIONS

        speech = "Hmm, I don't know that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

#ALL THE HANDLES ARE CODED INTO THE ALEXA SKILL
skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

#THIS IS WHERE YOU INPUT YOUR ALEXA TOKEN FOR API USAGE

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), 
    skill_id="amzn1.ask.skill.cfde5567-8532-456e-8fe3-cf22c37d1e2d",
    app=app)


skill_adapter.register(app=app, route="/")


if __name__ == '__main__':
    app.run()