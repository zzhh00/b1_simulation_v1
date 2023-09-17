from langchain.llms import OpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import ConversationChain
from langchain.memory import ConversationBufferMemory, CombinedMemory, ConversationSummaryMemory
import os
openai_api_key = os.getenv("OPENAI_API_KEY")


def summarizePrompt(history='', input_text=''):

    _DEFAULT_TEMPLATE = """you are a warehouse robot, can summay the tasks, record actions . \
        The robot is talkative and provides lots of specific details from its context. \
        If the robot does not know the answer to a question, it truthfully says it does not know.
    Summary of conversation:
    {history} 
    Human: {input}
    """

    PROMPT = PromptTemplate(
        input_variables=["history", "input"], template=_DEFAULT_TEMPLATE
    )
 
    return PROMPT

