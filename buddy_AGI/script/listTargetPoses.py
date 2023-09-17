from langchain.output_parsers import StructuredOutputParser, ResponseSchema
from langchain.prompts import PromptTemplate, ChatPromptTemplate, HumanMessagePromptTemplate
from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
import os
openai_api_key = os.getenv("OPENAI_API_KEY")

def listTargetPose():
    response_schemas = [
        ResponseSchema(name="list_pose", description="answer to the user's question." ),
        ResponseSchema(name="wait_time", description="source used to answer the user's question, should be a number."),
        ResponseSchema(name="repeat_time", description="source used to answer the user's question, should be a number.")
    ]
    output_parser = StructuredOutputParser.from_response_schemas(response_schemas)
    format_instructions = output_parser.get_format_instructions()

    prompt = ChatPromptTemplate(
        messages=[
            HumanMessagePromptTemplate.from_template("answer the users question as best as possible.\n{format_instructions}\n{question}")  
        ],
        input_variables=["question"],
        partial_variables={"format_instructions": format_instructions}
    )
    return prompt, output_parser

if __name__ == '__main__':
    prompt, output_parser = listTargetPose()
    #  for test case
    chat_model = ChatOpenAI(temperature=0)
    _input = prompt.format_prompt(question="Buddi, is a warehouse robot, \
        List the target poses with {'list_pose': }: buddi is going to finish Dual Picking Mission, \
            After each Tote Picking Mission, Buddi runs to box_picking_zone to help complete a Box Picking Mission. \
            After completing a mission at box_picking_zone, \
            it returns to tote_picking_zone to complete another Tote Picking Mission, and repeat\
            until user has new mission call.")
    output = chat_model(_input.to_messages())
    print( output_parser.parse(output.content) )
