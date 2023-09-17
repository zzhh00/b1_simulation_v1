from langchain import PromptTemplate, FewShotPromptTemplate
from langchain.llms import OpenAI
import os
openai_api_key = os.getenv("OPENAI_API_KEY")

def defFewShotForMission():
    # First, create the list of few shot examples.
    examples = [
        {"Mission": "Tote Picking Mission", "Details": "Buddi is running \
            between a picking spot at tote_picking_zone and \
            a drop-off spot at dropping_zone for one loop, pausing for three seconds \
            each time it reaches a location. These three seconds are used\
            to express the time it takes for a person to lift or lower an item."},
        
        {"Mission": "Box Picking Mission", "Details": "Buddi is running \
            between a picking spot at box_picking_zone and a drop-off spot at \
            dropping_zone for one loop, pausing for three seconds each time it reaches\
            a location.These three seconds are used to express the time it\
            takes for a person to lift or lower an item."},
        
        {"Mission": "Dual Picking Mission", "Details": " After each Tote Picking \
            Mission, Buddi runs to box_picking_zone to help complete a Box Picking Mission. \
            After completing a mission at box_picking_zone, \
            it returns to tote_picking_zone to complete another Tote Picking Mission, and repeat\
            1 time.These three seconds are used\
            to express the time it takes for a person to lift or lower an item."
        }
    ]
    # Next, we specify the template to format the examples we have provided.
    # We use the `PromptTemplate` class for this.
    example_formatter_template = """Mission: {Mission}
    Details: {Details}
    """
    example_prompt = PromptTemplate(
        input_variables=["Mission", "Details"],
        template=example_formatter_template,
    )

    # Finally, we create the `FewShotPromptTemplate` object.
    few_shot_prompt = FewShotPromptTemplate(
        # These are the examples we want to insert into the prompt.
        examples=examples,
        # This is how we want to format the examples when we insert them into the prompt.
        example_prompt=example_prompt,
        # The prefix is some text that goes before the examples in the prompt.
        # Usually, this consists of intructions.
        prefix="Give the Details of every input\n",
        # The suffix is some text that goes after the examples in the prompt.
        # Usually, this is where the user input will go
        suffix="Mission: {input}\nDetails: ",
        # The input variables are the variables that the overall prompt expects.
        input_variables=["input"],
        # The example_separator is the string we will use to join the prefix, examples, and suffix together with.
        example_separator="\n",
    )
    return few_shot_prompt

if __name__ == "__main__":
    # We can now generate a prompt using the `format` method.
    # print(few_shot_prompt.format(input="box picking")) 
    from langchain.prompts import PromptTemplate
    from langchain.llms import OpenAI
    from langchain.chains import LLMChain
    
    llm = OpenAI(temperature=0.9)
    few_shot_prompt = defFewShotForMission()
    chain = LLMChain(llm=llm, prompt=few_shot_prompt) 
    print(chain.run("dual picking : 1. storage to tote picking; 2. box picking to dropping zone"))
