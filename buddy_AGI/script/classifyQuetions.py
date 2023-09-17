from langchain import PromptTemplate, FewShotPromptTemplate
from langchain.llms import OpenAI
import os
openai_api_key = os.getenv("OPENAI_API_KEY")

def classifyQuetions():
    # First, create the list of few shot examples.
    examples = [
        {"Question": "how many times mission you have done", "Class": " Summarize"},
        
        {"Question": "summarize the total of missions you have done ", "Class": "Summarize"},
        
        {"Question": "how many kinds of missions have been arranged ", "Class": "Summarize"},
        
        {"Question": "以1次Tote Picking Mission为基本衡量单位, \
            一次Box Picking Mission相当于2次Tote Picking Mission, \
                那么Buddi所做的任务总量有多少", "Class": "Calculate"},
        
        {"Question": " do some Mission", "Class": " Action"},
        
        {"Question": " generate a dual mission and go ", "Class": " Action"}
    ]
    # Next, we specify the template to format the examples we have provided.
    # We use the `PromptTemplate` class for this.
    example_formatter_template = """Question: {Question}
    Class: {Class}
    """
    example_prompt = PromptTemplate(
        input_variables=["Question", "Class"],
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
        prefix="Give the Class of every input\n",
        # The suffix is some text that goes after the examples in the prompt.
        # Usually, this is where the user input will go
        suffix="Question: {input}\nClass: ",
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
    few_shot_prompt = classifyQuetions()
    chain = LLMChain(llm=llm, prompt=few_shot_prompt) 
    print(chain.run("hi buddi, I need you pick one box from box picking zone to dropping zone for three times"))
