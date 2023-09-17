from langchain import PromptTemplate, FewShotPromptTemplate
from langchain.llms import OpenAI
from langchain.chains import LLMChain
import os

openai_api_key = os.getenv("OPENAI_API_KEY")

def calculator():
    examples = [
        {"Question": "What is 2+2?", "Answer": "4"},
        {"Question": "What is 3*3?", "Answer": "9"},
        {"Question": "What is 5-3?", "Answer": "2"},
        {"Question": "What is 4/2?", "Answer": "2"},
        {"Question": "Jan has three times the number of pets as Marcia.\
        Marcia has two more pets than Cindy. If Cindy has four pets, \
        how many total pets do the three have?", "Answer": "28"}
    ]

    example_formatter_template = """Question: {Question}
    Answer: {Answer}
    """
    
    example_prompt = PromptTemplate(
        input_variables=["Question", "Answer"],
        template=example_formatter_template,
    )

    few_shot_prompt = FewShotPromptTemplate(
        examples=examples,
        example_prompt=example_prompt,
        prefix="Solve the following math problems step by step:\n",
        suffix="Question: {input}\nAnswer: ",
        input_variables=["input"],
        example_separator="\n",
    )

    return few_shot_prompt

def run_calculator(question):
    llm = OpenAI(temperature=0.9, max_tokens=512, openai_api_key=openai_api_key)
    few_shot_prompt = calculator()
    chain = LLMChain(llm=llm, prompt=few_shot_prompt)
    
    return chain.run(question)

if __name__ == "__main__":
    question = "tote picking has finished for 4 times and ,\
        Box Picking Mission has finished for 3 times \
        以1次Tote Picking Mission为基本衡量单位，\
        一次Box Picking Mission相当于2次Tote Picking Mission；\
        那么Buddi所做的任务总量有多少？"
    answer = run_calculator(question)
    print(answer)

