#!/usr/bin/env python3

import json
import rospy
from std_msgs.msg import String
#from faster_whisper import WhisperModel
from llama_cpp import Llama
from threading import Thread 
import time
def get_prompt(user_query: str, functions: list = []) -> str:
    system = """You are an AI programming assistant that always answer short and only about the function call never in a json format, if the user asked more than one function call, the first character will be a number of functions the user asked for, here is a template of answer: 
    3
    RobotHandler::MoveToGoal(objetivo="garagem", index=0)
    RobotHandler::MoveToGoal(objetivo="cozinha", index=1)
    RobotHandler::MoveToGoal(objetivo="banheiro", index=2)"""
    if len(functions) == 0:
        return f"{system}\n### Instruction: <<question>> {user_query}\n### Response: "
    functions_string = json.dumps(functions)
    #return f"system: {system} user query: {user_query} functions: {functions_string}"
    return f"{system}\n### Instruction: <<function>>{functions_string}\n<<question>>{user_query}\n### Response: "


JsonFunctions = [
    {
        "name": "Algoritmo de movimentação dos robos",
        "api_name": "RobotHandler::MoveToGoal",
        "description": "Algoritmo de planejamento de trajeto para movimento de robôs. Esta função calcula o trajeto para um robô específico alcançar um objetivo designado.",
        "parameters": [
            {
                "name": "objetivo",
                "enum": ["cozinha", "garagem", "quarto", "banheiro", "sala de estar", "sala de jogos"],
                "description": "Local para onde o usuario escolheu movimentar onde o ,robô precisa ir, as opções disponíveis incluem: cozinha, garagem, quarto, sala de estar, sala de jogos e banheiro."
            },
            {
                "name": "index",
                "description": "Um número que representa a identificação do robo, este número é usado para especificar qual robô precisa realizar o planejamento de trajeto, o identificador começa em 0, se o usuario escolheu 'o primeiro' este numero deve ser 0, se escolheu 'o segundo' este deve numero deve ser 1 e assim por diante."
            }
         ]
    },
    {
        "name": "Algoritmo de escaneamento",
        "api_name": "MapHandler::CreateMap",
        "description": "Algoritmo responsável pela criação de mapa e escaneamento do cenário, exemplos de utilização para este comando seria: 'Escaneie este local', 'Escaneie esta área', 'Escaneie este cômodo', 'Escaneie', 'Crie um mapa'. Se o usuario desejar executar esta função, retornar apenas 'Map::CreateMap()', não á opções de entrada para esta função",
    }
]

model = "/home/robotica/Documentos/sic_pedro/src/ros_multi/scripts/LocalAI-Llama3-8b-Function-Call-v0.2-Q4_K_M.gguf"
llm = Llama(model_path=model, verbose=False, n_ctx=2048)


StringList = []

def talker():
    pub = rospy.Publisher('LLama', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    print("Hello World!")
    while not rospy.is_shutdown():
        if StringList:
            #rospy.loginfo(hello_str)
            pub.publish(StringList[0])
            del StringList[0]
        time.sleep(0.1)
        

def LlamaCallback(data):
    rospy.loginfo("%s", data.data)
    model_output = llm(
       get_prompt(data.data, JsonFunctions),
       max_tokens=128,
       #temperature=0.3,
       #top_p=0.1,
       #echo=False,
       #stop=["Q", "\n"],
    )
    final_result = model_output["choices"][0]["text"].strip()
    StringList.append(final_result)
    print(final_result)

def listener():
    rospy.Subscriber('RobotHandler', String, LlamaCallback)
    rospy.Subscriber('Whisper', String, LlamaCallback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('AIWrapper', anonymous=True)
    t = Thread(target=talker).start()
    listener()
    pass