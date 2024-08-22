#! /usr/bin/python3

from llama_cpp import Llama
import transformers
import json


def get_prompt(user_query: str, functions: list = []) -> str:
    system = "You are an AI programming assistant, the format of your answer will be a function call, you won't write any comments."
    if len(functions) == 0:
        return f"{system}\n### Instruction: <<question>> {user_query}\n### Response: "
    functions_string = json.dumps(functions)
    return f"{system}\n### Instruction: <<function>>{functions_string}\n<<question>>{user_query}\n### Response: "


JsonFunctions = [
    {
        "name": "Algoritmo de movimentação dos robos",
        "api_name": "Robot::MoveToGoal",
        "description": "Algoritmo de planejamento de trajeto para movimento de robôs. Esta função calcula o trajeto para um robô específico alcançar um objetivo designado.",
        "parameters": [
            {
                "name": "objetivo",
                "enum": ["cozinha", "garagem", "quarto A", "quarto B", "seção A", "seção B", "sala de estar", "banheiro"],
                "description": "Local para onde o usuario escolheu movimentar onde o ,robô precisa ir, as opções disponíveis incluem: cozinha, garagem, quarto A, quarto B, seção A, seção B, sala de estar e banheiro."
            },
            {
                "name": "index",
                "description": "Um número que representa a identificação do robo, este número é usado para especificar qual robô precisa realizar o planejamento de trajeto, o identificador começa em 0, se o usuario escolheu 'o primeiro' este numero deve ser 0, se escolheu 'o segundo' este deve numero deve ser 1 e assim por diante."
            }
         ]
    },
    {
        "name": "Algoritmo de escaneamento",
        "api_name": "Map::CreateMap",
        "description": "Algoritmo responsável pela criação de mapa e escaneamento do cenário, exemplos de utilização para este comando seria: 'Escaneie este local', 'Escaneie esta área', 'Escaneie este cômodo', 'Escaneie', 'Crie um mapa'. Se o usuario desejar executar esta função, retornar apenas 'Map::CreateMap()', não á opções de entrada para esta função",
    }
]

model = "LocalAI-Llama3-8b-Function-Call-v0.2-Q4_K_M.gguf"

llm = Llama(model_path=model, verbose=False, n_ctx=2048)


prompt = "Mova o quarto robo para a cosinha."
model_output = llm(
       get_prompt(prompt, JsonFunctions),
       max_tokens=64,
       #temperature=0.3,
       #top_p=0.1,
       echo=False,
       stop=["Q", "\n"],
   )
final_result = model_output["choices"][0]["text"].strip()
print(final_result)