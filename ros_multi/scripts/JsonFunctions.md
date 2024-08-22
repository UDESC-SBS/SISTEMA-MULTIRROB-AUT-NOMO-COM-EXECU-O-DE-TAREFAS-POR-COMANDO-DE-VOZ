JsonFunctions = [
    {
        "name": "Algoritmo de movimentação dos robos",
        "api_name": "_Robot::MoveToGoal",
        "description": "Algoritmo de planejamento de trajeto para movimento de robôs. Esta função calcula o trajeto para um robô específico alcançar um objetivo designado.",
        "parameters": [
            {
                "name": "objetivo",
                "enum": ["cozinha", "garagem", "quarto A", "quarto B", "seção A", "seção B", "sala de estar", "banheiro"],
                "description": "Local para onde o usuario escolheu movimentar onde o ,robô precisa ir, as opções disponíveis incluem: cozinha, garagem, quarto A, quarto B, seção A, seção B, sala de estar e banheiro."
            },
            {
                "name": "index",
                "description": "Um número que representa a identificação do robo, este número é usado para especificar qual robô precisa realizar o planejamento de trajeto, o identificador começa em 0, se o usuario escolheu 'o primeiro robô' este numero deve ser 0, e assim por diante."
            }
         ]
    },
    {
        "name":"Algoritmo de escaneamento"
        "api_name":"_Map::scanMap"
        "description":"Algoritmo responsável pela criação de mapa e escaneamento do cenário"
    }
]