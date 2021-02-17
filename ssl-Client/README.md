Informacoes Gerais

field_length=1.500000 (mm)
field_width=1.300000

Path Planning:
Iremos implementar o path planning dos robôs da seguinte maneira:

Usando técnicas parecidas com as descritas em https://redblobgames.github.io/circular-obstacle-pathfinding/,
iremos descobrir pontos críticos do plano que servirão como nodos para a execução de um algoritmo de planejamento de caminho A*.

Uma vez descobrindo o caminho de pontos a ser traçado, podemos usar suas coordenadas como argumentos na chamada de ```PID()```.
