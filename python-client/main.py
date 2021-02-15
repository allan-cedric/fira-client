#!/usr/bin/python3

from fira_client import *
import socket
from time import sleep

def main():

    # Instância do simulador
    simulator = FiraClient()

    while True:
        # Metadados da simulação (info. sobre os robôs e o campo)
        pkg = simulator.receive()
        print(pkg)

        # Envia os dados da simulação para 1 robô
        # simulator.sendCommandsPacket(10, 10)

        # Envia os objetivos do time azul
        simulator.send_objectives(10 , -10, is_yellow = False)

        # Envia os objetivos do time amarelo
        simulator.send_objectives(10, -10, is_yellow = True)

if __name__ == '__main__':
    main()
