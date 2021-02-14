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
        for i in range(30, 10, -1):
            simulator.send_objectives(i , -i, is_yellow = False)
            sleep(1)

        # Envia os objetivos do time amarelo
        for i in range(10, 30, 1):
            simulator.send_objectives(i, -i, is_yellow = True)
            sleep(1)

if __name__ == '__main__':
    main()
