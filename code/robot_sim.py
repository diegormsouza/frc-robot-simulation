#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file        robot_sim.py
@brief       Módulo de simulação de pose para robô FRC com visualização no Field2d (Glass).
@details     Esta classe define o comportamento simulado de um robô FRC em termos de posição, orientação,
             velocidade e rotação. O módulo permite a visualização do robô em um campo virtual 2D via Glass
             utilizando o SmartDashboard. É usado como base para testes de algoritmos no modo autônomo
             e teleoperado sem hardware físico.
@version     2.1
@date        2025-07-25
@license     MIT
@authors     Diego Souza
"""

import wpilib  # Importa a biblioteca WPILib para funcionalidades de robótica, como TimedRobot e NetworkTables
from wpilib import SmartDashboard  # Importa o módulo SmartDashboard para enviar dados ao painel (ex.: Glass)
from wpilib import Field2d  # Importa o módulo Field2d para criar um campo virtual 2D no Glass
from wpimath.geometry import Pose2d, Rotation2d  # Importa classes para representar posição (x, y) e rotação do robô
import math  # Importa a biblioteca math para cálculos matemáticos, como seno, cosseno e conversão de graus para radianos

class RobotSim(wpilib.TimedRobot):  # Define a classe RobotSim, que herda de TimedRobot para simulação periódica
    def __init__(self):  # Método construtor, chamado quando a classe é instanciada
        super().__init__()  # Chama o construtor da classe pai (TimedRobot) para inicializar o framework WPILib
        self.drivetrain = None  # Inicializa a variável drivetrain como None (será definida pelo robot.py)
        self.timer = None  # Inicializa a variável timer como None (será usada para contar o tempo)
        self.field = None  # Inicializa a variável field como None (será usada para o campo virtual no Glass)

        # Estado do robô na simulação
        self.x = 0.0  # Define a posição inicial do robô no eixo X (em metros, começa em 0)
        self.y = 0.0  # Define a posição inicial do robô no eixo Y (em metros, começa em 0)
        self.heading = 0.0  # Define a orientação inicial do robô (em graus, começa em 0)

        # Velocidade atual aplicada, que você deve atualizar em robot.py
        self.current_speed = 0.0  # Define a velocidade inicial do robô na simulação (0 = parado)
        self.current_rotation = 0.0  # Define a rotação inicial do robô na simulação (0 = sem girar)

        # Escalas para transformar comandos em deslocamento real
        self.movement_scale = 0.08  # Define a escala para converter velocidade em deslocamento (metros por ciclo)
        self.rotation_scale = 2.0  # Define a escala para converter rotação em mudança de ângulo (graus por ciclo)

        # Inicializa o estado do comando genérico
        SmartDashboard.putString("Command State", "Desativado")  # Envia o estado inicial para o SmartDashboard

    def robotInit(self):  # Método chamado uma vez quando o robô é inicializado
        # Inicializa o campo virtual no Glass
        self.field = Field2d()  # Cria um objeto Field2d para exibir o robô em um campo 2D no Glass
        SmartDashboard.putData("Field", self.field)  # Envia o campo Field2d para o SmartDashboard para visualização
        self.set_pose(0.0, 0.0, 0.0)  # Define a posição inicial do robô (x=0, y=0, orientação=0 graus)

        # Inicializa o temporizador
        self.timer = wpilib.Timer()
        print("Simulação simples inicializada.")  # Exibe uma mensagem no console para confirmar que a simulação começou

    def set_drivetrain(self, drivetrain):  # Método para conectar o sistema de condução do robot.py à simulação
        self.drivetrain = drivetrain  # Armazena o objeto drivetrain (DifferentialDrive) passado pelo robot.py

    def set_pose(self, x, y, heading):  # Método para definir a posição e orientação do robô na simulação
        self.x = x  # Atualiza a coordenada X do robô (em metros)
        self.y = y  # Atualiza a coordenada Y do robô (em metros)
        self.heading = heading % 360  # Atualiza a orientação (em graus, mantida entre 0 e 360)
        if self.field:  # Verifica se o campo Field2d foi inicializado
            pose = Pose2d(self.x, self.y, Rotation2d.fromDegrees(self.heading))  # Cria um objeto Pose2d com posição (x, y) e rotação
            self.field.setRobotPose(pose)  # Atualiza a posição do robô no campo virtual no Glass

    def update_pose(self):  # Método para atualizar a posição do robô com base na velocidade e rotação
        theta_rad = math.radians(self.heading)  # Converte a orientação do robô de graus para radianos
        dx = self.current_speed * self.movement_scale * math.cos(theta_rad)  # Calcula o deslocamento no eixo X usando cosseno
        dy = self.current_speed * self.movement_scale * math.sin(theta_rad)  # Calcula o deslocamento no eixo Y usando seno
        self.x += dx  # Atualiza a coordenada X somando o deslocamento calculado
        self.y += dy  # Atualiza a coordenada Y somando o deslocamento calculado
        self.heading += self.current_rotation * self.rotation_scale  # Atualiza a orientação somando a rotação escalada
        self.heading %= 360  # Mantém a orientação entre 0 e 360 graus

        # Atualiza pose no Field2d
        if self.field:  # Verifica se o campo Field2d foi inicializado
            pose = Pose2d(self.x, self.y, Rotation2d.fromDegrees(self.heading))  # Cria um novo objeto Pose2d com a posição e rotação atualizadas
            self.field.setRobotPose(pose)  # Atualiza a posição do robô no campo virtual no Glass

    def update_command_state(self, active):  # Atualiza o estado do comando genérico
        state = "Ativado" if active else "Desativado"  # Converte o estado booleano para string
        SmartDashboard.putString("Command State", state)  # Envia o estado para o SmartDashboard

    def autonomousInit(self):  # Método chamado uma vez no início do modo autônomo
        self.set_pose(0.0, 0.0, 0.0)  # Reseta a posição do robô para (x=0, y=0, orientação=0)
        self.current_speed = 0.0  # Define a velocidade inicial como 0 (robô parado)
        self.current_rotation = 0.0  # Define a rotação inicial como 0 (sem girar)
        self.timer.reset()  # Zera o temporizador para começar a contar do zero
        self.timer.start()  # Inicia a contagem do tempo no temporizador
        self.update_command_state(False)  # Reseta o estado do comando

    def autonomousPeriodic(self):  # Método chamado repetidamente (a cada 20ms) no modo autônomo
        # NÃO controla o drivetrain aqui, só atualiza pose e dashboard
        self.update_pose()  # Atualiza a posição e orientação do robô com base em current_speed e current_rotation
        SmartDashboard.putNumber("X", self.x)  # Envia a coordenada X para o SmartDashboard (exibida no Glass)
        SmartDashboard.putNumber("Y", self.y)  # Envia a coordenada Y para o SmartDashboard (exibida no Glass)
        SmartDashboard.putNumber("Heading", self.heading)  # Envia a orientação (em graus) para o SmartDashboard
        SmartDashboard.putNumber("Time", self.timer.get())  # Envia o tempo atual do temporizador para o SmartDashboard

    def teleopPeriodic(self):  # Método chamado repetidamente (a cada 20ms) no modo teleoperado
        # Também não controla o drivetrain aqui
        self.update_pose()  # Atualiza a posição e orientação do robô com base em current_speed e current_rotation
        SmartDashboard.putNumber("X", self.x)  # Envia a coordenada X para o SmartDashboard (exibida no Glass)
        SmartDashboard.putNumber("Y", self.y)  # Envia a coordenada Y para o SmartDashboard (exibida no Glass)
        SmartDashboard.putNumber("Heading", self.heading)  # Envia a orientação (em graus) para o SmartDashboard
        SmartDashboard.putNumber("Time", self.timer.get())  # Envia o tempo atual do temporizador para o SmartDashboard