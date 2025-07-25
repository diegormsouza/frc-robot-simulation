#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file        main.py
@brief       Simulação de robô FRC com modos autônomo e teleoperado usando WPILib.
@details     Para faciliar a familiarização dos estudantes, este script implementa uma simulação de um 
             robô com drivetrain diferencial, controlado por joystick no modo teleoperado e com movimento 
             programado no modo autônomo. Utiliza motores VictorSPX e integração com NetworkTables e SmartDashboard.
             TODO: Verificar como simular satisfatóriamente comandos e feedback de sensores.
@version     1.8
@date        2025-07-24
@license     MIT
@authors     Diego Souza
"""

# Importações principais da biblioteca WPILib e suporte à simulação
import wpilib                     # Funções principais do WPILib (Timer, Joystick, etc.)
import wpilib.drive               # Sistemas de direção como DifferentialDrive
import phoenix5                   # Controle de motores VictorSPX (CTRE)

from wpilib import SmartDashboard # Envio de dados ao SmartDashboard
import math                       # Importa a biblioteca math para cálculos, como seno (usado no ziguezague)
from robot_sim import RobotSim    # Classe base com suporte à simulação (Field2d, Glass)

class MyRobot(RobotSim):
    def robotInit(self):
        """
        Esta função é executada uma vez quando o robô é ligado.
        Aqui configuramos os motores, o controle, o temporizador e o menu de modos autônomos.
        """  
        # Configura os motores do robô (esquerda e direita)
        self.left_back = phoenix5.WPI_VictorSPX(1)   # Motor traseiro esquerdo
        self.right_back = phoenix5.WPI_VictorSPX(2)  # Motor traseiro direito
        self.left_front = phoenix5.WPI_VictorSPX(3)  # Motor frontal esquerdo
        self.right_front = phoenix5.WPI_VictorSPX(4) # Motor frontal direito

        # Agrupa os motores para controlar juntos
        self.left = wpilib.MotorControllerGroup(self.left_back, self.left_front)
        self.right = wpilib.MotorControllerGroup(self.right_back, self.right_front)

        # Configura o sistema de movimentação do robô
        self.drivetrain = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.right.setInverted(True)  # Inverte o lado direito para girar corretamente

        # Configura o joystick (controle) no porto 0
        self.joystick = wpilib.Joystick(0)

        # Configura o temporizador para contar o tempo
        self.timer = wpilib.Timer()

        # Conecta o drivetrain à simulação
        self.set_drivetrain(self.drivetrain)

        # Cria um menu para escolher o modo autônomo
        self.selector = wpilib.SendableChooser()
        self.selector.setDefaultOption("Parado (não se move)", 0)  # Modo padrão: parado
        self.selector.addOption("Avançar 1 segundo", 1)
        self.selector.addOption("Avançar 2 segundos", 2)
        self.selector.addOption("Avançar 3 segundos", 3)
        self.selector.addOption("Modo Quadrado", 4)
        self.selector.addOption("Modo Ziguezague", 5)
        self.selector.addOption("Modo Círculo", 6)
        SmartDashboard.putData("Auto Selector", self.selector)  # Mostra o menu no painel

        # Para simular um comando genérico
        SmartDashboard.putString("Command State", "Desativado")

        # Inicia a simulação
        super().robotInit()

    def autonomousInit(self):
        """
        Esta função é chamada uma vez quando o modo autônomo começa.
        Ela escolhe o modo selecionado no painel e inicia o temporizador.
        """
        # Pega o modo escolhido no painel
        self.autonomous_mode = self.selector.getSelected()
        # Reseta e inicia o temporizador
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """
        Esta função roda repetidamente durante o modo autônomo.
        É chamada automaticamente pelo WPILib a cada 20 ms (50 vezes por segundo).
        Aqui programamos o que o robô faz sozinho, sem controle humano.
        """        
        # Define a velocidade e rotação iniciais
        speed = 0.0
        turn = 0.0

        # Modos autônomos - a serem trabalhados pelos alunos

        # Modo autônomo 1 - Exemplo: ir adiante 1 s e parar
        if self.autonomous_mode == 1:
            if self.timer.get() < 1:
                speed = 0.5
                turn = 0.0                

        # Modo autônomo 2 - Exemplo: ir adiante 2 s e parar
        elif self.autonomous_mode == 2:
            if self.timer.get() < 2:
                speed = 0.5
                turn = 0.0                

        # Modo autônomo 3 - Exemplo: ir adiante 3 s e parar
        elif self.autonomous_mode == 3:
            if self.timer.get() < 3:
                speed = 0.5
                turn = 0.0    

        # Modo autônomo 4 - Exemplo: fazer trajetória em quadrado
        elif self.autonomous_mode == 4:  # Modo quadrado
            time = self.timer.get()  # Obtém o tempo atual do temporizador
            if time < 1.0:  # 1º lado: avança por 1s
                speed = 0.5
                turn = 0.0
            elif time < 1.5:  # 1º giro: gira 90 graus por 0.5s
                speed = 0.0
                turn = 1.8  # Rotação ajustada para 90 graus
            elif time < 2.5:  # 2º lado: avança por 1s
                speed = 0.5
                turn = 0.0
            elif time < 3.0:  # 2º giro: gira 90 graus por 0.5s
                speed = 0.0
                turn = 1.8
            elif time < 4.0:  # 3º lado: avança por 1s
                speed = 0.5
                turn = 0.0
            elif time < 4.5:  # 3º giro: gira 90 graus por 0.5s
                speed = 0.0
                turn = 1.8
            elif time < 5.5:  # 4º lado: avança por 1s
                speed = 0.5
                turn = 0.0
            elif time < 6.0:  # 4º giro: gira 90 graus por 0.5s
                speed = 0.0
                turn = 1.8
            # Após 6s, o robô para (speed e turn já são 0.0)          
        
        # Modo autônomo 5 - Exemplo: fazer trajetória em ziguezague
        elif self.autonomous_mode == 5:  # Modo ziguezague
            if self.timer.get() < 8:
                speed = 0.6  # Avança com velocidade constante
                turn = 0.8 * math.sin(self.timer.get() * 2.0)  # Gira em ziguezague usando seno
        
        # Modo autônomo 6 - Exemplo: fazer trajetória em círculo
        elif self.autonomous_mode == 6:  # Modo círculo
            if self.timer.get() < 6:
                speed = 0.6  # Avança com velocidade constante
                turn = 0.6   # Gira suavemente para formar um círculo

        # Aplica a velocidade e rotação ao robô    
        self.drivetrain.arcadeDrive(speed, turn)

        # Atualiza a simulação
        self.current_speed = speed
        self.current_rotation = turn
        super().autonomousPeriodic()

    def teleopInit(self):
        """
        Esta função é chamada uma vez quando começa o modo operado (com joystick).
        Aqui zeramos o timer e garantimos que o robô comece parado.
        """
        # Reseta o temporizador e para o robô
        self.timer.reset()
        self.timer.start()
        self.drivetrain.arcadeDrive(0, 0)

    def teleopPeriodic(self):
        """
        Esta função roda várias vezes por segundo no modo controlado.
        É chamada automaticamente pelo WPILib a cada 20 ms (50 vezes por segundo).
        Aqui lemos o joystick e movimentamos o robô conforme o usuário.
        """
        # Lê os valores do joystick (eixo Y para frente/trás, eixo X para giro)
        speed = -self.joystick.getRawAxis(1)
        turn = -self.joystick.getRawAxis(4)
        
        # Limita a velocidade e rotação para não ir muito rápido
        speed = max(min(speed, 0.5), -0.5)
        turn = max(min(turn, 0.45), -0.45)

        # Aplica a velocidade e rotação ao robô
        self.drivetrain.arcadeDrive(speed, turn)

        # Simulando um comando genérico na simulação: detecta se o botão x foi pressionado (ID 3 para o controle Xbox)
        # No Glass o estado do comando aparece no quadro NetworkTables
        # O aluno pode simular a quantidade de comandos que desejar
        button_x = self.joystick.getRawButton(3)
        self.update_command_state(button_x)
        if button_x:
            SmartDashboard.putString("Command Output", "Mecanismo ON")
        else:
            SmartDashboard.putString("Command Output", "Mecanismo OFF")
    
        # Atualiza a simulação
        self.current_speed = speed
        self.current_rotation = turn
        super().teleopPeriodic()

if __name__ == "__main__":
    import ntcore
    # Verifica se o script está sendo executado diretamente (e não importado como módulo).
    inst = ntcore.NetworkTableInstance.getDefault()
    # Obtém a instância padrão do NetworkTables para comunicação com ferramentas como o SmartDashboard.
    inst.startServer(port=5810)
    # Inicia o servidor do NetworkTables na porta 5810 para troca de dados.
    wpilib.run(MyRobot)
    # Inicia o programa do robô, executando a classe MyRobot usando o framework do WPILib.
