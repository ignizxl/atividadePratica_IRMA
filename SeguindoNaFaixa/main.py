#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import socket
import sys
import os  
import os.path
import time

TKEYPRESS = 250

def detectMotoresSensores(ev3, falar):
    # Motores
    try:
        motorLeft = Motor(Port.D)
    except OSError:
        if falar:
            ev3.speaker.say("Motor esquerdo não encontrado.")
            ev3.speaker.say("Conecte o motor esquerdo na porta D")
        return
    try:
        motorRight = Motor(Port.A)
    except OSError:
        if falar:
            ev3.speaker.say("Motor direito não encontrado.")
            ev3.speaker.say("Conecte o motor direito na porta A")
        return
    
    if falar:
        ev3.speaker.say("Motores encontrados")
    
    # Sensores
    infrared = None
    try:
        infrared = InfraredSensor(Port.S1)
    except OSError:
        if falar:
            ev3.speaker.say("Sensor infravermelho não encontrado.")
            ev3.speaker.say("Conecte o sensor infravermelho na porta S1")
    
    try:
        corLeft = ColorSensor(Port.S3)
    except OSError:
        if falar:
            ev3.speaker.say("Sensor esquerdo não encontrado.")
            ev3.speaker.say("Conecte o sensor de cor esquerdo na porta S3")
        return
    
    try: 
        corRight  = ColorSensor(Port.S2)
    except OSError:
        if falar:
            ev3.speaker.say("Sensor direito não encontrado.")
            ev3.speaker.say("Conecte o sensor de cor direito na porta S2")
        return
    
    if falar:
        ev3.speaker.say("Sensores encontrados")
    
    return (motorLeft, motorRight, infrared, corLeft, corRight)

def initNetwork(ev3, falar):
    if(falar):
        ev3.speaker.say("Iniciando interface de rede")
    HOST = '192.168.1.133'    # Endereco IP do Servidor
    PORT = 2508             # Porta que o Servidor esta
    udp  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (HOST, PORT)

    return (udp, dest)

def detectaNovoEstado(ev3, cronometro, estado, MotorSensor, parametros):
    
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor
    (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR) = parametros

    novoEstado = estado
    botoes = ev3.buttons.pressed()

    if((Button.CENTER in botoes) and (estado == "PARADO") and (cronometro.time()>=TKEYPRESS)):
        novoEstado = "ANDANDO"
        PoCorL     = corLeft.rgb()[2]
        PoCorR     = corRight.rgb()[2]
        cronometro.reset()
    elif((Button.CENTER in botoes) and (estado == "ANDANDO") and (cronometro.time()>=TKEYPRESS)):
        novoEstado = "PARADO"
        cronometro.reset()
    elif((estado == "ANDANDO") and False):
        pass

    parametros = (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR)
    return (novoEstado, parametros)

def executaEstado(ev3, MotorSensor, estado, parametros):
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor
    (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR) = parametros

    CorL = corLeft.rgb()[2]
    CorR = corRight.rgb()[2]
    if(estado == "ANDANDO"):
        if(CorL<10 and CorR<10): # Encruzilhada com preto
            # Anda em frente
            PotL = 50 
            PotR = 50
            motorLeft.dc(PotL)
            motorRight.dc(PotR)
            time.sleep(0.3)
        else:
            PotL = (PoCorL - CorL)*KpL + PoMotorL
            PotR = (PoCorR - CorR)*KpR + PoMotorR
            # É preciso fazer uma correção da potência dos motores para evitar
            # uma "zona morta", onde a potência submetida é baixa demais e o motor efetivamente 
            # não faz nada. Para ficar mais fácil para os alunos eu vou fazer o ajuste com um if
            # mas o legal mesmo seria fazer a modelagem do erro sistemático
            #print(PotL, PotR)

            if(-40<PotL and PotL<10):
                PotL = -40
            if(-40<PotR and PotR<10):
                PotR = -40
            motorLeft.dc(PotL)
            motorRight.dc(PotR)
    elif(estado == "PARADO"):
        motorLeft.brake()
        motorRight.brake()
    elif(estado == "VIRARDIREITA"):
        motorLeft.dc(50)
        motorRight.dc(50)
        time.sleep(0.4)
        motorLeft.brake()
        motorRight.brake()
        # while():
        #     motorLeft.dc(-40)
        #     motorRight.dc(40)

def enviaDados(conn, MotorSensor, contador):
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor
    (udp, dest) = conn

    left = corLeft.rgb()
    right= corRight.rgb()

    lr = "{0:.2f}".format(left[0])
    lg = "{0:.2f}".format(left[1])
    lb = "{0:.2f}".format(left[2])

    rr = "{0:.2f}".format(right[0])
    rg = "{0:.2f}".format(right[1])
    rb = "{0:.2f}".format(right[2])

    contador += 1

    msg = str(contador)+" "+lr+" "+lg+" "+lb+" "+rr+" "+rg+" "+rb

    udp.sendto(msg, dest)

    return contador

def aprendizagem(ev3, corLeft, corRight):
    pass

def passear(ev3, MotorSensor):
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor
    
    while(True):
        botoes = ev3.buttons.pressed()
        #Botoes
        if(Button.UP in botoes):
            motorRight.run(200)
            motorLeft.run(200)

        if(Button.DOWN in botoes):
            motorRight.run(-200)
            motorLeft.run(-200)

        if(Button.LEFT in botoes):
            motorRight.run(200)
            motorLeft.run(-200)

        if(Button.RIGHT in botoes):
            motorRight.run(-200)
            motorLeft.run(200)

        if(Button.CENTER in botoes):
            motorRight.brake()
            motorLeft.brake()
            botao = ev3.buttons.pressed()
            while(Button.CENTER not in botao):
                if(Button.UP in botao):
                    motorRight.run(200)
                    motorLeft.run(200)
                    break
                if(Button.CENTER in botao):
                    ev3.speaker.say("Fim do passeio")
                    return

        if(infrared != None and infrared.distance()<= 10):
            motorRight.brake()
            motorLeft.brake() 

def main(argv):
    rede = False
    fala = not "-f" in argv
    rede = "-n" in argv

    # Create your objects here.
    ev3 = EV3Brick()

    # Write your program here.
    ev3.speaker.set_speech_options('pt-br','m3')

    if(rede):
        conn = initNetwork(ev3, fala)

    MotorSensor = detectMotoresSensores(ev3, fala)
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor   

    cronometro = StopWatch()

    contador = 0
    PoCorL = 0
    KpL = -1.7 # A diferença no ganho é por causa da assimetria dos sensores
    PoMotorL = 60

    PoCorR = 0
    KpR = -1.2
    PoMotorR = 60

    parametros = (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR)
    estado = "PARADO"
    calibrado = False
    ev3.speaker.beep()
    while(True):
        botoes = ev3.buttons.pressed()
        if(Button.LELF in botoes):
            ev3.speaker.say("Vamos passear")
            passear(ev3, MotorSensor)
        
        if(Button.UP in botoes):
            ev3.speaker.say("Modo aprendizagem habilitado.")
            aprendizagem(ev3, corLeft, corRight)
            ev3.speaker.say("Aprendi o que eu queria.")
        
        if(rede):
            contador = enviaDados(conn, MotorSensor, contador)
        
        if(Button.CENTER in botoes):
            NovoEstado, parametros = detectaNovoEstado(ev3, cronometro, estado, MotorSensor, parametros)
            
            executaEstado(ev3, MotorSensor, NovoEstado, parametros)
            
            estado = NovoEstado

main(sys.argv)