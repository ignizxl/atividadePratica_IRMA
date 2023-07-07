#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Create the sensors and motors objects
ev3 = EV3Brick()

import time

TKEYPRESS = 250

def detectMotoresSensores(ev3, falar):
    # Motores
    try:
        motorLeft = Motor(Port.A)
    except OSError:
        if falar:
            ev3.speaker.say("Motor esquerdo não encontrado.")
            ev3.speaker.say("Conecte o motor esquerdo na porta D")
        return
    try:
        motorRight = Motor(Port.B)
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
        infrared = InfraredSensor(Port.S2)
    except:
        if falar:
            ev3.speaker.say("Sensor infravermelho não encontrado.")
            ev3.speaker.say("Conecte o sensor infravermelho na porta S1")
        
    
    try:
        corLeft = ColorSensor(Port.S1)
    except OSError:
        if falar:
            ev3.speaker.say("Sensor de cor esquerdo não encontrado.")
            ev3.speaker.say("Conecte o sensor de cor esquerdo na porta S3")
        return
    
    try: 
        corRight  = ColorSensor(Port.S4)
    except OSError:
        if falar:
            ev3.speaker.say("Sensor de cor direito não encontrado.")
            ev3.speaker.say("Conecte o sensor de cor direito na porta S2")
        return
    
    if falar:
        ev3.speaker.say("Sensores encontrados")
    
    return (motorLeft, motorRight,infrared, corLeft, corRight)

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
            break

        if(infrared != None and infrared.distance() <= 10):
            motorRight.brake()
            motorLeft.brake() 
    ev3.speaker.say("Fim do passeio!")

def leituraCor(ev3, corLeft, corRight, iteracoes):
    corRL, corGL, corBL, corRR, corGR, corBR = 0,0,0,0,0,0
    for i in range(iteracoes):
        medicaoLeft = corLeft.rgb()
        medicaoRight = corRight.rgb()
        
        corRL += medicaoLeft[0]
        corGL += medicaoLeft[1]
        corBL += medicaoLeft[2]

        corRR += medicaoRight[0]
        corGR += medicaoRight[1]
        corBR += medicaoRight[2]
    
    corRL = corRL/iteracoes
    corGL = corGL/iteracoes
    corBL = corBL/iteracoes

    corRR = corRR/iteracoes
    corGR = corGR/iteracoes
    corBR = corBR/iteracoes

    return (corRL, corGL, corBL, corRR, corGR, corBR)

def Sig(m2,n,M):
    return (m2/n - M**2)**(1/2)

def Mi(m,n):
    return m/n

def leituraCorErroAleatorio(ev3, corLeft, corRight, iteracoes):
    corRLM, corGLM, corBLM, corRRM, corGRM, corBRM = 0,0,0,0,0,0
    corRLM2, corGLM2, corBLM2, corRRM2, corGRM2, corBRM2 = 0,0,0,0,0,0

    for i in range(iteracoes):
        medicaoLeft = corLeft.rgb()
        medicaoRight = corRight.rgb()
        
        corRLM += medicaoLeft[0]
        corGLM += medicaoLeft[1]
        corBLM += medicaoLeft[2]
        corRLM2 += (medicaoLeft[0]**2)
        corGLM2 += (medicaoLeft[1]**2)
        corBLM2 += (medicaoLeft[2]**2)

        corRRM += medicaoRight[0]
        corGRM += medicaoRight[1]
        corBRM += medicaoRight[2]
        corRRM2 += (medicaoRight[0]**2)
        corGRM2 += (medicaoRight[1]**2)
        corBRM2 += (medicaoRight[2]**2)
    
    mi = [Mi(corRLM/iteracoes),Mi(corGLM/iteracoes),Mi(corBLM/iteracoes),
          Mi(corRRM/iteracoes),Mi(corGRM/iteracoes),Mi(corBRM/iteracoes)]
    
    sig = [Sig(corRLM2,iteracoes,mi[0]),
           Sig(corGLM2,iteracoes,mi[1]),
           Sig(corBLM2,iteracoes,mi[2]),
           Sig(corRRM2,iteracoes,mi[3]),
           Sig(corGRM2,iteracoes,mi[4]),
           Sig(corBRM2,iteracoes,mi[5])]
    
    return (mi, sig)

def salvaCalibracao(leitura,nome):
    (corRL, corGL, corBL, corRR, corGR, corBR) = leitura
    arquivo = open(nome+".csv", "w")
    arquivo.write(str(corRL)+";"+str(corGL)+";"+str(corBL)+";"+str(corRR)+";"+str(corGR)+";"+str(corBR))
    arquivo.close()

def aprendizagem(ev3, MotorSensor):
    (motorLeft, motorRight,infrared, corLeft, corRight) = MotorSensor
    referenciaCores = {}
    opcoes = {"LEFT":"Cor VIRA ESQUERDA", "RIGHT":"Cor VIRA DIREITA", "UP":"COR PARAR", "CENTER":"PARAR APRENDIZADO"}
    menu(ev3, opcoes)
    while(True):
        botoes = ev3.buttons.pressed()
        if(Button.LEFT in botoes):
            ev3.speaker.say("Aprendendo a cor ViraEsquerda")
            corViraEsquerda = leituraCorErroAleatorio(ev3, corLeft, corRight, 100)
            referenciaCores["VIRAESQUERDA"] = corViraEsquerda
            print(corViraEsquerda)
            ev3.speaker.beep()
            
        if(Button.RIGHT in botoes):
            ev3.speaker.say("Aprendendo a cor ViraDireira")
            corViraDireira = leituraCorErroAleatorio(ev3, corLeft, corRight, 100)
            referenciaCores["VIRADIREITA"] = corViraDireira
            print(corViraDireira)
            ev3.speaker.beep()
        
        if(Button.UP in botoes):
            ev3.speaker.say("Aprendendo a cor PARAR")
            corParar = leituraCorErroAleatorio(ev3, corLeft, corRight, 100)
            referenciaCores["PARADO"] = corParar
            print(corParar)
            ev3.speaker.beep()
            
        if(Button.CENTER in botoes):
            ev3.speaker.say("Aprendizado finalizado")
            break
    return referenciaCores

def compararCor(medicaoLeft, medicaoRight, corReferencia):
    sig = 2
    if((corReferencia[0] - sig <= medicaoLeft[0] <= corReferencia[0] + sig) and
       (corReferencia[1] - sig <= medicaoLeft[1] <= corReferencia[1] + sig) and
       (corReferencia[2] - sig <= medicaoLeft[2] <= corReferencia[2] + sig) and
       (corReferencia[3] - sig <= medicaoRight[0] <= corReferencia[3] + sig) and
       (corReferencia[4] - sig <= medicaoRight[1] <= corReferencia[4] + sig) and
       (corReferencia[5] - sig <= medicaoRight[2] <= corReferencia[5] + sig)):
        return True
    return False

def compararCorErroAleatorio(medicaoLeft, medicaoRight, corReferencia):
    x = 1
    (mi, sig) = corReferencia
    if(((mi[0] - (sig[0]*x)) <= medicaoLeft[0]  <= (mi[0] + (sig[0]*x))) and
       ((mi[1] - (sig[1]*x)) <= medicaoLeft[1]  <= (mi[1] + (sig[1]*x))) and
       ((mi[2] - (sig[2]*x)) <= medicaoLeft[2]  <= (mi[2] + (sig[2]*x))) and
       ((mi[3] - (sig[3]*x)) <= medicaoRight[0] <= (mi[3] + (sig[3]*x))) and
       ((mi[4] - (sig[4]*x)) <= medicaoRight[1] <= (mi[4] + (sig[4]*x))) and
       ((mi[5] - (sig[5]*x)) <= medicaoRight[2] <= (mi[5] + (sig[5]*x)))):
        return True
    return False

def detectaNovoEstado(ev3, cronometro, estado, MotorSensor, parametros, referenciasCores):
    
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor
    (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR) = parametros

    novoEstado = estado
    botoes = ev3.buttons.pressed()

    if((Button.CENTER in botoes) and (estado == "PARADO")):
        novoEstado = "ANDANDO"
        PoCorL = corLeft.rgb()[2]
        PoCorR = corRight.rgb()[2]
    elif((Button.CENTER in botoes) and (estado == "ANDANDO")):
        novoEstado = "PARADO"
    elif((estado == "ANDANDO") and compararCor(corLeft.rgb(),corRight.rgb(), referenciasCores["VIRAESQUERDA"])):
        novoEstado = "VIRAESQUERDA"
    elif((estado == "ANDANDO") and compararCor(corLeft.rgb(),corRight.rgb(), referenciasCores["VIRADIREITA"])):
        novoEstado = "VIRADIREITA"
    elif((estado == "ANDANDO") and compararCor(corLeft.rgb(),corRight.rgb(), referenciasCores["PARADO"])):
        novoEstado = "PARADO"

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
            
            if(-40<PotL and PotL<10):
                PotL = -40
            if(-40<PotR and PotR<10):
                PotR = -40
            motorLeft.dc(PotL)
            motorRight.dc(PotR)
    elif(estado == "PARADO"):
        motorLeft.brake()
        motorRight.brake()
    elif(estado == "VIRAESQUERDA"):
        motorLeft.run(50)
        motorRight.run(50)
        time.sleep(0.4)
        motorLeft.brake()
        motorRight.brake()
        motorLeft.run(-100)
        motorRight.run(100)
        time.sleep(0.2)
    elif(estado == "VIRADIREITA"):
        motorLeft.run(50)
        motorRight.run(50)
        time.sleep(0.4)
        motorLeft.brake()
        motorRight.brake()
        motorLeft.run(100)
        motorRight.run(-100)
        time.sleep(0.2)
    elif(estado == "PARADO"):
        motorLeft.brake()
        motorRight.brake()

def segueFaixa(ev3, MotorSensor):
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor   

    cronometro = 0

    contador = 0
    PoCorL = 0
    KpL = -1.7 # A diferença no ganho é por causa da assimetria dos sensores
    PoMotorL = 60

    PoCorR = 0
    KpR = -1.2
    PoMotorR = 60
    parametros = (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR)
    referenciasCores = {}
    
    estado = "PARADO"
    while(True):
        NovoEstado, parametros = detectaNovoEstado(ev3, cronometro, estado, MotorSensor, parametros)
            
        executaEstado(ev3, MotorSensor, NovoEstado, parametros, referenciasCores)
            
        estado = NovoEstado

def menu(ev3, dicOpcoes):
    for botao, opcao in dicOpcoes.items():
        ev3.speaker.say("Botão "+botao+" : "+opcao)

def main():
    fala = False

    # Create your objects here.
    ev3 = EV3Brick()

    # Write your program here.
    ev3.speaker.set_speech_options('pt-br','m3')

    MotorSensor = detectMotoresSensores(ev3, fala)

    ev3.speaker.beep()
    opcoesMenuGeral = {"LEFT":"MODO PASSEAR","RIGHT":"MODO APRENDIZAGEM","UP":"SEGUE FAIXA","DOWN":"Finalizar Programa"}
    menu(ev3, opcoesMenuGeral)

    referenciasCores = {}
    while(True):
        botoes = ev3.buttons.pressed()
        if(Button.LEFT in botoes):
            ev3.speaker.say("Vamos passear")
            passear(ev3, MotorSensor)
        
        if(Button.RIGHT in botoes):
            if(referenciasCores.get("VIRAESQUERDA") == None and referenciasCores.get("VIRADIREITA") == None and referenciasCores.get("PARADO") == None):
                ev3.speaker.say("Modo aprendizagem, eu sou muito burro kk")
                referenciasCores = aprendizagem(ev3, MotorSensor)
                ev3.speaker.beep()
            else:
                ev3.speaker.say("Eu sou inteligente agora")
                ev3.speaker.beep()
        
        if(Button.UP in botoes):
            if(referenciasCores.get("VIRAESQUERDA") == None and referenciasCores.get("VIRADIREITA") == None and referenciasCores.get("PARADO") == None):
                ev3.speaker.say("Preciso aprender para andar")
                ev3.speaker.beep()
            else :
                ev3.speaker.say("Eu sei seguir faixa")
                segueFaixa(ev3, MotorSensor, referenciasCores)
                ev3.speaker.beep()
        
        if(Button.DOWN in botoes):
            ev3.speaker.say("Finalizando programa.")
            break

main()
ev3.speaker.beep()