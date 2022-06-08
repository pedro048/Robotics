#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Projeto manipulador robotico
"""

from ufrn_al5d import RoboticArmAL5D
import time
import math


###################################
######    CANAIS DOS SERVOS  ###### 
###### E LIMITES DE OPERAÇÃO ######
###################################
#0. BASE
BAS_SERVO = 0
#LIMITES
BAS_MIN = 500
BAS_MAX = 2400

#1. SHOULDER
SHL_SERVO = 1
#LIMITES
SHL_MIN = 1200
SHL_MAX = 2000

#2. ELBOW
ELB_SERVO = 2
#LIMITES
ELB_MIN = 1100
ELB_MAX = 2000

#3. WRIST
WRI_SERVO = 3
#LIMITES
WRI_MIN = 500
WRI_MAX = 2500

#4. GRIPPER
GRI_SERVO = 4
#LIMITES
GRI_MIN = 1300
GRI_MAX = 2400

#PROPRIEDADES DO BRAÇO: SERVOS E LIMITES DE OPERACAO
properties = [BAS_SERVO, BAS_MIN, BAS_MAX,
              SHL_SERVO, SHL_MIN, SHL_MAX,
              ELB_SERVO, ELB_MIN, ELB_MAX,
              WRI_SERVO, WRI_MIN, WRI_MAX,
              GRI_SERVO, GRI_MIN, GRI_MAX]
##################################

##################################
#######    PROGRAMA DEMO   #######
##################################

#POSICAO INICIAL PARA TODOS OS SERVOS
HOME_POS = '#0P1500#1P1500#2P1500#3P1500#4P1500T1500'
p0=1500
p1=1500
p2=1500
p3=1500
p4=1500

# Está em cm
L1 = 8 
L2 = 14.5
L3 = 18.5
L4 = 6


#INICIALIZACAO DO BRACO PASSANDO AS PROPRIEDADES COMO PARAMETRO
braco = RoboticArmAL5D(properties)

#CONFIGURACAO DA PORTA
braco.setup()

def move(x, y, z, phi):
   '''
    theta1 = math.atan2(y, x)	# angula da junta 1
    x_l = x - L4*math.cos(math.radians(phi)) 
    z_l = z - L4*math.sin(math.radians(phi)) - L1
    theta2 = math.atan2(z_l, x_l) + math.acos((-L3**2+L2**2+x_l**2+z_l**2)/(2*L2*math.sqrt(x_l**2+z_l**2))) # angula da junta 2
    theta3 = math.acos(((x-L4*math.cos(math.radians(phi)))**2 + (z-L4*math.sin(math.radians(phi))-L1)**2 - L2**2 - L3**2)/(2*L2*L3)) # angula da junta 3
    theta4 = phi - theta2 - theta3 # angula da junta 4	
'''
   theta1 = math.atan2(y, x)	# angula da junta 1
    x_l = x - L4*math.cos(phi) 
    z_l = z - L4*math.sin(phi) - L1
    theta2 = math.atan2(z_l, x_l) + math.acos((-L3**2+L2**2+x_l**2+z_l**2)/(2*L2*math.sqrt(x_l**2+z_l**2))) # angula da junta 2
    theta3 = math.acos(((x-L4*math.cos(phi)**2 + (z-L4*math.sin(phi)-L1)**2 - L2**2 - L3**2)/(2*L2*L3)) # angula da junta 3
    theta4 = phi - theta2 - theta3 # angula da junta 4	

    valor_servo1 = valorServo1(theta1)
    print(theta1)
    print(valor_servo1)
    valor_servo2 = valorServo2(theta2)
    print(theta2)
    print(valor_servo2)
    valor_servo3 = valorServo3(theta3)
    print(theta3)
    print(valor_servo3)
    valor_servo4 = valorServo4(theta4)
    print(theta4)
    print(valor_servo4)

    braco.envia_comando('#%dP%dT%d' % (0, valor_servo1,1500))
    braco.envia_comando('#%dP%dT%d' % (1, valor_servo2,1500))
    braco.envia_comando('#%dP%dT%d' % (2, valor_servo3,1500))
    braco.envia_comando('#%dP%dT%d' % (3, valor_servo4,1500))
    #print(math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), math.degrees(theta4))
    #print(theta1, theta2, theta3, theta4)
    #try:
        #braco.envia_comando('#%dP%dT%d' % (0,2000,1500))
	#braco.envia_comando('#%dP%dT%d' % (1,2000,1500))
	#braco.envia_comando('#%dP%dT%d' % (2,1500,1500))
	#braco.envia_comando('#%dP%dT%d' % (3,1800,1500))
    
    #except:
       # print('Problema no envio do comando\nAbortando o programa...')


def funcaoPonto(p1, p2, p3, p4):
	
    theta1 = -0.1*p1+150
    theta2 = 0.09*p2-45
    theta3 = -0.11*p3+79
    theta4 = 0.1*p4-150
    vetor = [theta1, theta2, theta3, theta4]
    return vetor

'''
def valoresServo(theta1, theta2, theta3, theta4):
	
    servo1 = (theta1 - 150)/(-0.1)
    servo2 = (theta2 + 45)/(0.09)
    servo3 = (theta3 - 79)/(-0.11)
    servo4 = (theta4 + 150)/(0.1)
 
    return [servo1, servo2, servo3, servo4] 
'''


def valorServo1(theta1):
	servo1 = (theta1 - 150)/(-0.1)

	return servo1

def valorServo2(theta2):
	servo2 = (theta2 + 45)/(0.09)

	return servo2

def valorServo3(theta3):
	servo3 = (theta3 - 79)/(-0.11)

	return servo3

def valorServo4(theta4):
	servo4 = (theta4 + 150)/(0.1)

	return servo4
 


def calculaSomaCosDois(theta1, theta2):
    somaCos = 0;
    theta1_rad = (theta1*math.pi)/180
    theta2_rad = (theta2*math.pi)/180
    somaCos = (math.cos(theta1_rad)*math.cos(theta2_rad)) - (math.sin(theta1_rad)*math.sin(theta2_rad))
    return somaCos

def calculaSomaSenDois(theta1, theta2):
    somaSen = 0;
    theta1_rad = (theta1*math.pi)/180
    theta2_rad = (theta2*math.pi)/180
    somaSen = (math.sin(theta1_rad)*math.cos(theta2_rad)) + (math.sin(theta2_rad)*math.cos(theta1_rad))
    return somaSen

def calculaXYZ(theta1, theta2, theta3, theta4):
    x = 0
    y = 0
    z = 0
    c23 = 0
    c234 = 0
    s23 = 0
    s234 = 0
    theta1_rad = (theta1*math.pi)/180
    theta2_rad = (theta2*math.pi)/180
    c23 = calculaSomaCosDois(theta2, theta3)
    c234 = calculaSomaCosDois(c23, theta4)
    s23 = calculaSomaSenDois(theta2, theta3)
    s234 = calculaSomaSenDois(s23, theta4)

    
    x = ((math.cos(theta1_rad)*c234*L4) + (math.cos(theta1_rad)*c23*L3) + (math.cos(theta1_rad)*math.cos(theta2_rad)*L2))
    y = ((math.sin(theta1_rad)*c234*L4) + (math.sin(theta1_rad)*c23*L3) + (math.sin(theta1_rad)*math.cos(theta2_rad)*L2)) 
    z = ((s234*L4) + (s23*L3) + (math.sin(theta2_rad)*L2) + L1) 
    vetor_coord = [x,y,z]
    return vetor_coord


def calculaOrientacao(theta1,theta2,theta3,theta4):
   a11 = 0
   a12 = 0
   a13 = 0
   a21 = 0
   a22 = 0
   a23 = 0
   c23 = 0
   a31 = 0 
   a32 = 0
   a33 = 0
   c234 = 0
   s23 = 0
   s234 = 0
   theta1_rad = (theta1*math.pi)/180
   c23 = calculaSomaCosDois(theta2, theta3)
   c234 = calculaSomaCosDois(c23, theta4)
   s23 = calculaSomaSenDois(theta2, theta3)
   s234 = calculaSomaSenDois(s23, theta4)

   a11 = math.cos(theta1_rad)*c234
   a12 = -math.cos(theta1_rad)*s234
   a13 = math.sin(theta1_rad)
   a21 = math.sin(theta1_rad)*c234
   a22 = -math.sin(theta1_rad)*s234
   a23 = -math.cos(theta1_rad)
   a31 = s234
   a32 = c234
   a33 = 0

   return (a11,a12,a13,a21,a22,a23,a31,a32,a33)



#ABRINDO A PORTA
if(braco.abre_porta() == -1):
    print ('Erro abrindo a porta serial /dev/ttyS0\nAbortando o programa...\n')
else: 
    print('PROGRAMA DEMONSTRACAO INICIADO\n\n');
    print ('Porta serial /dev/ttyS0 aberta com sucesso\n')
   

    #############################
    ##### POSICAO INICIAL ######
    #############################
    
    print('\nPOSICAL INICIAL\n')
    try:
        braco.envia_comando(HOME_POS)
        print(' Envio de comando com teste de envio: %s \n' % (HOME_POS))
    except:
        print('Problema no envio do comando\nAbortando o programa...')
        
	
    ###################################
    ##### MANIPULAÇÃO DAS JUNTAS ######
    ###################################

    print('\nMANIPULAÇÃO DAS JUNTAS\n')	
    print('Espere 2 segundos...\n')
    time.sleep(2)
    op = 100
    x = 0
    y = 0
    z = 0
#   vetor_v1 = funcaoPonto(1525, 1600, 1825, 1500)
#   print(vetor_v1)
#    vetor_v2 = funcaoPonto(1450, 1400, 1675, 1500)
#   print(vetor_v2)
 #  vetor_v3 = funcaoPonto(1675, 1400, 1800, 1500)
#  print(vetor_v3)
  #  vetor_v4 = funcaoPonto(1450, 1625, 2000, 1375)
  #  print(vetor_v4)
    
    '''
    vet1_coord = calculaXYZ(vetor_v1[0],vetor_v1[1],vetor_v1[2],vetor_v1[3])
    vet2_coord = calculaXYZ(vetor_v2[0],vetor_v2[1],vetor_v2[2],vetor_v2[3])
    vet3_coord = calculaXYZ(vetor_v3[0],vetor_v3[1],vetor_v3[2],vetor_v3[3])
    vet4_coord = calculaXYZ(vetor_v4[0],vetor_v4[1],vetor_v4[2],vetor_v4[3])

    print(vet1_coord)
    print(vet2_coord)
    print(vet3_coord)
    print(vet4_coord)
 
    '''
    while(op != 10): 
	op = raw_input('Digite...')
        if(op == "ABRE_GARRA"):
	      try:
        	 braco.envia_comando('#%dP%dT%d' % (4,1500,1500))
    	      except:
        	 print('Problema no envio do comando\nAbortando o programa...')

	if(op == "FECHA_GARRA"):
	      try:
                 braco.envia_comando('#%dP%dT%d' % (4,2300,1500))
    	      except:
                 print('Problema no envio do comando\nAbortando o programa...')
  
        if(op == "REPOUSO"):
              braco.envia_comando(HOME_POS)

	if(op == 'MOVE'):	
		move(11, 5, 6, 45)

        '''
	#JUNTA 0 - JUNTA ROTACIONAL
	#Horario
	if op == 0:
		if p0 < BAS_MAX: 
			p0 = p0+25
		braco.envia_comando('#%dP%dT%d' % (0,p0,1500))
		print('p0 = ',p0)
	#Anti-horario
	if op == 1:
		if p0 > BAS_MIN: 
			p0 = p0-25
		braco.envia_comando('#%dP%dT%d' % (0,p0,1500))
		print('p0 = ',p0)
	#JUNTA 1
	#Anti-horario
       	if op == 2:
		if p1 < SHL_MAX:
			p1 = p1+25
		braco.envia_comando('#%dP%dT%d' % (1,p1,1500))
		print('p1 = ',p1)
	#Horario
       	if op == 3:
		if p1 > SHL_MIN:
			p1 = p1-25
		braco.envia_comando('#%dP%dT%d' % (1,p1,1500))
		print('p1 = ',p1)
	#JUNTA 2
	#Horario
       	if op == 4:
		if p2 < ELB_MAX:
			p2 = p2+25
		braco.envia_comando('#%dP%dT%d' % (2,p2,1500))
		print('p2 = ',p2)
	#Anti-horario
       	if op == 5:
		if p2 > ELB_MIN:
			p2 = p2-25
		braco.envia_comando('#%dP%dT%d' % (2,p2,1500))
		print('p2 = ',p2)
	#JUNTA 3
       	if op == 6:
		if p3 < WRI_MAX:
			p3 = p3+25
		braco.envia_comando('#%dP%dT%d' % (3,p3,1500))
		print('p3 = ',p3)
       	if op == 7:
		if p3 > WRI_MIN: 
			p3 = p3-25
		braco.envia_comando('#%dP%dT%d' % (3,p3,1500))
		print('p3 = ',p3)
	#JUNTA 4 - GARRA
        if op == 8:
		if p4 < GRI_MAX:
			p4 = p4+25
		braco.envia_comando('#%dP%dT%d' % (4,p4,1500))
		print('p4 = ',p4)
        if op == 9:
		if p4 > GRI_MIN:
			p4 = p4-25
		braco.envia_comando('#%dP%dT%d' % (4,p4,1500))
		print('p4 = ',p4)
	if op == 11:
		print('junta0 = ',p0, 'junta1 = ',p1, 'junta2 = ',p2, 'junta3 = ',p3, 'junta4 =',p4)
        x, y, z = calculaX(p1,p2,p3,p4)
        #print("x = ", x, "y = ", y, "z = ")
        
        '''
	
    #############################
    ###### QUARTO COMANDO  ######
    ###### TESTE DE TRAVAS ######
    #############################
    braco.envia_comando(HOME_POS)
    print('\nQUARTO COMANDO - MOVER A BASE TESTANDO TRAVAS\n')
    print('Espere 2 segundos...\n')
    time.sleep(2)
    try:
        #FUNCAO TRAVA (trava) RECEBE COMO PARAMETROS
        #O SERVO E O VALOR DA POSICAO DESEJADA E
        #RETORNA A POSICAO CORRIGIDA DE ACORDO COM OS LIMITES MAX E MIN
        #ANTERIORMENTE ESTABELECIDOS
        pos = braco.trava(BAS_SERVO,99999)
        braco.envia_comando('#%dP%dT%d' % (BAS_SERVO,pos,1500))
        print('Envio de comando com teste de envio e de travas: %s \n' % ('#0P%sT1500' % (pos)))
    except:
        print('Problema no envio do comando\nAbortando o programa...')
        
    ##FIM DO PROGRAMA DEMO##
    braco.fecha_porta()
    print('\nAcesso a porta serial /dev/ttyS0 finalizado\n')
    
print('\nPROGRAMA DEMONSTRACAO FINALIZADO\n\n')

