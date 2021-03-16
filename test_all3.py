import sqlite3
import RPi.GPIO as GPIO
import time
import math
import numpy as np
import sys
sys.path.append('/usr/local/lib/python2.7/dist-packages')
import mysql.connector as connect
import smbus
import multiprocessing

# GPIO.cleanup()
## Variable gyroscope
global address
power_mgmt_1=0x6b
power_mgmt_2=0x6c
bus=smbus.SMBus(1)
address=0x68
AxCal=0
AyCal=0
AzCal=0
GxCal=0
GyCal=0
GzCal=0

## Variable  moteur
moteurGIn1=12
moteurGIn2=16
moteurDIn1=22
moteurDIn2=18
moteurGpwm=32
moteurDpwm=33
moteurGEncoder1=11
moteurGEncoder2=13
moteurDEncoder1=15
moteurDEncoder2=29

global previousPosG, previousPosD, previoustG, previoustD, tG, tD, posG, posD
global KpG, KiG, KpD, KiD, sommeErreurG, sommeErreurD, erreurG, erreurD, maxSpeed
global consigneG, consigneD, moteurGPositif, moteurDPositif
previousPosG=0
previousPosD=0
previoustG=0
previoustD=0
tG=0
tD=0
posG=0
posD=0
moyenneG=np.zeros(3)
moyenneD=np.zeros(3)
vitesseG=0
vitesseD=0
KpG=1.37
KiG=0.17
KpD=1.35
KiD=0.15
sommeErreurG=0.0
sommeErreurD=0.0
erreurG=0.0
erreurD=0.0
consigneG=0
consigneD=0
moteurGPositif=True
moteurDPositif=True
maxSpeed=102

global Xactuel, Yactuel, TcalculPos, currentAngle, path
Xactuel=0
Yactuel=0
TcalculPos=0
currentAngle=0
path=np.array(([]))

##Definition fonction moteur
def moyenneGlissante(tabMoyenne,valeur):
    l=len(tabMoyenne)
    moyenne=valeur
    element=1
    for i in range(l):
        if(tabMoyenne[i]!=0):
            moyenne=moyenne+tabMoyenne[i]
            element+=1
    moyenne=moyenne/element
    tabMoyenne[0]=tabMoyenne[1]
    tabMoyenne[1]=tabMoyenne[2]
    tabMoyenne[2]=valeur
    return tabMoyenne,moyenne

def correcteurG():
    global vitesseG, moyenneG, tG, previoustG, posG, previousPosG, erreurG, sommeErreurG, KpG, KiG, consigneG, pwmG, moteurGPositif
    tG=time.time_ns()
    #print(posG)
    vitesseG=1e9*(posG-previousPosG)/(tG-previoustG)
    vitesseG=vitesseG*60/700
    previoustG=tG
    previousPosG=posG
    if(moteurGPositif==False):
        vitesseG=-vitesseG
    moyenneG,vitesseG=moyenneGlissante(moyenneG,vitesseG)
    erreurG=consigneG-vitesseG
    sommeErreurG=sommeErreurG+erreurG
    #print('Vitesse G: '+str(vitesseG))
    #print('Erreur :'+str(erreurG))
    #print('SommeErreur :'+str(sommeErreurG))
    commandSpeedG=erreurG*KpG+sommeErreurG*KiG
    if(commandSpeedG<0):
        commandSpeedG=-commandSpeedG
    if(commandSpeedG>102):
        commandSpeedG=102
    #print(math.floor(commandSpeedG*100/141))
    pwmG.ChangeDutyCycle(math.floor(commandSpeedG*42/102))

def correcteurD():
    global vitesseD, moyenneD, tD, previoustD, posD, previousPosD, erreurD, sommeErreurD, KpD, KiD, consigneD, pwmD, moteurDPositif
    tD=time.time_ns()
    vitesseD=1e9*(posD-previousPosD)/(tD-previoustD)
    vitesseD=vitesseD*60/700
    previoustD=tD
    previousPosD=posD
    if(moteurDPositif==False):
        vitesseD=-vitesseD
    moyenneD,vitesseD=moyenneGlissante(moyenneD,vitesseD)
    erreurD=consigneD-vitesseD
    sommeErreurD=sommeErreurD+erreurD
    #print('Vitesse D: '+str(vitesseD))
    commandSpeedD=erreurD*KpD+sommeErreurD*KiD
    if(commandSpeedD<0):
        commandSpeedD=-commandSpeedD
    if(commandSpeedD>102):
        commandSpeedD=102
    pwmD.ChangeDutyCycle(math.floor(commandSpeedD*42/102))

def frontG(self):
    #print("front")
    global moteurGEncoder1, moteurGEncoder2, posG
    posG=posG+1
    ##Le compteur ne fonctionne pas bien avec les 2 voies
    ##On ne compte donc qu'avec une
    ##Le sens de rotation est connu de base donc ça n'est pas un problème
    ##Ce code se lançant à chaque front, on compte chaque front
    #sA=GPIO.input(moteurGEncoder1)
    #sB=GPIO.input(moteurGEncoder2)
    #if(sA==sB):
    #    posG=posG+1
    #else :
    #    posG=posG-1
    #print(posG)
        
def frontD(self):
    global posD, moteurDEncoder1, moteurDEncoder2
    posD=posD+1
    #sA=GPIO.input(moteurDEncoder1)
    #sB=GPIO.input(moteurDEncoder2)
    #if(sA==sB):
    #    posD=posD+1
    #else :
    #    posD=posD-1
    #print(posD)

def stop():
    GPIO.output(moteurGIn1,GPIO.LOW)
    GPIO.output(moteurGIn2,GPIO.LOW)
    GPIO.output(moteurDIn1,GPIO.LOW)
    GPIO.output(moteurDIn2,GPIO.LOW)
    pwmG.ChangeDutyCycle(0)
    pwmD.ChangeDutyCycle(0)
    
def forward_mg(speed):
    global consigneG, maxSpeed, moteurGPositif
    if(speed>maxSpeed):
        speed=maxSpeed
    GPIO.output(moteurGIn1,GPIO.LOW)
    GPIO.output(moteurGIn2,GPIO.LOW)
    consigneG=0
    time.sleep(0.05)
    moteurGPositif=True
    consigneG=speed
    GPIO.output(moteurGIn1,GPIO.HIGH)
    GPIO.output(moteurGIn2,GPIO.LOW)
    pwmG.ChangeDutyCycle(speed*42/maxSpeed)
    
def forward_md(speed):
    global consigneD, maxSpeed, moteurDPositif
    if(speed>maxSpeed):
        speed=maxSpeed
    GPIO.output(moteurDIn1,GPIO.LOW)
    GPIO.output(moteurDIn2,GPIO.LOW)
    consigneD=0
    time.sleep(0.05)
    moteurDPositif=True
    consigneD=speed
    GPIO.output(moteurDIn1,GPIO.HIGH)
    GPIO.output(moteurDIn2,GPIO.LOW)
    pwmD.ChangeDutyCycle(speed*42/maxSpeed)
    
def reverse_mg(speed):
    global consigneG, maxSpeed, moteurGPositif
    if(speed<0):
        speed=-speed
    if(speed>maxSpeed):
        speed=maxSpeed
    GPIO.output(moteurGIn1,GPIO.LOW)
    GPIO.output(moteurGIn2,GPIO.LOW)
    consigneG=0
    time.sleep(0.05)
    moteurGPositif=False
    consigneG=speed
    GPIO.output(moteurGIn2,GPIO.HIGH)
    pwmG.ChangeDutyCycle(speed*42/maxSpeed)
    
def reverse_md(speed):
    global consigneD, maxSpeed, moteurDPositif
    if(speed<0):
        speed=-speed
    if(speed>maxSpeed):
        speed=maxSpeed
    GPIO.output(moteurDIn1,GPIO.LOW)
    GPIO.output(moteurDIn2,GPIO.LOW)
    consigneD=0
    time.sleep(0.05)
    moteurDPositif=False
    consigneD=speed
    GPIO.output(moteurDIn2,GPIO.HIGH)
    pwmD.ChangeDutyCycle(speed*42/maxSpeed)
    
def forward(speed):
    forward_mg(speed)
    forward_md(speed)
    
def reverse(speed):
    reverse_mg(speed)
    reverse_md(speed)
    
def turn_left(speed,alpha):
    global currentAngle
    start=time.time()
    arret=0
    reverse_mg(speed)
    forward_md(speed)
    while(arret==0):
        print('Angle :'+str(currentAngle))
        currentYaw=-getYaw()
        elapse=time.time()-start
        currentAngle=currentAngle+(elapse*currentYaw)
        start=time.time()
        if (currentAngle<=alpha+1 and currentAngle>=alpha-1):
            stop()
            arret=1

def turn_right(speed,alpha):
    global currentAngle
    start=time.time()
    arret=0
    forward_mg(speed)
    reverse_md(speed)
    print(alpha)
    while(arret==0):
        currentYaw=-getYaw()
        elapse=time.time()-start
        currentAngle=currentAngle+(elapse*currentYaw)
        start=time.time()
        print('Angle :'+str(currentAngle))
        if(currentAngle>=alpha-1):
            stop()
            arret=1
        
    
## Définition fonction bdd
global executionDep, heureOrdre
executionDep=False
heureOrdre=0
    
def recupererBDD(queueBdd):
    BDD=connect.connect(host="192.168.4.1",user="root",password="rpiSMR2",database="projet")
    curseur=BDD.cursor()
    curseur.execute("SELECT heure,destinataire,etat,typeOrdre,syntaxeOrdre FROM ordre WHERE destinataire=3")
    lecture=True
    while lecture:
        ligne=curseur.fetchone()
        if ligne==None:
            lecture=False
        elif ligne[1]==3:
            if ligne[2]==0:
                try:
                    curseur.fetchall()
                except connect.errors.InterfaceError as ie:
                    if ie.msg == 'No result set to fetch from.':
                        pass
                    else:
                        raise
                #curseur2=BDD.cursor()
                curseur.execute("UPDATE ordre SET etat=1 WHERE destinataire = 3 AND etat = 0 ORDER BY heure LIMIT 1")
                BDD.commit()
                queueBdd.put(ligne)
                lecture=False
            elif ligne[2]==1 or ligne[2]==2:
                queueBdd.put(ligne)
                lecture=False
            elif ligne[2]==3:
                #curseur2=BDD.cursor()
                query="UPDATE ordre SET etat=4 WHERE destinataire = 3 AND etat = 0 ORDER BY heure LIMIT 1"
                curseur.execute(query)
                queueBdd.put(ligne)
                BDD.commit()
    BDD.close()

def separePoints(listeTot):
    listeSplt=listeTot.split("!")
    listeDest=np.zeros((len(listeSplt),2))
    for i in range(len(listeSplt)):
        listeCoord=convertListeStr(listeSplt[i])
        listeDest[i][0]=listeCoord[0]
        listeDest[i][1]=listeCoord[1]
    return listeDest

def convertListeStr(stri):
    liste=stri.split(";")
    list2=np.zeros(len(liste))
    for i in range(len(liste)):
        list2[i]=float(liste[i])
    return list2

def interpreterBDD(ligne):
    global executionDep, path, Xactuel, Yactuel, currentAngle, heureOrdre
    print(ligne)
    if ligne[2]==0:
        if ligne[3]==3:
            listCoord=convertListeStr(ligne[4])
            Xactuel=listCoord[0]
            Yactuel=listCoord[1]
            currentAngle=listCoord[2]
            ## Annonce bdd ordre exécuter
            BDD=connect.connect(host="192.168.4.1",user="root",password="rpiSMR2",database="projet")
            curseur=BDD.cursor()
            curseur.execute("UPDATE ordre SET etat=4 WHERE destinataire = 3 AND etat = 1 ORDER BY heure LIMIT 1")
            BDD.commit()
            BDD.close()
        elif ligne[3]==4:
            path=separePoints(ligne[4])
            path=convertPath(path)
            print(path)
            executionDep=True
            heureOrdre=ligne[0]
    elif ligne[2]==2:
        executionDep=False
    elif ligne[2]==3:
        executionDep=False
        heureOrdre=0
        path=np.array(([]))

def signalementObstacleBDD(Xactuel, Yactuel):
    BDD=connect.connect(host="192.168.4.1",user="root",password="rpiSMR2",database="projet")
    curseur=BDD.cursor()
    query="INSERT INTO ordre (heure,destinataire,etat,typeOrdre,syntaxeOrdre) VALUES (NOW(),%s,%s,%s,%s)"
    syntaxe=str(Xactuel)+";"+str(Yactuel)
    data=(0,0,5,syntaxe)
    curseur.execute(query,data)
    BDD.commit()
    BDD.close()

##Définition fonction gyroscope
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
    
def initMPU():
    bus.write_byte_data(address,power_mgmt_1,0)
    time.sleep(0.5)
    
def read_byte(reg):
    return bus.read_byte_data(address,reg)

def read_word(reg):
    h=bus.read_byte_data(address,reg)
    l=bus.read_byte_data(address,reg+1)
    value=(h<<8)+l
    return value

def read_word_2c(reg):
    val=read_word(reg)
    if(val>=0x8000):
        return -((65535-val)+1)
    else:
        return val
    
def get_y_rotation(x,y,z):
    radians=math.atan2(x,dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians=math.atan2(y,dist(x,z))
    return math.degrees(radians)

def gyroscope():
    global GxCal, GyCal, GzCal
    Gx=read_word_2c(0x43)/131.0-GxCal
    Gy=read_word_2c(0x45)/131.0-GyCal
    Gz=read_word_2c(0x47)/131.0-GzCal

def acceleration():
    global AxCal, AyCal, AzCal
    Ax=(read_word_2c(0x3B)/16384.0-AxCal)
    Ay=(read_word_2c(0x3D)/16384.0-AyCal)
    Az=(read_word_2c(0x3F)/16384.0-AzCal)
    teta_x=get_x_rotation(Ax,Ay,Az)
    #teta_y=get_y_rotation(Ax,Ay,Az)
    return teta_x
    
def getYaw():
    global GzCal
    Gz=0
    #GzCal=-1.9
    for i in range(300):
        Gz=Gz+read_word_2c(0x47)/131.0-GzCal
    Gz=Gz/300
    print("Gz="+str(Gz))
    return Gz

def calibrateMPU():
    global GxCal, GyCal, GzCal
    x=0
    y=0
    z=0
    for i in range(10000):
        #x=x+read_word_2c(0x43)
        #y=y+read_word_2c(0x45)
        z=z+read_word_2c(0x47)
    #x=x/100
    #y=y/100
    z=z/10000
    #GxCal=x/131.0
    #GyCal=y/131.0
    GzCal=z/131.0
    print(GzCal)

##Calcul de position
def calculPosition(vitesseG, vitesseD):
    global Xactuel, Yactuel, TcalculPos, currentAngle, moteurGPositif, moteurDPositif
    rayonRoue=0.04  #diamètre 80mm
    coeffG=1
    coeffD=1
    if(moteurGPositif==False):
        coeffG=-1
    if(moteurDPositif==False):
        coeffD=-1
    t=time.time_ns()
    vitesseRobot=((vitesseG*coeffG+vitesseD*coeffD)*2*math.pi/60)*rayonRoue/2 #m/s
    distance=vitesseRobot*(t-TcalculPos)/1e9
    TcalculPos=t
    Xactuel=Xactuel+distance*math.cos(math.radians(currentAngle))
    Yactuel=Yactuel+distance*math.sin(math.radians(currentAngle))
    #print(math.cos(currentAngle))
    #print(math.sin(currentAngle))
    #print(distance)
    #print('#####')
## suivi de trajectoire
def convertPath(chemin):
    for k in range(len(chemin)):
        chemin[k][0]=0.05*chemin[k][0]
        chemin[k][1]=0.05*chemin[k][1]
    return chemin

def enlevePointAtteint(path):
    if(len(path)>1):
        newPath=np.zeros((len(path)-1,2))
        for i in range(len(newPath)):
            newPath[i][0]=path[i+1][0]
            newPath[i][1]=path[i+1][1]
    else:
        newPath=np.array(([]))
    return newPath
        
def isPositionOK(path):
    global Xactuel, Yactuel
    Xok=False
    Yok=False
    positionOK=False
    if path.any()==False :
        positionOK=True
    else :
        if(Xactuel<=path[0][0]+0.0075 and Xactuel>=path[0][0]-0.005):
            Xok=True
        if(Yactuel<=path[0][1]+0.0075 and Yactuel>=path[0][1]-0.005):
            Yok=True
        if(Xok and Yok):
            positionOK=True
    return positionOK    

def trajetFini(path):
    global executionDep, heureOrdre
    fini=False
    if(path.any()==False):
        executionDep=False
        fini=True
        if(heureOrdre!=0):
            BDD=connect.connect(host="192.168.4.1",user="root",password="rpiSMR2",database="projet")
            curseur=BDD.cursor()
            curseur.execute("UPDATE ordre SET etat=4 WHERE destinataire = 3 AND etat = 1 AND typeOrdre=4 ORDER BY heure LIMIT 1")
            BDD.commit()
            BDD.close()
            heureOrdre=0
    return fini
        
def calculTempsTeta(path, speedMS):
    global Xactuel, Yactuel, currentAngle
    x=path[0][0]-Xactuel
    y=path[0][1]-Yactuel
    dis=dist(x,y)
    t=dis/speedMS
    if x!=0:
        teta=math.degrees(math.atan2(y,x))
    else :
        if y>=0:
            teta=90
        else:
            teta=-90
    print('teta :'+str(teta))
    return t,teta

def rotation(teta_a_atteindre, speedRPM):
    global currentAngle
    if teta_a_atteindre <=180:
        print('C1')
        if(teta_a_atteindre-currentAngle>1):
            delta_teta=teta_a_atteindre-currentAngle
            turn_right(speedRPM+20,teta_a_atteindre)
            stop()
            print('C1-1')
        elif(teta_a_atteindre-currentAngle<-1):
            delta_teta=currentAngle-teta_a_atteindre
            turn_left(speedRPM+20,teta_a_atteindre)
            stop()
            print('C1-2')
    elif teta_a_atteindre>180:
        print('C2')
        if (currentAngle-teta_a_atteindre>1):
            delta_teta=currentAngle-teta_a_atteindre
            turn_left(speedRPM+20,teta_a_atteindre)
            stop()
            print('C2-1')
        elif(currentAngle-teta_a_atteindre<-1):
            delta_teta=teta_a_atteindre-currentAngle
            turn_right(speedRPM+20,teta_a_atteindre)
            stop()
            print('C2-2')
    return True

            
speedMoyRPM=40 #rpm
speedMoyMS=(speedMoyRPM*2*math.pi/60)*0.04 #m/s


##Initialisation des pins
def init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(moteurGIn1,GPIO.OUT)
    GPIO.setup(moteurGIn2,GPIO.OUT)
    GPIO.setup(moteurDIn1,GPIO.OUT)
    GPIO.setup(moteurDIn2,GPIO.OUT)
    GPIO.setup(moteurGpwm,GPIO.OUT)
    GPIO.setup(moteurDpwm,GPIO.OUT)
    GPIO.setup(moteurGEncoder1,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(moteurGEncoder2,GPIO.IN)
    GPIO.setup(moteurDEncoder1,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(moteurDEncoder2,GPIO.IN)
    initMPU()
    calibrateMPU()
    time.sleep(0.5)
    GPIO.add_event_detect(moteurGEncoder1,GPIO.BOTH,callback=frontG)
    GPIO.add_event_detect(moteurDEncoder1,GPIO.BOTH,callback=frontD)

tpsDep=0
dernierTpsBDD=0
rotationFini=False
##boucle infinie
init()
pwmG=GPIO.PWM(moteurGpwm,100)
pwmD=GPIO.PWM(moteurDpwm,100)
pwmG.start(0)
pwmD.start(0)
#faire un multiprocess pour la bdd toutes les 5 secs

#while executeDep=True autorise le déplacement du robot
if __name__=='__main__':
    queueBDD=multiprocessing.Queue()
    process=multiprocessing.Process(target=recupererBDD,args=(queueBDD,))
    while(process.is_alive()):
        time.sleep(0.1)
    dernierTpsBDD=time.time()
    print(Xactuel)
    print(Yactuel)
    print(currentAngle)
    if(queueBDD.empty()==False):
        interpreterBDD(queueBDD.get())
    while True:
        #time.sleep(1)
       # print('new boucle')
        #print(process.is_alive())
        #print(Xactuel)
        #print(Yactuel)
        #print(currentAngle)
        if(queueBDD.empty()==False):
            #si je n'ai pas de message de la bdd, je ne fais rien
            interpreterBDD(queueBDD.get())
            print('interpreter')
        if(executionDep==False):
            #si mon déplacement n'est pas autorisé je me stop
            correcteurG()
            correcteurD()
            calculPosition(vitesseG,vitesseD)
            stop()
            rotationFini=False #pour la reprise après une pause de bdd
        if(path.any() and executionDep and (rotationFini==False)):
            # si j'ai un chemin à effectué et que mon déplacement est autorisé
            # et que je ne me suis pas déjà tourner
            # je calcule l'angle à atteindre ainsi que le temps de déplacement
            tpsDep,teta=calculTempsTeta(path,speedMoyMS)
            #je fais ma rotation
            print('rotate :'+str(teta))
            rotationFini=rotation(teta,speedMoyRPM)
            #je réinitialise mes compteur pour le correcteur
            time.sleep(0.1)
            posG=0
            posD=0
            previousPosG=0
            previousPosD=0
            previoustG=time.time_ns()
            previoustD=time.time_ns()
            moyenneG=np.zeros(3)
            moyenneD=np.zeros(3)
            #correcteurG()
            #correcteurD()
            #je lance mon déplacement
            heureDep=time.time()
            forward(speedMoyRPM)
        if(path.any() and executionDep and (time.time()-heureDep<tpsDep+0.1)):
            # si je n'ai pas fait le temps de déplacement nécéssaire,
            # je corrige mes vitesse et calcule ma position
            print('dep en cours')
            correcteurG()
            correcteurD()
            calculPosition(vitesseG,vitesseD)
            print(Xactuel)
            print(Yactuel)
            print(currentAngle)
            #time.sleep(0.05)
        if(path.any() and executionDep and (time.time()-heureDep>tpsDep+0.1)):
            # si j'ai dépassé mon temps de déplacement, je calcule mes vitesses et ma position
            correcteurG()
            correcteurD()
            stop()
            calculPosition(vitesseG,vitesseD)
            print("pause")
            # je réinitialise mon flag de rotation            
            rotationFini=False
            if(isPositionOK(path)):
                print('pos ok')
                # si j'ai atteint ma position avec la précision nécessaire
                # je définit mon nouveau chemin total
                path=enlevePointAtteint(path)
                # je regarde sur le trajet est fini
                test=trajetFini(path) # a réaliser l'appel à la bdd si nécessaire
                if(test==True):
                    # si tous les déplacements sont faits, j'annule mon autorisation de déplacement
                    # il est autorisé à l'appel de la bdd si je dois de nouveau me déplacer
                    executionDep=False
        if(time.time()>dernierTpsBDD+5):
            #appel bdd toutes les 5 sec
            print('lecture bdd')
            recupererBDD(queueBDD)
            #process=multiprocessing.Process(target=recupererBDD,args=(queueBDD,))
            dernierTpsBDD=time.time()
        time.sleep(0.05)