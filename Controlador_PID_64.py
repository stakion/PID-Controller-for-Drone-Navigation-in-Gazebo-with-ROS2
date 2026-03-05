# Uso de las librerias correspondientes.
########################################################################################################################################
import pandas as pd
import subprocess
import time
import glob
import json
import math
import os
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# Iniciar comunicacion con ROS2.
rclpy.init(args=None)
########################################################################################################################################


# Creación de la función para preguntar si existe o no la carpeta correspondiente.
########################################################################################################################################
## Devuelve la respuesta a la pregunta si existe la carpeta que se considera o no.
def Validate_Folder( aux_s_00 ):
    try:	
        os.stat( aux_s_00 )
        return True
    except:	
        os.mkdir( aux_s_00 )
        return False
########################################################################################################################################


# Validar ubicacion de forma manual.
# Verificacion y cambio de directorio de ser posible ya que presenta algunos bugs en ubuntu la ubicaicion.
########################################################################################################################################
directorio_original = os.getcwd()                               ## Guarda el directorio original
print("Directorio original:", directorio_original)

''' Si se presenta bug de workspace.
os.chdir('..')                                                  ## Cambia a un nuevo directorio (ejemplo: directorio padre)
print("Directorio después de cambiar:", os.getcwd() )

os.chdir("./PRACTICA_1A")                                       ## Vuelve al directorio original
print("Vuelta al directorio original:", os.getcwd() )
'''

# os.chdir("./Desktop/MAESTRIA/2_SEMESTRE/Introducción a la Robotica/PRACTICA_1A")
os.getcwd()                                                     ## Verificacion del directorio actual.



## Definición de los nombres para las carpetas a considerar.
### Preguntar si existen cada una de las carpetas de control con la función previamente definida.
s_aux_1_DP = "DESIRED_POSITION";        Validate_Folder( s_aux_1_DP )
s_aux_2_FP = "FINAL_POSITION";          Validate_Folder( s_aux_2_FP )
s_aux_4_GP = "GRAPHICS";                Validate_Folder( s_aux_4_GP )
########################################################################################################################################


# Funcion para calcular la distancia del punto actual vs la posicion deseada.
########################################################################################################################################
def distancia(punto1, punto2):
    x1, y1, z1 = punto1
    x2, y2, z2 = punto2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2 )
########################################################################################################################################


# Clase para obtener la posicion del objeto en linea de comandos ros2.
########################################################################################################################################
class Position_Drone(Node):
    def __init__(self, duration: float):
        super().__init__('Position_Drone')
        self.topic_to_suscribe = "/simple_drone/gt_pose"
        self.subscription = self.create_subscription(Pose, self.topic_to_suscribe, self.listener_callback, 20)

        # Controlar la vida del nodo
        self.alive = True  

        # Inicializacion de arreglos auxiliares para almacenar las coordenadas x,y,z. 
        self.a_aux_x = []
        self.a_aux_y = []
        self.a_aux_z = []

        # Creacion del archivo csv y asignacion de los headers correspondientes a cada columna.
        ## Consideracion de la ubicacion completa con todo y carpeta.
        self.csv_path = "CURRENT_POSITION"
        self.csv_file = 'CURRENT_POSITION.csv'
        self.csv_file_path = './' + str(self.csv_path) + '/' + str(self.csv_file) 

        # Validacion de la existencia de la carpeta correspondiente.
        Validate_Folder( self.csv_path )

        # Preparar un DataFrame vacío con las columnas correctas si el archivo no existe
        ## Deteccion del archivo hecha con glob.
        self.a_aux_current_position_file = glob.glob('./' + str(self.csv_path) + '**/*.csv')
        
        # Se pregunta si no se detecta con la expresion regular de forma recursiva ningun archivo csv.
        ### Es mas rapido crear un archivo vacio unicamente con los headers en las columnas y posteriormente reescribirlo.
        if( len(self.a_aux_current_position_file) == 0):
            self.df = pd.DataFrame(columns=['x', 'y', 'z'])
            self.df.to_csv(self.csv_file_path, index=False)

        else:
            # Cargar el archivo existente para evitar sobrescribirlo
            self.df = pd.read_csv(self.csv_file_path)

        # Configura un temporizador para terminar el nodo después del tiempo especificado
        self.create_timer(duration, self.terminate_node)

    #--- Subscriber Part ---
    def listener_callback(self, msg):                                            
        position = msg.position
        #time.sleep(0.001)

        # Se concatenan los resultados en el arreglo correspondiente a cada variable. 
        self.a_aux_x.append(position.x)
        self.a_aux_y.append(position.y)
        self.a_aux_z.append(position.z)
                                             
    def terminate_node(self):
        # Cada arreglo auxiliar se convierte a un dataframe de una unica columna.
        self.df_x = pd.DataFrame( self.a_aux_x , columns=['x'])
        self.df_y = pd.DataFrame( self.a_aux_y , columns=['y'])
        self.df_z = pd.DataFrame( self.a_aux_z , columns=['z'])

        # Una vez terminando el proceso se almacena toda la información en un dataframe.
        self.df_ap = pd.concat([self.df_x , self.df_y, self.df_z], axis=1)

        # Se almacena el dataframe en la ubicación final.
        self.df_ap.to_csv(self.csv_file_path ,index=False, encoding="utf-8-sig")

        # Cambiar el estado para terminar el bucle de eventos
        self.alive = False                                                                                              

########################################################################################################################################


# Funcion que considera el archivo csv con la posicion actual que se va actualizando, pero permite devolver el promedio sobre la posicion a lo largo del tiempo muestreado
## Devuelve una tupla en forma x,y,z con los promedios de cada dimension.
########################################################################################################################################
def Get_Average_Actual_Position_ROS2():
    aux_Position_Drone = Position_Drone(0.25)                                            # 0.25 segundos de captura de datos funcionan bien.
    
    while rclpy.ok() and aux_Position_Drone.alive:          
        rclpy.spin_once(aux_Position_Drone)                                             # Bucle para mantener el nodo activo hasta que 'alive' sea False
    aux_df = pd.read_csv( aux_Position_Drone.csv_file_path )                            # Considerar la ubicacion correspondiente del archivo csv con la ubicacion original.

    # Consideracion de los promedios correspondientes de cada una de las 3 columnas. 
    return aux_df['x'].mean(), aux_df['y'].mean(), aux_df['z'].mean()

# Prueba de la funcion para obtener la posicion promedio en base al archivo que se actualiza de forma constante.
Get_Average_Actual_Position_ROS2()
# time.sleep(1)
########################################################################################################################################


# Clase copiada del paquete keyboard, y probada para adaptar a usar unicamente los avances correspondientes en cada direccion, detenimientos, despegue y aterrizaje.
########################################################################################################################################
class Control_Drone (Node):
    def __init__(self):
        super().__init__('Control_Drone')
        self.pub_takeoff = self.create_publisher(Empty,'/simple_drone/takeoff', 10)     # Topico para Despegar.
        self.pub_land = self.create_publisher(Empty, '/simple_drone/land', 10)          # Topico para Aterrizar.
        self.pub_cmd_vel = self.create_publisher(Twist,'/simple_drone/cmd_vel', 10)     # Topico para enviar la velocidad en cada eje y angular.
        self.pub_override = self.create_publisher(Int8, '/keyboard/override', 10)       # Topico para interrumpir el teclado.
        self.vel_msg = Twist()
        self.OvR_msg = 0
        self.alive = True

    # Funcion para despegar el dron. 
    def D_Takeoff(self):
        msg = Empty()
        self.pub_takeoff.publish(msg)

    # Funcion para aterrizar el dron. 
    def D_Land(self):
        msg = Empty()
        self.pub_land.publish(msg)

    # Funcion para detener el dron. 
    def D_Stop(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.vel_msg)
    
    # Movimiento en una única direccion seleccionada.
    def D_Move_One(self, direction:str, velocity:float):
        self.vel_msg.linear.x = float( int(direction == 'X' or direction == 'x' ) )*round(velocity,2)
        self.vel_msg.linear.y = float( int(direction == 'Y' or direction == 'y' ) )*round(velocity,2)
        self.vel_msg.linear.z = float( int(direction == 'Z' or direction == 'z' ) )*round(velocity,2)
        self.vel_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.vel_msg)

    # Movimiento en las 3 direcciones de forma simultanea.
    def D_Move_All(self, v_x:float, v_y:float, v_z:float):
        self.vel_msg.linear.x = round(v_x,2)
        self.vel_msg.linear.y = round(v_y,2)
        self.vel_msg.linear.z = round(v_z,2)
        self.vel_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.vel_msg)

    # Funcion para iniciar el control del dron. 
    def D_Begin_Control(self):
        int8_msg = Int8()
        int8_msg.data = 10
        self.pub_override.publish(int8_msg)

    # Funcion para finalizar el control del dron.
    def D_End_Control(self):
        int8_msg = Int8()
        int8_msg.data = 6
        self.pub_override.publish(int8_msg)

    # Funcion para finalizar el nodo creado.
    def terminate_node(self):
        # Cambiar el estado para terminar el bucle de eventos
        self.alive = False                                                                                              
########################################################################################################################################


# Funcion para controlar el despegue del dron.
## No inicializar ni terminar cada nodo si no hasta el final de su uso
########################################################################################################################################
def Action_Bailesito_Prueba():
    aux_Control_Subdrone = Control_Drone()
    aux_Control_Subdrone.D_Takeoff()
    Get_Average_Actual_Position_ROS2()
    time.sleep(1)
    aux_Control_Subdrone.D_Move_One('X', -0.8)
    Get_Average_Actual_Position_ROS2()
    time.sleep(1)
    aux_Control_Subdrone.D_Stop()
    Get_Average_Actual_Position_ROS2()
    time.sleep(1)
    aux_Control_Subdrone.D_Land()
    Get_Average_Actual_Position_ROS2()
    time.sleep(3)
    aux_Control_Subdrone.D_Takeoff()
    Get_Average_Actual_Position_ROS2()
    time.sleep(1)
    aux_Control_Subdrone.D_Move_One('X', 0.8)
    Get_Average_Actual_Position_ROS2()
    time.sleep(1)
    aux_Control_Subdrone.D_Stop()
    Get_Average_Actual_Position_ROS2()
    time.sleep(1)
    aux_Control_Subdrone.D_Land()

# Prueba de funcionamiento conexion a Gazebo. 
Action_Bailesito_Prueba()
time.sleep(1)
print("Rutina de prueba de inicio concluida...")
########################################################################################################################################


# Funcion que considera el archivo csv con la posicion deseada, este mismo archivo no se va actualizando.
## Devuelve una tupla en forma x,y,z
########################################################################################################################################
def Get_Desired_Position():
    # Uso de la libreria glob para detectar de forma automatica el unico archivo presente en la carpeta DESIRE_POSITION.
    a_aux_desired_position_file = glob.glob('./' + str(s_aux_1_DP) + '**/*.csv')

    # Creacion del dataframe correspondiente en base al archivo leido.
    df_desired_position = pd.read_csv(a_aux_desired_position_file[0])

    # Asignacion de cada una de las variables en x,y,z.
    desired_position_x, desired_position_y, desired_position_z = df_desired_position.iloc[0]

    # Devuelta de la tupla correspondiente, considerando que el archivo no cambia.
    return desired_position_x, desired_position_y, desired_position_z

# Cabe destacar que se considero la posicion del bloque con el numero
Get_Desired_Position()
########################################################################################################################################


## Generación Automática de 5 gráficas donde se comparan:
# Posicion vs Numero de Iteraciones.
# Velocidad vs Numero de Iteraciones.
# Error vs Numero de Iteraciones.
# Error Acumulado vs Numero de Iteraciones.
# Error Previo vs Numero de Iteraciones.
# Ejes : (x - rojo, y - verde, z - azul)
########################################################################################################################################
def MULTI_GRAPH( A_x, A_y, A_S2 , A_S3, A_S4, A_S5):
	''' A_x : Arreglo en el eje independiente																		'''
	''' A_y : Arreglo con los arreglos a graficar en la parte del eje dependiente.                                  '''
	''' A_S2 : String auxiliar para eje y.																			'''
	''' A_S3 : String auxiliar para título gráfica.																	'''
	''' A_S4 : Ruta para almacenar la gráfica.																		'''
	''' A_S5 : String auxiliar para tomar en cuenta el label de la gráfica.											'''
	with plt.style.context(('dark_background')):

		for k in range(5):
			plt.figure( num = None, dpi = 1400, facecolor = 'w', edgecolor = 'r' )
			plt.xlabel( " Numero de iteraciones: (i)" )
			plt.ylabel( A_S2[k] )
			plt.title( A_S3[k] , fontsize = 12 )
			plt.grid( True )
			plt.grid( color = '0.5', linestyle = '--', linewidth = 0.3 )
			plt.plot( A_x, A_y[k][0], 'r.', linewidth = 0.20, label = A_S5[k][0] )
			plt.plot( A_x, A_y[k][1], 'g.', linewidth = 0.20, label = A_S5[k][1] )
			plt.plot( A_x, A_y[k][2], 'b.', linewidth = 0.20, label = A_S5[k][2] )
			plt.legend( loc = 'upper right' )
			plt.savefig( A_S4[k] , bbox_inches = 0, dpi = 1400 )
			plt.close() 
########################################################################################################################################


########################################################################################################################################
# Posiciones deseadas de los cubos presentes en la simulacion con un offset en z de +3
# 1 : -3.44 , -9.53 ,  3.4  # CAPTURADO.
# 2 : -9.34 ,  5.92 ,  3.4  # CAPTURADO - VIDEO.
# 3 : 4.34  ,  9.36 ,  3.4  # CAPTURADO.
# 4 : 9.51  , -4.51 ,  3.4  # CAPTURADO.
########################################################################################################################################


# Incializacion de los parametros.
########################################################################################################################################
# Constantes PID para cada dimensión (ajustar según sea necesario)
k_p, k_i, k_d = 1, 0.0001, 0.01

## Inicio de los valores asociados a la derivada, integral y ajustes.
error_previo_x, error_previo_y, error_previo_z = 0, 0, 0
integral_x, integral_y, integral_z = 0, 0, 0
ajuste_x, ajuste_y, ajuste_z = 0, 0, 0
error_x, error_y, error_z = 0, 0, 0
diff_x, diff_y, diff_z = 0, 0, 0

# Incio del parametro delta de tiempo a considerar.
dt = 0.01

# Parametro de velocidad maxima a considerar
velocidad_maxima = 0.3

# Inicialización de los arreglos para considerar las actualizaciones de la posicion final.
A_aux_i = [];       A_aux_Kp = [];          A_aux_Ki = [];               A_aux_Kd = [];             A_aux_D = []
A_aux_F_x = [];     A_aux_error_x = [];     A_aux_integral_x = [];       A_aux_diff_x = [];         A_aux_ajuste_x = []  
A_aux_F_y = [];     A_aux_error_y = [];     A_aux_integral_y = [];       A_aux_diff_y = [];         A_aux_ajuste_y = []
A_aux_F_z = [];     A_aux_error_z = [];     A_aux_integral_z = [];       A_aux_diff_z = [];         A_aux_ajuste_z = []

# Posicion actual, primera iteracion
## Deteccion de la posicion actual haciendo uso de un archivo csv.
c_p_000 = Get_Average_Actual_Position_ROS2()

# Iniciar una instancia para la clase que permita controlar el dron. 
aux_Control_Drone = Control_Drone()

# Despegar el dronsito
aux_Control_Drone.D_Takeoff()
time.sleep(1)
aux_Control_Drone.D_Stop()

# Asignaciones individuales de la posicion actual en cada dimension.
current_position_x = c_p_000[0]
current_position_y = c_p_000[1]
current_position_z = c_p_000[2]

# Actualizacion de la tupla para representar la distancia actual.
v_current_p = c_p_000
print( "La posicion en x,y,z del punto actual es de: " + "( " + str(current_position_x) + " , " + str(current_position_y) + " , " + str(current_position_z) + " )" )

# Posicion deseada, primera iteracion y unica.
v_desired_p = Get_Desired_Position()
desired_position_x, desired_position_y, desired_position_z = v_desired_p
print( "La posicion en x,y,z del punto deseado es de: " + "( " + str(desired_position_x) + " , " + str(desired_position_y) + " , " + str(desired_position_z) + " )" )

# Actualizacion de la distancia a la posicion deseada.
v_aux_d = distancia(v_current_p, v_desired_p)

# Distancia actual al punto deseado. 
print("La distancia desde el punto actual al punto deseado es de: " + str(v_aux_d) )
########################################################################################################################################


# Ciclo while correspondiente al controlador PID.
## Se debe de trabajar con la ventana del keyboard abierta.
########################################################################################################################################
c_i = 0
while(v_aux_d >= 0.01):
    # Se añaden los valores correspondientes de x,y,z a los arreglos que se consideraran como parte del dataframe final.
    A_aux_i.append(c_i);                        A_aux_Kp.append(k_p);               A_aux_Ki.append(k_i);                       A_aux_Kd.append(k_d);               A_aux_D.append(v_aux_d)                       
    A_aux_F_x.append(current_position_x);       A_aux_error_x.append(error_x);      A_aux_integral_x.append(integral_x);        A_aux_diff_x.append(diff_x);        A_aux_ajuste_x.append(ajuste_x)    
    A_aux_F_y.append(current_position_y);       A_aux_error_y.append(error_y);      A_aux_integral_y.append(integral_y);        A_aux_diff_y.append(diff_y);        A_aux_ajuste_y.append(ajuste_y)   
    A_aux_F_z.append(current_position_z);       A_aux_error_z.append(error_z);      A_aux_integral_z.append(integral_z);        A_aux_diff_z.append(diff_z);        A_aux_ajuste_z.append(ajuste_z)  

    ## ZONA DE CALCULOS RELACIONADOS AL CONTROLADOR PID
    ###############################################################################################################################################################################################
    # Calcular el error para cada dimensión.
    error_x = desired_position_x - current_position_x
    error_y = desired_position_y - current_position_y
    error_z = desired_position_z - current_position_z

    # Calculo de la derivada
    diff_x = (error_x - error_previo_x)/dt
    diff_y = (error_y - error_previo_y)/dt
    diff_z = (error_z - error_previo_z)/dt

    # Ajuste debido al PID.
    ajuste_x = k_p*error_x + k_i*integral_x + k_d*diff_x
    ajuste_y = k_p*error_y + k_i*integral_y + k_d*diff_y
    ajuste_z = k_p*error_z + k_i*integral_z + k_d*diff_z

    # Limitacion de la velocidad maxima considerando la condicion de umax = 0.5
    # Condicion para evitar divirgencias en el movimiento del dron.
    if( ajuste_x**2 + ajuste_y**2 + ajuste_z**2 < velocidad_maxima**2 ):
        integral_x += error_x*dt
        integral_y += error_y*dt
        integral_z += error_z*dt

    # Envio de las velocidades de ajuste en cada direccion de forma simultanea.
    aux_Control_Drone.D_Move_All( ajuste_x , ajuste_y , ajuste_z )

    # Actualizar errores previos para la próxima iteración.
    error_previo_x = error_x
    error_previo_y = error_y
    error_previo_z = error_z
    ###############################################################################################################################################################################################

    # Impresión de la posición cada 10 iteraciones.
    if(c_i%10 == 0):
        print( "X: " + str(current_position_x) + " , " + "Y: " + str(current_position_y) + " , " + "Z: " + str(current_position_z))
        print( "La distancia actual al punto es de : " + str(v_aux_d) )

    # Se actualiza la posición actual
    c_p_000 = Get_Average_Actual_Position_ROS2()
    
    # Asignaciones individuales de la posicion actual en cada dimension.
    current_position_x = c_p_000[0]
    current_position_y = c_p_000[1]
    current_position_z = c_p_000[2]

    # Actualizacion de la tupla para representar la distancia actual.
    v_current_p = c_p_000

    # Actualizacion de la distancia a la posicion deseada.
    v_aux_d = distancia(v_current_p, v_desired_p)

    # Espera 1/100 segundo antes de la próxima actualización
    time.sleep(dt)  
    c_i += 1

# Impresion de la posicion final.
if(v_aux_d < 0.01):
    print("Se llego al punto deseado o aproximado : " + "X: " + str(current_position_x) + " , " + "Y: " + str(current_position_y) + " , " + "Z: " + str(current_position_z) )
    print("La distancia actual al punto es de : " + str(v_aux_d) )
    print("Cantidad de iteraciones: " + str(c_i) )

    # Detener el dronsito.
    aux_Control_Drone.D_Stop()
    time.sleep(0.5)
    aux_Control_Drone.D_Land()
########################################################################################################################################


# Una vez terminando el proceso se almacena toda la información en un dataframe.
########################################################################################################################################
df_fp_i = pd.DataFrame( A_aux_i , columns=['I'])
df_fp_kp = pd.DataFrame( A_aux_Kp , columns=['K_P'])
df_fp_ki = pd.DataFrame( A_aux_Ki , columns=['K_I'])
df_fp_kd = pd.DataFrame( A_aux_Kd , columns=['K_D'])
df_fp_D = pd.DataFrame( A_aux_D , columns=['D'])
df_fp_x = pd.DataFrame( A_aux_F_x , columns=['X'])
df_fp_y = pd.DataFrame( A_aux_F_y , columns=['Y'])
df_fp_z = pd.DataFrame( A_aux_F_z , columns=['Z'])
df_fp_ex = pd.DataFrame( A_aux_error_x , columns=['E_X'])
df_fp_ey = pd.DataFrame( A_aux_error_y , columns=['E_Y'])
df_fp_ez = pd.DataFrame( A_aux_error_z , columns=['E_Z'])
df_fp_ix = pd.DataFrame( A_aux_integral_x , columns=['I_X'])
df_fp_iy = pd.DataFrame( A_aux_integral_y , columns=['I_Y'])
df_fp_iz = pd.DataFrame( A_aux_integral_z , columns=['I_Z'])
df_fp_dx = pd.DataFrame( A_aux_diff_x , columns=['D_X'])
df_fp_dy = pd.DataFrame( A_aux_diff_y , columns=['D_Y'])
df_fp_dz = pd.DataFrame( A_aux_diff_z , columns=['D_Z'])
df_fp_vx = pd.DataFrame( A_aux_ajuste_x , columns=['V_X'])
df_fp_vy = pd.DataFrame( A_aux_ajuste_y , columns=['V_Y'])
df_fp_vz = pd.DataFrame( A_aux_ajuste_z , columns=['V_Z'])

# Se concatenan los dataframes columnares en uno solo.
df_fp = pd.concat([df_fp_i, df_fp_kp, df_fp_ki, df_fp_kd, df_fp_D, df_fp_x , df_fp_y, df_fp_z, df_fp_ex, df_fp_ey, df_fp_ez, df_fp_ix, df_fp_iy, df_fp_iz, df_fp_dx, df_fp_dy, df_fp_dz, df_fp_vx, df_fp_vz, df_fp_vz], axis=1)

# Se almacena el dataframe en la ubicación final.
df_fp.to_csv(f'./' + str(s_aux_2_FP) + '/FINAL_POSITION.csv',index=False, encoding="utf-8-sig") 
########################################################################################################################################


# Inicio de los parámetros relacionados a las gráficas.
########################################################################################################################################
A_aux_0Y =  [ [ A_aux_F_x , A_aux_F_y , A_aux_F_z] , [ A_aux_ajuste_x , A_aux_ajuste_y , A_aux_ajuste_z ] , [ A_aux_error_x , A_aux_error_y , A_aux_error_z ] , 
                [ A_aux_integral_x , A_aux_integral_y , A_aux_integral_z ] , [ A_aux_diff_x , A_aux_diff_y , A_aux_diff_z] ]

A_aux_S2 =  [ "Posicion en los ejes correspondientes (m)" , "Velocidades en los ejes correspondientes (m/s)" , "Error en los ejes correspondientes" , 
                "Integrales correspondientes a los ejes" , "Derivadas correspondientes a los ejes"]

A_aux_S3 =  [ "Posición con respecto a Iteraciones, 3 ejes" , "Velocidad con respecto a Iteraciones, 3 ejes" , "Error con respecto a Iteraciones, 3 ejes" , 
                "Integral con respecto a Iteraciones, 3 ejes" , "Derivada con respecto a Iteraciones, 3 ejes"]

A_aux_S4 =  [ f'./' + str(s_aux_4_GP) + '/P_vs_Ni.png' , f'./' + str(s_aux_4_GP) + '/V_vs_Ni.png' , f'./' + str(s_aux_4_GP) + '/E_vs_Ni.png' , 
                f'./' + str(s_aux_4_GP) + '/I_vs_Ni.png' , f'./' + str(s_aux_4_GP) + '/D_vs_Ni.png'] 

A_aux_S5 = [ [ "X" , "Y" , "Z"] , [ "X" , "Y" , "Z"] , [ "X" , "Y" , "Z"] , [ "X" , "Y" , "Z"] , [ "X" , "Y" , "Z"] ]

# Graficación de las correspondientes variables considerando el número de iteración.
MULTI_GRAPH( A_aux_i, A_aux_0Y , A_aux_S2 , A_aux_S3, A_aux_S4, A_aux_S5)
########################################################################################################################################


## Considerar images viewer para capturrar cada determinado tiempo la imagen.
## Csv donde se almacenaran las teclas presionadas en keyboard.
## Csv considerado en donde se toman en cuenta múltiples posiciones.


# Proceso de finalización del nodo y conexión de ROS2.
########################################################################################################################################
# El nodo se puede destruir unicamente si se llego a un punto cercano, si se destruye entonces se perdera toda la conexion y sera necesesario reiniciar todo.
## Destruccion de ambos nodos. 
#aux_Control_Subdrone.destroy_node()
#Position_Subdrone.destroy_node()

# Finalizar comunicacion con ROS2.
#rclpy.shutdown()
########################################################################################################################################









































































































































































































