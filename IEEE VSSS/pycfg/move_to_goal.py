import time
import argparse

parser = argparse.ArgumentParser(description='BahiaRT_V3S')
parser.add_argument('--env', default='simulation') # real or simulation
args = parser.parse_args()

# Criando comunicação ----------------------------------------------------------

# Para o teste real
if args.env == 'real':
    
    from vision import visionReal
    from comm import serial

    # Criando comunicação
    vision = visionReal.VisionReal()
    serial_comm = serial.SerialComm()
    serial_comm.start()

# Para o teste simulado
else:
    
    from vision import visionSim

    # Criando comunicação
    vision = visionSim.VisionSim()

vision.start()# Inicializando comunicação

# Criando entidades -----------------------------------------------------

from entities.Robot import Robot
from entities.Ball import Ball
from strategy import clever_trick

robot = Robot(
    clever_trick.CleverTrick(consider_back=False),
    env=args.env,
    team_color=True,   
)
ball = Ball(
    env=args.env  
)

# Criando campo potencial -----------------------------------------------------

from strategy import univector_field

pot_field = univector_field.MoveToGoalField(
    home_point = [0, 0],
    env = args.env
)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    
    while True:

        time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
        
        if vision.frame:
                
            print('Vision data received')
            
            while True:
                
                time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
                
                # Atualizando informações
                robot.update(vision.frame)
                ball.update(vision.frame)
                
                #Campo potencial
                pot_field.update_home_point(ball.position)
                
                phi = pot_field.compute(
                    robot.position[0] - pot_field.home_point[0],
                    robot.position[1] - pot_field.home_point[1]
                )
                
                print('phi:', phi)
                
                robot.set_desired(
                    univector_field.Nh(phi) * 2000
                )
                
                print('wl:',robot.wl)
                print('wr:',robot.wr)
                print(' ')
                
                # Enviando informações via Serial
                if args.env == 'real':
                    serial_comm.send(
                        [
                            {
                                'robot_id': robot.robot_id,
                                'wheel_left': robot.wl,
                                'wheel_right': robot.wr,
                                'color': robot.team_color_str()
                            }
                        ]
                    )
                else:
                    vision.send_data(robot)
     
        else:
            print('Waiting for vision data...')
            time.sleep(3)